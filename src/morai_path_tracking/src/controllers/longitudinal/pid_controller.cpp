#include "morai_path_tracking/controllers/longitudinal/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace morai_path_tracking {
namespace {

void requireFiniteNonNegative(const char* name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

void requireFinitePositive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

}  // namespace

LongitudinalPid::LongitudinalPid(const PidConfig& config) : config_(config) {
  requireFiniteNonNegative("kp", config_.kp);
  requireFiniteNonNegative("ki", config_.ki);
  requireFiniteNonNegative("kd", config_.kd);
  requireFiniteNonNegative("integral_limit", config_.integral_limit);
  requireFinitePositive("integral_unwind_rate_per_sec",
                        config_.integral_unwind_rate_per_sec);
  requireFiniteNonNegative("error_deadband_mps", config_.error_deadband_mps);
  requireFiniteNonNegative("accel_feedforward_gain_per_mps",
                           config_.accel_feedforward_gain_per_mps);
  requireFiniteNonNegative("coast_overspeed_threshold_mps",
                           config_.coast_overspeed_threshold_mps);
  requireFinitePositive("brake_overspeed_threshold_mps",
                        config_.brake_overspeed_threshold_mps);
  if (config_.brake_overspeed_threshold_mps <=
      config_.coast_overspeed_threshold_mps) {
    throw std::invalid_argument(
        "brake_overspeed_threshold_mps must be greater than "
        "coast_overspeed_threshold_mps");
  }
  requireFinitePositive("hard_brake_activation_speed_mps",
                        config_.hard_brake_activation_speed_mps);
  requireFiniteNonNegative("minimum_hard_brake_command",
                           config_.minimum_hard_brake_command);
  requireFiniteNonNegative("maximum_accel", config_.maximum_accel);
  requireFiniteNonNegative("maximum_brake", config_.maximum_brake);
  if (config_.minimum_hard_brake_command > config_.maximum_brake) {
    throw std::invalid_argument(
        "minimum_hard_brake_command must not exceed maximum_brake");
  }
  requireFiniteNonNegative("command_rate_limit_per_sec",
                           config_.command_rate_limit_per_sec);
}

LongitudinalCommand LongitudinalPid::update(double target_speed_mps,
                                            double measured_speed_mps,
                                            double dt_sec) {
  if (!std::isfinite(target_speed_mps) || !std::isfinite(measured_speed_mps) ||
      !std::isfinite(dt_sec) || dt_sec <= 0.0) {
    throw std::invalid_argument(
        "target speed, measured speed, and dt must be finite and dt positive");
  }

  const double error = target_speed_mps - measured_speed_mps;
  const double overspeed_mps = measured_speed_mps - target_speed_mps;
  const bool coast_requested =
      overspeed_mps >= config_.coast_overspeed_threshold_mps &&
      overspeed_mps < config_.brake_overspeed_threshold_mps;
  const bool brake_requested =
      overspeed_mps >= config_.brake_overspeed_threshold_mps;
  const bool hard_brake_requested =
      measured_speed_mps >= config_.hard_brake_activation_speed_mps;
  const bool inside_deadband =
      std::abs(error) <= config_.error_deadband_mps;
  const double derivative = !inside_deadband && has_previous_measurement_
                                ? (measured_speed_mps - previous_measurement_mps_) /
                                      dt_sec
                                : 0.0;
  const bool unwind_integral = inside_deadband || coast_requested;
  const double candidate_integral = unwind_integral
                                        ? std::copysign(
                                              std::max(0.0, std::abs(integral_) -
                                                                config_.integral_unwind_rate_per_sec *
                                                                    dt_sec),
                                              integral_)
                                        : clamp(integral_ + error * dt_sec,
                                                -config_.integral_limit,
                                                config_.integral_limit);
  const double feedforward =
      (coast_requested || brake_requested || hard_brake_requested)
          ? 0.0
          : config_.accel_feedforward_gain_per_mps *
                std::max(0.0, target_speed_mps);
  double candidate_output =
      feedforward + config_.kp * (inside_deadband ? 0.0 : error) +
      config_.ki * candidate_integral -
      config_.kd * derivative;
  if (coast_requested) {
    candidate_output = 0.0;
  } else if ((brake_requested || hard_brake_requested) &&
             candidate_output > 0.0) {
    candidate_output = 0.0;
  }

  const bool saturated_high = candidate_output > config_.maximum_accel;
  const bool saturated_low = candidate_output < -config_.maximum_brake;
  if (unwind_integral ||
      !((saturated_high && error > 0.0) ||
        (saturated_low && error < 0.0))) {
    integral_ = candidate_integral;
  }

  previous_measurement_mps_ = measured_speed_mps;
  has_previous_measurement_ = true;

  double signed_command =
      clamp(candidate_output, -config_.maximum_brake, config_.maximum_accel);
  if (config_.command_rate_limit_per_sec > 0.0) {
    const double maximum_change =
        config_.command_rate_limit_per_sec * dt_sec;
    signed_command =
        clamp(signed_command, previous_signed_command_ - maximum_change,
              previous_signed_command_ + maximum_change);
    if ((previous_signed_command_ > 0.0 && signed_command < 0.0) ||
        (previous_signed_command_ < 0.0 && signed_command > 0.0)) {
      signed_command = 0.0;
    }
  }
  if (hard_brake_requested) {
    signed_command =
        std::min(signed_command, -config_.minimum_hard_brake_command);
  }
  previous_signed_command_ = signed_command;

  const LongitudinalState state =
      hard_brake_requested
          ? LongitudinalState::kHardSpeedBrake
          : signed_command > 0.0
                ? LongitudinalState::kAccel
                : signed_command < 0.0 ? LongitudinalState::kBrake
                                       : LongitudinalState::kCoast;
  if (signed_command >= 0.0) {
    return {signed_command, 0.0, state};
  }
  return {0.0, -signed_command, state};
}

void LongitudinalPid::reset() {
  integral_ = 0.0;
  has_previous_measurement_ = false;
  previous_measurement_mps_ = 0.0;
  previous_signed_command_ = 0.0;
}

const char* longitudinalStateName(LongitudinalState state) noexcept {
  switch (state) {
    case LongitudinalState::kAccel:
      return "ACCEL";
    case LongitudinalState::kCoast:
      return "COAST";
    case LongitudinalState::kBrake:
      return "BRAKE";
    case LongitudinalState::kHardSpeedBrake:
      return "HARD_SPEED_BRAKE";
  }
  return "UNKNOWN";
}

}  // namespace morai_path_tracking
