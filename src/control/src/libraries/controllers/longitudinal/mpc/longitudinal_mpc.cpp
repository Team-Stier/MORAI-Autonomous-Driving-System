#include "control/libraries/tracking/controllers/longitudinal/mpc/longitudinal_mpc.hpp"
#include "control/libraries/tracking/common/osqp_sqp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace control { namespace tracking {
namespace {

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

LongitudinalMpc::LongitudinalMpc(const LongitudinalMpcConfig& config)
    : config_(config),
      warm_start_(static_cast<std::size_t>(config.horizon_steps), 0.0) {
  const bool solver_valid = config_.solver == "osqp" ||
                            config_.solver == "projected_gradient" ||
                            config_.solver == "coordinate_search";
  if (!solver_valid || config_.horizon_steps < 2 ||
      config_.optimizer_iterations <= 0 || config_.line_search_steps <= 0 ||
      !finitePositive(config_.prediction_dt_sec) ||
      !finitePositive(config_.gradient_epsilon) ||
      !finitePositive(config_.gradient_step) ||
      !std::isfinite(config_.gradient_tolerance) ||
      config_.gradient_tolerance < 0.0 ||
      !finitePositive(config_.coordinate_initial_step) ||
      !finitePositive(config_.coordinate_minimum_step) ||
      !finitePositive(config_.command_step_decay) ||
      config_.command_step_decay >= 1.0 ||
      !finitePositive(config_.maximum_speed_mps) ||
      !finitePositive(config_.hard_brake_activation_speed_mps) ||
      config_.hard_brake_activation_speed_mps > config_.maximum_speed_mps ||
      !finitePositive(config_.maximum_accel_command) ||
      config_.maximum_accel_command > 1.0 ||
      !finitePositive(config_.maximum_brake_command) ||
      config_.maximum_brake_command > 1.0 ||
      !finitePositive(config_.command_rate_limit_per_sec) ||
      !finitePositive(config_.acceleration_gain_mps2) ||
      !finitePositive(config_.braking_gain_mps2) ||
      !finitePositive(config_.actuator_time_constant_sec) ||
      !finitePositive(config_.maximum_acceleration_mps2) ||
      !finitePositive(config_.maximum_deceleration_mps2) ||
      !finitePositive(config_.maximum_jerk_mps3) ||
      !finitePositive(config_.acceleration_estimate_filter_time_constant_sec) ||
      !std::isfinite(config_.minimum_hard_brake_command) ||
      config_.minimum_hard_brake_command < 0.0 ||
      config_.minimum_hard_brake_command > config_.maximum_brake_command ||
      !finitePositive(config_.speed_weight) ||
      !std::isfinite(config_.acceleration_weight) ||
      config_.acceleration_weight < 0.0 ||
      !std::isfinite(config_.jerk_weight) || config_.jerk_weight < 0.0 ||
      !std::isfinite(config_.command_weight) || config_.command_weight < 0.0 ||
      !std::isfinite(config_.command_rate_weight) ||
      config_.command_rate_weight < 0.0 ||
      !finitePositive(config_.terminal_speed_weight)) {
    throw std::invalid_argument("invalid longitudinal MPC configuration");
  }
}

void LongitudinalMpc::project(std::vector<double>* commands) const {
  double previous = previous_signed_command_;
  const double maximum_step =
      config_.command_rate_limit_per_sec * config_.prediction_dt_sec;
  for (double& command : *commands) {
    command = clamp(command, -config_.maximum_brake_command,
                    config_.maximum_accel_command);
    command = clamp(command, previous - maximum_step,
                    previous + maximum_step);
    previous = command;
  }
}

LongitudinalMpc::RolloutResult LongitudinalMpc::rollout(
    const std::vector<double>& commands, double target_speed_mps,
    double measured_speed_mps) const {
  RolloutResult result;
  result.cost = 0.0;
  double speed = std::max(0.0, measured_speed_mps);
  double acceleration = estimated_acceleration_mps2_;
  double previous_command = previous_signed_command_;
  result.maximum_speed_mps = speed;
  const double dt = config_.prediction_dt_sec;
  const double actuator_alpha =
      1.0 - std::exp(-dt / config_.actuator_time_constant_sec);
  for (double command : commands) {
    const double requested_acceleration =
        command >= 0.0 ? config_.acceleration_gain_mps2 * command
                       : config_.braking_gain_mps2 * command;
    double next_acceleration =
        acceleration + actuator_alpha * (requested_acceleration - acceleration);
    next_acceleration =
        clamp(next_acceleration, -config_.maximum_deceleration_mps2,
              config_.maximum_acceleration_mps2);
    const double maximum_acceleration_change = config_.maximum_jerk_mps3 * dt;
    next_acceleration = clamp(next_acceleration,
                              acceleration - maximum_acceleration_change,
                              acceleration + maximum_acceleration_change);
    speed = std::max(0.0, speed + next_acceleration * dt);
    result.maximum_speed_mps = std::max(result.maximum_speed_mps, speed);
    const double speed_error = target_speed_mps - speed;
    const double jerk = (next_acceleration - acceleration) / dt;
    const double command_change = command - previous_command;
    result.cost += config_.speed_weight * speed_error * speed_error +
                   config_.acceleration_weight * next_acceleration *
                       next_acceleration +
                   config_.jerk_weight * jerk * jerk +
                   config_.command_weight * command * command +
                   config_.command_rate_weight * command_change *
                       command_change;
    if (speed > config_.maximum_speed_mps) {
      const double violation = speed - config_.maximum_speed_mps;
      result.cost += 1.0e8 * violation * violation + 1.0e6;
    }
    acceleration = next_acceleration;
    previous_command = command;
  }
  const double terminal_error = target_speed_mps - speed;
  result.cost += config_.terminal_speed_weight * terminal_error * terminal_error;
  return result;
}

bool LongitudinalMpc::solve(double target_speed_mps, double measured_speed_mps,
                            std::vector<double>* commands, double* cost,
                            double* predicted_maximum_speed_mps,
                            int* iterations) const {
  project(commands);
  RolloutResult best = rollout(*commands, target_speed_mps, measured_speed_mps);
  if (!std::isfinite(best.cost)) return false;
  *iterations = 0;
  if (config_.solver == "osqp") {
    const int horizon = static_cast<int>(commands->size());
    const int rows = 2 * horizon;
    std::vector<double> matrix(static_cast<std::size_t>(rows * horizon), 0.0);
    std::vector<double> lower(static_cast<std::size_t>(rows));
    std::vector<double> upper(static_cast<std::size_t>(rows));
    const double maximum_step = config_.command_rate_limit_per_sec *
                                config_.prediction_dt_sec;
    for (int index = 0; index < horizon; ++index) {
      matrix[static_cast<std::size_t>(index * horizon + index)] = 1.0;
      lower[static_cast<std::size_t>(index)] = -config_.maximum_brake_command;
      upper[static_cast<std::size_t>(index)] = config_.maximum_accel_command;
      const int rate_row = horizon + index;
      matrix[static_cast<std::size_t>(rate_row * horizon + index)] = 1.0;
      if (index == 0) {
        lower[static_cast<std::size_t>(rate_row)] =
            previous_signed_command_ - maximum_step;
        upper[static_cast<std::size_t>(rate_row)] =
            previous_signed_command_ + maximum_step;
      } else {
        matrix[static_cast<std::size_t>(rate_row * horizon + index - 1)] = -1.0;
        lower[static_cast<std::size_t>(rate_row)] = -maximum_step;
        upper[static_cast<std::size_t>(rate_row)] = maximum_step;
      }
    }
    OsqpSqpConfig solver_config;
    solver_config.sqp_iterations = std::min(3, config_.optimizer_iterations);
    solver_config.finite_difference_epsilon = config_.gradient_epsilon;
    const OsqpSqpResult solved = solveWithOsqpSqp(
        [&](const std::vector<double>& candidate) {
          return rollout(candidate, target_speed_mps, measured_speed_mps).cost;
        },
        *commands, matrix, rows, lower, upper, solver_config);
    if (!solved.success) return false;
    *commands = solved.solution;
    best = rollout(*commands, target_speed_mps, measured_speed_mps);
    *iterations = solved.iterations;
  } else if (config_.solver == "coordinate_search") {
    double step = config_.coordinate_initial_step;
    for (int iteration = 0; iteration < config_.optimizer_iterations;
         ++iteration) {
      for (std::size_t index = 0; index < commands->size(); ++index) {
        std::vector<double> best_commands = *commands;
        for (double sign : {-1.0, 1.0}) {
          std::vector<double> candidate = *commands;
          candidate[index] += sign * step;
          project(&candidate);
          const RolloutResult candidate_result =
              rollout(candidate, target_speed_mps, measured_speed_mps);
          if (candidate_result.cost < best.cost) {
            best = candidate_result;
            best_commands = candidate;
          }
        }
        *commands = best_commands;
      }
      *iterations = iteration + 1;
      if (step <= config_.coordinate_minimum_step) break;
      step = std::max(config_.coordinate_minimum_step,
                      step * config_.command_step_decay);
    }
  } else {
    std::vector<double> gradient(commands->size(), 0.0);
    for (int iteration = 0; iteration < config_.optimizer_iterations;
         ++iteration) {
      double norm_squared = 0.0;
      for (std::size_t index = 0; index < commands->size(); ++index) {
        std::vector<double> plus = *commands;
        std::vector<double> minus = *commands;
        plus[index] += config_.gradient_epsilon;
        minus[index] -= config_.gradient_epsilon;
        project(&plus);
        project(&minus);
        const double denominator = plus[index] - minus[index];
        gradient[index] = 0.0;
        if (std::abs(denominator) > 1.0e-12) {
          const double plus_cost =
              rollout(plus, target_speed_mps, measured_speed_mps).cost;
          const double minus_cost =
              rollout(minus, target_speed_mps, measured_speed_mps).cost;
          if (std::isfinite(plus_cost) && std::isfinite(minus_cost)) {
            gradient[index] = (plus_cost - minus_cost) / denominator;
          }
        }
        norm_squared += gradient[index] * gradient[index];
      }
      *iterations = iteration + 1;
      if (!std::isfinite(norm_squared)) return false;
      if (std::sqrt(norm_squared) <= config_.gradient_tolerance) break;
      bool improved = false;
      double step = config_.gradient_step;
      for (int line_search = 0; line_search < config_.line_search_steps;
           ++line_search) {
        std::vector<double> candidate = *commands;
        for (std::size_t index = 0; index < candidate.size(); ++index) {
          candidate[index] -= step * gradient[index];
        }
        project(&candidate);
        const RolloutResult candidate_result =
            rollout(candidate, target_speed_mps, measured_speed_mps);
        if (candidate_result.cost < best.cost) {
          best = candidate_result;
          *commands = candidate;
          improved = true;
          break;
        }
        step *= 0.5;
      }
      if (!improved) break;
    }
  }
  *cost = best.cost;
  *predicted_maximum_speed_mps = best.maximum_speed_mps;
  return std::isfinite(*cost) && !commands->empty();
}

LongitudinalMpcResult LongitudinalMpc::update(double target_speed_mps,
                                              double measured_speed_mps,
                                              double dt_sec) {
  LongitudinalMpcResult result;
  if (!std::isfinite(target_speed_mps) || target_speed_mps < 0.0) {
    result.error = "INVALID_LONGITUDINAL_MPC_TARGET_SPEED";
    return result;
  }
  if (!std::isfinite(measured_speed_mps)) {
    result.error = "INVALID_LONGITUDINAL_MPC_MEASURED_SPEED";
    return result;
  }
  if (!finitePositive(dt_sec)) {
    result.error = "INVALID_LONGITUDINAL_MPC_CONTROL_DT";
    return result;
  }
  // Competition longitudinal velocity is signed. MPC controls speed magnitude;
  // reverse/gear authority remains with MORAI and the gear interlock.
  measured_speed_mps = std::abs(measured_speed_mps);
  target_speed_mps = std::min(target_speed_mps, config_.maximum_speed_mps);
  if (has_previous_speed_) {
    const double raw_acceleration =
        (measured_speed_mps - previous_speed_mps_) / dt_sec;
    const double alpha = 1.0 - std::exp(
        -dt_sec / config_.acceleration_estimate_filter_time_constant_sec);
    estimated_acceleration_mps2_ +=
        alpha * (clamp(raw_acceleration, -config_.maximum_deceleration_mps2,
                       config_.maximum_acceleration_mps2) -
                 estimated_acceleration_mps2_);
  }
  previous_speed_mps_ = measured_speed_mps;
  has_previous_speed_ = true;

  std::vector<double> commands = warm_start_;
  const auto start = std::chrono::steady_clock::now();
  result.valid = solve(target_speed_mps, measured_speed_mps, &commands,
                       &result.cost, &result.predicted_maximum_speed_mps,
                       &result.iterations);
  result.solve_time_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - start)
                             .count();
  result.estimated_acceleration_mps2 = estimated_acceleration_mps2_;
  if (!result.valid) {
    result.error = "LONGITUDINAL_MPC_SOLVER_FAILED";
    return result;
  }

  double command = commands.front();
  const double measured_guard_speed = measured_speed_mps;
  const double predicted_guard_speed =
      measured_speed_mps + std::max(0.0, estimated_acceleration_mps2_) * 0.35;
  if (measured_guard_speed >= config_.hard_brake_activation_speed_mps ||
      predicted_guard_speed >= config_.maximum_speed_mps ||
      result.predicted_maximum_speed_mps > config_.maximum_speed_mps) {
    command = std::min(command, -config_.minimum_hard_brake_command);
    result.hard_speed_guard_active = true;
  }
  const double actual_maximum_step = config_.command_rate_limit_per_sec * dt_sec;
  if (!result.hard_speed_guard_active) {
    command = clamp(command, previous_signed_command_ - actual_maximum_step,
                    previous_signed_command_ + actual_maximum_step);
  }
  command = clamp(command, -config_.maximum_brake_command,
                  config_.maximum_accel_command);
  previous_signed_command_ = command;
  result.signed_command = command;
  result.accel = std::max(0.0, command);
  result.brake = std::max(0.0, -command);
  result.state = result.hard_speed_guard_active
                     ? LongitudinalState::kHardSpeedBrake
                     : (command > 1.0e-6
                            ? LongitudinalState::kAccel
                            : (command < -1.0e-6 ? LongitudinalState::kBrake
                                                : LongitudinalState::kCoast));
  warm_start_.assign(commands.size(), commands.back());
  for (std::size_t index = 0; index + 1 < commands.size(); ++index) {
    warm_start_[index] = commands[index + 1];
  }
  result.error.clear();
  return result;
}

void LongitudinalMpc::reset() {
  std::fill(warm_start_.begin(), warm_start_.end(), 0.0);
  has_previous_speed_ = false;
  previous_speed_mps_ = 0.0;
  estimated_acceleration_mps2_ = 0.0;
  previous_signed_command_ = 0.0;
}

} }  // namespace control::tracking
