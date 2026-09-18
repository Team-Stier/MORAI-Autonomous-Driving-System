#pragma once

namespace morai_path_tracking {

struct PidConfig {
  double kp{0.35};
  double ki{0.08};
  double kd{0.02};
  double integral_limit{2.0};
  double integral_unwind_rate_per_sec{0.5};
  double error_deadband_mps{0.05};
  double accel_feedforward_gain_per_mps{0.0};
  double coast_overspeed_threshold_mps{0.05};
  double brake_overspeed_threshold_mps{0.50};
  double hard_brake_activation_speed_mps{59.0 / 3.6};
  double minimum_hard_brake_command{0.25};
  double maximum_accel{0.40};
  double maximum_brake{0.60};
  double command_rate_limit_per_sec{0.0};
};

enum class LongitudinalState {
  kAccel,
  kCoast,
  kBrake,
  kHardSpeedBrake,
};

struct LongitudinalCommand {
  double accel{0.0};
  double brake{0.0};
  LongitudinalState state{LongitudinalState::kCoast};
};

const char* longitudinalStateName(LongitudinalState state) noexcept;

class LongitudinalPid {
 public:
  explicit LongitudinalPid(const PidConfig& config);
  LongitudinalCommand update(double target_speed_mps,
                             double measured_speed_mps, double dt_sec);
  void reset();

 private:
  PidConfig config_;
  double integral_{0.0};
  bool has_previous_measurement_{false};
  double previous_measurement_mps_{0.0};
  double previous_signed_command_{0.0};
};

}  // namespace morai_path_tracking
