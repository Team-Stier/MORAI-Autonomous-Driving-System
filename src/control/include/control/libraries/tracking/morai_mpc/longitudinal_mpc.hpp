#pragma once

#include <limits>
#include <string>
#include <vector>

#include "control/libraries/tracking/morai_mpc/types.hpp"

namespace control { namespace morai_mpc {

struct LongitudinalMpcConfig {
  std::string solver{"osqp"};
  int horizon_steps{30};
  int optimizer_iterations{10};
  int line_search_steps{8};
  double prediction_dt_sec{0.10};
  double gradient_epsilon{0.01};
  double gradient_step{0.08};
  double gradient_tolerance{1.0e-5};
  double coordinate_initial_step{0.20};
  double coordinate_minimum_step{0.01};
  double command_step_decay{0.5};
  double maximum_accel_command{0.70};
  double maximum_brake_command{0.60};
  double command_rate_limit_per_sec{2.0};
  double acceleration_gain_mps2{3.0};
  double braking_gain_mps2{6.0};
  double actuator_time_constant_sec{0.25};
  bool physical_rollout{false};
  double constant_drag_mps2{0.0};
  double quadratic_drag_per_m{0.0};
  double maximum_acceleration_mps2{2.5};
  double maximum_deceleration_mps2{5.0};
  double maximum_jerk_mps3{3.0};
  double acceleration_estimate_filter_time_constant_sec{0.20};
  double speed_weight{8.0};
  double acceleration_weight{0.20};
  double jerk_weight{1.2};
  double command_weight{0.08};
  double command_rate_weight{2.5};
  double terminal_speed_weight{24.0};
};

struct LongitudinalMpcResult {
  bool valid{false};
  double accel{0.0};
  double brake{0.0};
  double signed_command{0.0};
  double estimated_acceleration_mps2{0.0};
  double predicted_maximum_speed_mps{0.0};
  double cost{std::numeric_limits<double>::infinity()};
  double solve_time_ms{0.0};
  int iterations{0};
  LongitudinalState state{LongitudinalState::kCoast};
  std::string error;
};

class LongitudinalMpc {
 public:
  explicit LongitudinalMpc(const LongitudinalMpcConfig& config);
  LongitudinalMpcResult update(double target_speed_mps,
                               double measured_speed_mps, double dt_sec);
  void reset();

 private:
  struct RolloutResult {
    double cost{std::numeric_limits<double>::infinity()};
    double maximum_speed_mps{0.0};
  };

  void project(std::vector<double>* commands) const;
  RolloutResult rollout(const std::vector<double>& commands,
                        double target_speed_mps,
                        double measured_speed_mps) const;
  bool solve(double target_speed_mps, double measured_speed_mps,
             std::vector<double>* commands, double* cost,
             double* predicted_maximum_speed_mps, int* iterations) const;

  LongitudinalMpcConfig config_;
  std::vector<double> warm_start_;
  bool has_previous_speed_{false};
  double previous_speed_mps_{0.0};
  double estimated_acceleration_mps2_{0.0};
  double previous_signed_command_{0.0};
};

}}  // namespace control::morai_mpc
