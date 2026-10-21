#pragma once
#include <control/cores/morai_mpc.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace control { namespace autoware_ai_mpc {
namespace parameter_detail {
inline bool finitePositive(double value) { return std::isfinite(value) && value > 0.0; }
inline void positive(std::initializer_list<double> values) {
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("morai_mpc parameter must be finite and positive");
  }
}
inline void nonnegative(std::initializer_list<double> values) {
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("morai_mpc parameter must be finite and nonnegative");
  }
}
}  // namespace

inline void validateParameters(const Parameters& parameters) {
  const auto& p = parameters.lateral;
  const auto& s = parameters.steering_prediction;
  parameter_detail::positive({p.ctrl_period, p.traj_resample_dist, p.vehicle_model_wheelbase,
      p.steer_lim_deg, p.morai_steer_rate_limit_deg, p.mpc_prediction_sampling_time,
      p.steering_lpf_cutoff_hz, p.morai_straight_lat_weight_scale,
      p.morai_curve_steer_weight_scale, s.time_constant_sec, s.gain,
      parameters.minimum_control_dt_sec, parameters.maximum_control_dt_sec});
  parameter_detail::nonnegative({p.mpc_weight_lat_error, p.mpc_weight_heading_error,
      p.mpc_weight_heading_error_squared_vel_coeff, p.mpc_weight_steering_input,
      p.mpc_weight_steering_input_squared_vel_coeff, p.mpc_weight_lat_jerk,
      p.mpc_weight_terminal_lat_error, p.mpc_weight_terminal_heading_error,
      p.mpc_zero_ff_steer_deg, p.morai_weight_first_steer_change,
      p.morai_weight_front_point_error, p.morai_front_overhang_m, s.delay_sec});
  if (p.ctrl_period < parameters.minimum_control_dt_sec ||
      p.ctrl_period > parameters.maximum_control_dt_sec ||
      p.steering_lpf_cutoff_hz * p.ctrl_period >= 0.5 ||
      p.steer_lim_deg > 40.0 || p.morai_max_prediction_horizon > 150 ||
      p.mpc_prediction_horizon > 150 || p.morai_min_prediction_length_m > 50.0 ||
      p.qpoases_max_iter <= 0 || p.path_filter_moving_ave_num < 1 ||
      p.path_smoothing_times < 1 || p.curvature_smoothing_num < 1 ||
      p.morai_yaw_smoothing_num < 1 || p.morai_yaw_smoothing_num > 50 ||
      p.morai_straight_lat_weight_scale > 1.0 || p.morai_curve_steer_weight_scale > 1.0 ||
      p.morai_weight_first_steer_change > 100.0 ||
      p.morai_weight_front_point_error > 100.0 || p.morai_front_overhang_m > 3.0 ||
      p.vehicle_model_wheelbase + p.morai_front_overhang_m > 8.0 ||
      s.delay_sec > 0.5 || s.time_constant_sec < 0.01 || s.time_constant_sec > 1.0 ||
      s.gain < 0.5 || s.gain > 1.5 || parameters.longitudinal.horizon_steps < 2 ||
      parameters.longitudinal.solver != "osqp") {
    throw std::invalid_argument("morai_mpc parameter outside supported range");
  }
  if (p.mpc_prediction_horizon < 2 || !std::isfinite(p.mpc_prediction_sampling_time) ||
      p.mpc_prediction_sampling_time <= 0. || !std::isfinite(p.morai_min_prediction_length_m) ||
      p.morai_min_prediction_length_m < 0. ||
      (p.morai_min_prediction_length_m > 0. && p.morai_max_prediction_horizon < p.mpc_prediction_horizon))
    throw std::invalid_argument("Invalid prediction length parameters");
  if (!std::isfinite(p.morai_weight_curvature_low) || !std::isfinite(p.morai_weight_curvature_high) ||
      p.morai_weight_curvature_low < 0. || p.morai_weight_curvature_high <= p.morai_weight_curvature_low)
    throw std::invalid_argument("Invalid curvature weight interval");
  const auto& config = parameters.longitudinal;
  parameter_detail::nonnegative({config.high_speed_acceleration_gain_mps2,
      config.high_speed_constant_drag_mps2,config.high_speed_quadratic_drag_per_m,
      config.drive_model_transition_low_mps,config.drive_model_transition_high_mps});
  if(config.high_speed_acceleration_gain_mps2>0 && (!config.physical_rollout ||
      config.drive_model_transition_high_mps<=config.drive_model_transition_low_mps))
    throw std::invalid_argument("Invalid drive model transition interval");
  const bool solver_valid = config.solver == "osqp" ||
                            config.solver == "projected_gradient" ||
                            config.solver == "coordinate_search";
  if (!solver_valid || config.horizon_steps < 2 ||
      config.optimizer_iterations <= 0 || config.line_search_steps <= 0 ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.prediction_dt_sec) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.gradient_epsilon) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.gradient_step) ||
      !std::isfinite(config.gradient_tolerance) ||
      config.gradient_tolerance < 0.0 ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.coordinate_initial_step) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.coordinate_minimum_step) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.command_step_decay) ||
      config.command_step_decay >= 1.0 ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.maximum_accel_command) ||
      config.maximum_accel_command > 1.0 ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.maximum_brake_command) ||
      config.maximum_brake_command > 1.0 ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.command_rate_limit_per_sec) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.acceleration_gain_mps2) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.braking_gain_mps2) ||
      !std::isfinite(config.high_speed_braking_gain_mps2) || config.high_speed_braking_gain_mps2 < 0.0 ||
      !std::isfinite(config.braking_gain_transition_low_mps) || config.braking_gain_transition_low_mps < 0.0 ||
      !std::isfinite(config.braking_gain_transition_high_mps) || config.braking_gain_transition_high_mps < 0.0 ||
      !std::isfinite(config.strong_brake_transition_low) || config.strong_brake_transition_low < 0.0 ||
      !std::isfinite(config.strong_brake_transition_high) || config.strong_brake_transition_high > 1.0 ||
      (config.high_speed_braking_gain_mps2 > 0.0 &&
       (!config.physical_rollout || config.braking_gain_transition_high_mps <= config.braking_gain_transition_low_mps ||
        config.strong_brake_transition_high <= config.strong_brake_transition_low)) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.actuator_time_constant_sec) ||
      !std::isfinite(config.constant_drag_mps2) || config.constant_drag_mps2 < 0.0 ||
      !std::isfinite(config.quadratic_drag_per_m) || config.quadratic_drag_per_m < 0.0 ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.maximum_acceleration_mps2) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.maximum_deceleration_mps2) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.maximum_jerk_mps3) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.acceleration_estimate_filter_time_constant_sec) ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.speed_weight) ||
      !std::isfinite(config.acceleration_weight) ||
      config.acceleration_weight < 0.0 ||
      !std::isfinite(config.jerk_weight) || config.jerk_weight < 0.0 ||
      !std::isfinite(config.command_weight) || config.command_weight < 0.0 ||
      !std::isfinite(config.command_rate_weight) ||
      config.command_rate_weight < 0.0 ||
      !autoware_ai_mpc::parameter_detail::finitePositive(config.terminal_speed_weight)) {
    throw std::invalid_argument("invalid longitudinal MPC configuration");
  }
}
}}  // namespace control::autoware_ai_mpc

namespace control {
inline autoware_ai_mpc::Parameters loadMoraiMpcParameters(const node_pipeline::ParamReader& params) {
  using autoware_ai_mpc::Parameters;
  Parameters p;
  p.minimum_control_dt_sec = params.number("minimum_control_dt_sec");
  p.maximum_control_dt_sec = params.number("maximum_control_dt_sec");
  p.lateral.ctrl_period = params.number("lateral/ctrl_period");
  p.lateral.enable_path_smoothing = params.flag("lateral/enable_path_smoothing");
  p.lateral.enable_yaw_recalculation = params.flag("lateral/enable_yaw_recalculation");
  p.lateral.path_filter_moving_ave_num = params.integer("lateral/path_filter_moving_ave_num");
  p.lateral.path_smoothing_times = params.integer("lateral/path_smoothing_times");
  p.lateral.curvature_smoothing_num = params.integer("lateral/curvature_smoothing_num");
  p.lateral.morai_yaw_smoothing_num = params.integer("lateral/morai_yaw_smoothing_num");
  p.lateral.traj_resample_dist = params.number("lateral/traj_resample_dist");
  p.lateral.vehicle_model_wheelbase = params.number("lateral/vehicle_model_wheelbase");
  p.lateral.steer_lim_deg = params.number("lateral/steer_lim_deg");
  p.lateral.morai_steer_rate_limit_deg = params.number("lateral/morai_steer_rate_limit_deg");
  p.lateral.mpc_prediction_horizon = params.integer("lateral/mpc_prediction_horizon");
  p.lateral.mpc_prediction_sampling_time = params.number("lateral/mpc_prediction_sampling_time");
  p.lateral.mpc_weight_lat_error = params.number("lateral/mpc_weight_lat_error");
  p.lateral.mpc_weight_heading_error = params.number("lateral/mpc_weight_heading_error");
  p.lateral.mpc_weight_heading_error_squared_vel_coeff = params.number("lateral/mpc_weight_heading_error_squared_vel_coeff");
  p.lateral.mpc_weight_steering_input = params.number("lateral/mpc_weight_steering_input");
  p.lateral.mpc_weight_steering_input_squared_vel_coeff = params.number("lateral/mpc_weight_steering_input_squared_vel_coeff");
  p.lateral.mpc_weight_lat_jerk = params.number("lateral/mpc_weight_lat_jerk");
  p.lateral.mpc_weight_terminal_lat_error = params.number("lateral/mpc_weight_terminal_lat_error");
  p.lateral.mpc_weight_terminal_heading_error = params.number("lateral/mpc_weight_terminal_heading_error");
  p.lateral.mpc_zero_ff_steer_deg = params.number("lateral/mpc_zero_ff_steer_deg");
  p.lateral.steering_lpf_cutoff_hz = params.number("lateral/steering_lpf_cutoff_hz");
  p.lateral.qpoases_max_iter = params.integer("lateral/qpoases_max_iter");
  p.lateral.morai_min_prediction_length_m = params.number("lateral/morai_min_prediction_length_m");
  p.lateral.morai_max_prediction_horizon = params.integer("lateral/morai_max_prediction_horizon");
  p.lateral.morai_use_output_steering_filter = params.flag("lateral/morai_use_output_steering_filter");
  p.lateral.morai_straight_lat_weight_scale = params.number("lateral/morai_straight_lat_weight_scale");
  p.lateral.morai_curve_steer_weight_scale = params.number("lateral/morai_curve_steer_weight_scale");
  p.lateral.morai_weight_curvature_low = params.number("lateral/morai_weight_curvature_low");
  p.lateral.morai_weight_curvature_high = params.number("lateral/morai_weight_curvature_high");
  p.lateral.morai_weight_first_steer_change = params.number("lateral/morai_weight_first_steer_change");
  p.lateral.morai_weight_front_point_error = params.number("lateral/morai_weight_front_point_error");
  p.lateral.morai_front_overhang_m = params.number("lateral/morai_front_overhang_m");
  p.longitudinal.solver = params.text("longitudinal/solver");
  p.longitudinal.horizon_steps = params.integer("longitudinal/horizon_steps");
  p.longitudinal.optimizer_iterations = params.integer("longitudinal/optimizer_iterations");
  p.longitudinal.line_search_steps = params.integer("longitudinal/line_search_steps");
  p.longitudinal.prediction_dt_sec = params.number("longitudinal/prediction_dt_sec");
  p.longitudinal.gradient_epsilon = params.number("longitudinal/gradient_epsilon");
  p.longitudinal.gradient_step = params.number("longitudinal/gradient_step");
  p.longitudinal.gradient_tolerance = params.number("longitudinal/gradient_tolerance");
  p.longitudinal.coordinate_initial_step = params.number("longitudinal/coordinate_initial_step");
  p.longitudinal.coordinate_minimum_step = params.number("longitudinal/coordinate_minimum_step");
  p.longitudinal.command_step_decay = params.number("longitudinal/command_step_decay");
  p.longitudinal.maximum_accel_command = params.number("longitudinal/maximum_accel_command");
  p.longitudinal.maximum_brake_command = params.number("longitudinal/maximum_brake_command");
  p.longitudinal.command_rate_limit_per_sec = params.number("longitudinal/command_rate_limit_per_sec");
  p.longitudinal.acceleration_gain_mps2 = params.number("longitudinal/acceleration_gain_mps2");
  p.longitudinal.braking_gain_mps2 = params.number("longitudinal/braking_gain_mps2");
  if (params.has("longitudinal/high_speed_braking_gain_mps2")) {
    p.longitudinal.high_speed_braking_gain_mps2 = params.number("longitudinal/high_speed_braking_gain_mps2");
    p.longitudinal.braking_gain_transition_low_mps = params.number("longitudinal/braking_gain_transition_low_mps");
    p.longitudinal.braking_gain_transition_high_mps = params.number("longitudinal/braking_gain_transition_high_mps");
    p.longitudinal.strong_brake_transition_low = params.number("longitudinal/strong_brake_transition_low");
    p.longitudinal.strong_brake_transition_high = params.number("longitudinal/strong_brake_transition_high");
  }
  p.longitudinal.actuator_time_constant_sec = params.number("longitudinal/actuator_time_constant_sec");
  p.longitudinal.physical_rollout = params.flag("longitudinal/physical_rollout");
  p.longitudinal.constant_drag_mps2 = params.number("longitudinal/constant_drag_mps2");
  p.longitudinal.quadratic_drag_per_m = params.number("longitudinal/quadratic_drag_per_m");
  if(params.has("longitudinal/high_speed_acceleration_gain_mps2")) {
    p.longitudinal.high_speed_acceleration_gain_mps2=params.number("longitudinal/high_speed_acceleration_gain_mps2");
    p.longitudinal.high_speed_constant_drag_mps2=params.number("longitudinal/high_speed_constant_drag_mps2");
    p.longitudinal.high_speed_quadratic_drag_per_m=params.number("longitudinal/high_speed_quadratic_drag_per_m");
    p.longitudinal.drive_model_transition_low_mps=params.number("longitudinal/drive_model_transition_low_mps");
    p.longitudinal.drive_model_transition_high_mps=params.number("longitudinal/drive_model_transition_high_mps");
  }
  p.longitudinal.maximum_acceleration_mps2 = params.number("longitudinal/maximum_acceleration_mps2");
  p.longitudinal.maximum_deceleration_mps2 = params.number("longitudinal/maximum_deceleration_mps2");
  p.longitudinal.maximum_jerk_mps3 = params.number("longitudinal/maximum_jerk_mps3");
  p.longitudinal.acceleration_estimate_filter_time_constant_sec = params.number("longitudinal/acceleration_estimate_filter_time_constant_sec");
  p.longitudinal.speed_weight = params.number("longitudinal/speed_weight");
  p.longitudinal.acceleration_weight = params.number("longitudinal/acceleration_weight");
  p.longitudinal.jerk_weight = params.number("longitudinal/jerk_weight");
  p.longitudinal.command_weight = params.number("longitudinal/command_weight");
  p.longitudinal.command_rate_weight = params.number("longitudinal/command_rate_weight");
  p.longitudinal.terminal_speed_weight = params.number("longitudinal/terminal_speed_weight");
  p.steering_prediction.delay_sec = params.number("steering_prediction/delay_sec");
  p.steering_prediction.time_constant_sec = params.number("steering_prediction/time_constant_sec");
  p.steering_prediction.gain = params.number("steering_prediction/gain");
  autoware_ai_mpc::validateParameters(p);
  return p;
}
}
