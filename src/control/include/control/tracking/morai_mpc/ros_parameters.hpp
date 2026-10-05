#pragma once
#include <control/tracking/morai_mpc/parameters.hpp>
#include <ros/node_handle.h>
#include <stdexcept>
#include <string>

namespace control { namespace morai_mpc {
namespace config_detail {
template<class T> T required(const ros::NodeHandle& node, const char* key) {
  T value{};
  if (!node.getParam(key, value))
    throw std::invalid_argument(std::string("missing or invalid morai_mpc parameter: ") + key);
  return value;
}
}  // namespace config_detail

// Pass a NodeHandle for the morai_mpc namespace, outside the Core.
inline Parameters loadParameters(const ros::NodeHandle& node) {
  Parameters p;
  p.minimum_control_dt_sec = config_detail::required<double>(node, "minimum_control_dt_sec");
  p.maximum_control_dt_sec = config_detail::required<double>(node, "maximum_control_dt_sec");
  p.lateral.ctrl_period = config_detail::required<double>(node, "lateral/ctrl_period");
  p.lateral.enable_path_smoothing = config_detail::required<bool>(node, "lateral/enable_path_smoothing");
  p.lateral.enable_yaw_recalculation = config_detail::required<bool>(node, "lateral/enable_yaw_recalculation");
  p.lateral.path_filter_moving_ave_num = config_detail::required<int>(node, "lateral/path_filter_moving_ave_num");
  p.lateral.path_smoothing_times = config_detail::required<int>(node, "lateral/path_smoothing_times");
  p.lateral.curvature_smoothing_num = config_detail::required<int>(node, "lateral/curvature_smoothing_num");
  p.lateral.morai_yaw_smoothing_num = config_detail::required<int>(node, "lateral/morai_yaw_smoothing_num");
  p.lateral.traj_resample_dist = config_detail::required<double>(node, "lateral/traj_resample_dist");
  p.lateral.vehicle_model_wheelbase = config_detail::required<double>(node, "lateral/vehicle_model_wheelbase");
  p.lateral.steer_lim_deg = config_detail::required<double>(node, "lateral/steer_lim_deg");
  p.lateral.morai_steer_rate_limit_deg = config_detail::required<double>(node, "lateral/morai_steer_rate_limit_deg");
  p.lateral.mpc_prediction_horizon = config_detail::required<int>(node, "lateral/mpc_prediction_horizon");
  p.lateral.mpc_prediction_sampling_time = config_detail::required<double>(node, "lateral/mpc_prediction_sampling_time");
  p.lateral.mpc_weight_lat_error = config_detail::required<double>(node, "lateral/mpc_weight_lat_error");
  p.lateral.mpc_weight_heading_error = config_detail::required<double>(node, "lateral/mpc_weight_heading_error");
  p.lateral.mpc_weight_heading_error_squared_vel_coeff = config_detail::required<double>(node, "lateral/mpc_weight_heading_error_squared_vel_coeff");
  p.lateral.mpc_weight_steering_input = config_detail::required<double>(node, "lateral/mpc_weight_steering_input");
  p.lateral.mpc_weight_steering_input_squared_vel_coeff = config_detail::required<double>(node, "lateral/mpc_weight_steering_input_squared_vel_coeff");
  p.lateral.mpc_weight_lat_jerk = config_detail::required<double>(node, "lateral/mpc_weight_lat_jerk");
  p.lateral.mpc_weight_terminal_lat_error = config_detail::required<double>(node, "lateral/mpc_weight_terminal_lat_error");
  p.lateral.mpc_weight_terminal_heading_error = config_detail::required<double>(node, "lateral/mpc_weight_terminal_heading_error");
  p.lateral.mpc_zero_ff_steer_deg = config_detail::required<double>(node, "lateral/mpc_zero_ff_steer_deg");
  p.lateral.steering_lpf_cutoff_hz = config_detail::required<double>(node, "lateral/steering_lpf_cutoff_hz");
  p.lateral.qpoases_max_iter = config_detail::required<int>(node, "lateral/qpoases_max_iter");
  p.lateral.morai_min_prediction_length_m = config_detail::required<double>(node, "lateral/morai_min_prediction_length_m");
  p.lateral.morai_max_prediction_horizon = config_detail::required<int>(node, "lateral/morai_max_prediction_horizon");
  p.lateral.morai_use_output_steering_filter = config_detail::required<bool>(node, "lateral/morai_use_output_steering_filter");
  p.lateral.morai_straight_lat_weight_scale = config_detail::required<double>(node, "lateral/morai_straight_lat_weight_scale");
  p.lateral.morai_curve_steer_weight_scale = config_detail::required<double>(node, "lateral/morai_curve_steer_weight_scale");
  p.lateral.morai_weight_curvature_low = config_detail::required<double>(node, "lateral/morai_weight_curvature_low");
  p.lateral.morai_weight_curvature_high = config_detail::required<double>(node, "lateral/morai_weight_curvature_high");
  p.lateral.morai_weight_first_steer_change = config_detail::required<double>(node, "lateral/morai_weight_first_steer_change");
  p.lateral.morai_weight_front_point_error = config_detail::required<double>(node, "lateral/morai_weight_front_point_error");
  p.lateral.morai_front_overhang_m = config_detail::required<double>(node, "lateral/morai_front_overhang_m");
  p.longitudinal.solver = config_detail::required<std::string>(node, "longitudinal/solver");
  p.longitudinal.horizon_steps = config_detail::required<int>(node, "longitudinal/horizon_steps");
  p.longitudinal.optimizer_iterations = config_detail::required<int>(node, "longitudinal/optimizer_iterations");
  p.longitudinal.line_search_steps = config_detail::required<int>(node, "longitudinal/line_search_steps");
  p.longitudinal.prediction_dt_sec = config_detail::required<double>(node, "longitudinal/prediction_dt_sec");
  p.longitudinal.gradient_epsilon = config_detail::required<double>(node, "longitudinal/gradient_epsilon");
  p.longitudinal.gradient_step = config_detail::required<double>(node, "longitudinal/gradient_step");
  p.longitudinal.gradient_tolerance = config_detail::required<double>(node, "longitudinal/gradient_tolerance");
  p.longitudinal.coordinate_initial_step = config_detail::required<double>(node, "longitudinal/coordinate_initial_step");
  p.longitudinal.coordinate_minimum_step = config_detail::required<double>(node, "longitudinal/coordinate_minimum_step");
  p.longitudinal.command_step_decay = config_detail::required<double>(node, "longitudinal/command_step_decay");
  p.longitudinal.maximum_accel_command = config_detail::required<double>(node, "longitudinal/maximum_accel_command");
  p.longitudinal.maximum_brake_command = config_detail::required<double>(node, "longitudinal/maximum_brake_command");
  p.longitudinal.command_rate_limit_per_sec = config_detail::required<double>(node, "longitudinal/command_rate_limit_per_sec");
  p.longitudinal.acceleration_gain_mps2 = config_detail::required<double>(node, "longitudinal/acceleration_gain_mps2");
  p.longitudinal.braking_gain_mps2 = config_detail::required<double>(node, "longitudinal/braking_gain_mps2");
  p.longitudinal.actuator_time_constant_sec = config_detail::required<double>(node, "longitudinal/actuator_time_constant_sec");
  p.longitudinal.physical_rollout = config_detail::required<bool>(node, "longitudinal/physical_rollout");
  p.longitudinal.constant_drag_mps2 = config_detail::required<double>(node, "longitudinal/constant_drag_mps2");
  p.longitudinal.quadratic_drag_per_m = config_detail::required<double>(node, "longitudinal/quadratic_drag_per_m");
  p.longitudinal.maximum_acceleration_mps2 = config_detail::required<double>(node, "longitudinal/maximum_acceleration_mps2");
  p.longitudinal.maximum_deceleration_mps2 = config_detail::required<double>(node, "longitudinal/maximum_deceleration_mps2");
  p.longitudinal.maximum_jerk_mps3 = config_detail::required<double>(node, "longitudinal/maximum_jerk_mps3");
  p.longitudinal.acceleration_estimate_filter_time_constant_sec = config_detail::required<double>(node, "longitudinal/acceleration_estimate_filter_time_constant_sec");
  p.longitudinal.speed_weight = config_detail::required<double>(node, "longitudinal/speed_weight");
  p.longitudinal.acceleration_weight = config_detail::required<double>(node, "longitudinal/acceleration_weight");
  p.longitudinal.jerk_weight = config_detail::required<double>(node, "longitudinal/jerk_weight");
  p.longitudinal.command_weight = config_detail::required<double>(node, "longitudinal/command_weight");
  p.longitudinal.command_rate_weight = config_detail::required<double>(node, "longitudinal/command_rate_weight");
  p.longitudinal.terminal_speed_weight = config_detail::required<double>(node, "longitudinal/terminal_speed_weight");
  p.steering_prediction.delay_sec = config_detail::required<double>(node, "steering_prediction/delay_sec");
  p.steering_prediction.time_constant_sec = config_detail::required<double>(node, "steering_prediction/time_constant_sec");
  p.steering_prediction.gain = config_detail::required<double>(node, "steering_prediction/gain");
  validateParameters(p);
  return p;
}
}}  // namespace control::morai_mpc
