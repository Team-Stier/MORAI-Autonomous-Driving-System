#pragma once
#include <control/cores/morai_mpc.hpp>
#include <node_pipeline/extension.hpp>

namespace control {
inline morai_mpc::Parameters loadMoraiMpcParameters(const node_pipeline::ParamReader& params) {
  morai_mpc::Parameters p;
  p.minimum_control_dt_sec = params.number("minimum_control_dt_sec");
  p.maximum_control_dt_sec = params.number("maximum_control_dt_sec");
  p.lateral.ctrl_period = params.number("lateral/ctrl_period");
  p.lateral.enable_path_smoothing = params.flag("lateral/enable_path_smoothing");
  p.lateral.enable_yaw_recalculation = params.flag("lateral/enable_yaw_recalculation");
  p.lateral.path_filter_moving_ave_num = static_cast<int>(params.integer("lateral/path_filter_moving_ave_num"));
  p.lateral.path_smoothing_times = static_cast<int>(params.integer("lateral/path_smoothing_times"));
  p.lateral.curvature_smoothing_num = static_cast<int>(params.integer("lateral/curvature_smoothing_num"));
  p.lateral.morai_yaw_smoothing_num = static_cast<int>(params.integer("lateral/morai_yaw_smoothing_num"));
  p.lateral.traj_resample_dist = params.number("lateral/traj_resample_dist");
  p.lateral.vehicle_model_wheelbase = params.number("lateral/vehicle_model_wheelbase");
  p.lateral.steer_lim_deg = params.number("lateral/steer_lim_deg");
  p.lateral.morai_steer_rate_limit_deg = params.number("lateral/morai_steer_rate_limit_deg");
  p.lateral.mpc_prediction_horizon = static_cast<int>(params.integer("lateral/mpc_prediction_horizon"));
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
  p.lateral.qpoases_max_iter = static_cast<int>(params.integer("lateral/qpoases_max_iter"));
  p.lateral.morai_min_prediction_length_m = params.number("lateral/morai_min_prediction_length_m");
  p.lateral.morai_max_prediction_horizon = static_cast<int>(params.integer("lateral/morai_max_prediction_horizon"));
  p.lateral.morai_use_output_steering_filter = params.flag("lateral/morai_use_output_steering_filter");
  p.lateral.morai_straight_lat_weight_scale = params.number("lateral/morai_straight_lat_weight_scale");
  p.lateral.morai_curve_steer_weight_scale = params.number("lateral/morai_curve_steer_weight_scale");
  p.lateral.morai_weight_curvature_low = params.number("lateral/morai_weight_curvature_low");
  p.lateral.morai_weight_curvature_high = params.number("lateral/morai_weight_curvature_high");
  p.lateral.morai_weight_first_steer_change = params.number("lateral/morai_weight_first_steer_change");
  p.lateral.morai_weight_front_point_error = params.number("lateral/morai_weight_front_point_error");
  p.lateral.morai_front_overhang_m = params.number("lateral/morai_front_overhang_m");
  p.longitudinal.solver = params.text("longitudinal/solver");
  p.longitudinal.horizon_steps = static_cast<int>(params.integer("longitudinal/horizon_steps"));
  p.longitudinal.optimizer_iterations = static_cast<int>(params.integer("longitudinal/optimizer_iterations"));
  p.longitudinal.line_search_steps = static_cast<int>(params.integer("longitudinal/line_search_steps"));
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
  p.longitudinal.actuator_time_constant_sec = params.number("longitudinal/actuator_time_constant_sec");
  p.longitudinal.physical_rollout = params.flag("longitudinal/physical_rollout");
  p.longitudinal.constant_drag_mps2 = params.number("longitudinal/constant_drag_mps2");
  p.longitudinal.quadratic_drag_per_m = params.number("longitudinal/quadratic_drag_per_m");
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
  morai_mpc::validateParameters(p);
  return p;
}
}  // namespace control
