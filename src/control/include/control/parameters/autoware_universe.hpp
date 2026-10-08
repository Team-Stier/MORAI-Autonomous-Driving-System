#pragma once
#include <control/cores/autoware_universe.hpp>
#include <node_pipeline/extension.hpp>
#include <stdexcept>
namespace control {
inline universe::Parameters loadAutowareUniverseParameters(const node_pipeline::ParamReader& params) {
  auto p = universe::defaultParameters();
  if (params.has("lateral/traj_resample_dist")) p.trajectory_filter.traj_resample_dist = params.number("lateral/traj_resample_dist");
  if (params.has("lateral/use_steer_prediction")) p.use_steer_prediction = params.flag("lateral/use_steer_prediction");
  if (params.has("lateral/enable_path_smoothing")) p.trajectory_filter.enable_path_smoothing = params.flag("lateral/enable_path_smoothing");
  if (params.has("lateral/path_filter_moving_ave_num")) p.trajectory_filter.path_filter_moving_ave_num = params.integer("lateral/path_filter_moving_ave_num");
  if (params.has("lateral/curvature_smoothing_num_traj")) p.trajectory_filter.curvature_smoothing_num_traj = params.integer("lateral/curvature_smoothing_num_traj");
  if (params.has("lateral/curvature_smoothing_num_ref_steer")) p.trajectory_filter.curvature_smoothing_num_ref_steer = params.integer("lateral/curvature_smoothing_num_ref_steer");
  if (params.has("lateral/extend_trajectory_for_end_yaw_control")) p.trajectory_filter.extend_trajectory_for_end_yaw_control = params.flag("lateral/extend_trajectory_for_end_yaw_control");
  if (params.has("lateral/mpc_prediction_horizon")) p.lateral.prediction_horizon = params.integer("lateral/mpc_prediction_horizon");
  if (params.has("lateral/mpc_prediction_dt")) p.lateral.prediction_dt = params.number("lateral/mpc_prediction_dt");
  if (params.has("lateral/mpc_weight_lat_error")) p.lateral.nominal_weight.lat_error = params.number("lateral/mpc_weight_lat_error");
  if (params.has("lateral/mpc_weight_heading_error")) p.lateral.nominal_weight.heading_error = params.number("lateral/mpc_weight_heading_error");
  if (params.has("lateral/mpc_weight_heading_error_squared_vel")) p.lateral.nominal_weight.heading_error_squared_vel = params.number("lateral/mpc_weight_heading_error_squared_vel");
  if (params.has("lateral/mpc_weight_steering_input")) p.lateral.nominal_weight.steering_input = params.number("lateral/mpc_weight_steering_input");
  if (params.has("lateral/mpc_weight_steering_input_squared_vel")) p.lateral.nominal_weight.steering_input_squared_vel = params.number("lateral/mpc_weight_steering_input_squared_vel");
  if (params.has("lateral/mpc_weight_lat_jerk")) p.lateral.nominal_weight.lat_jerk = params.number("lateral/mpc_weight_lat_jerk");
  if (params.has("lateral/mpc_weight_steer_rate")) p.lateral.nominal_weight.steer_rate = params.number("lateral/mpc_weight_steer_rate");
  if (params.has("lateral/mpc_weight_steer_acc")) p.lateral.nominal_weight.steer_acc = params.number("lateral/mpc_weight_steer_acc");
  if (params.has("lateral/mpc_low_curvature_weight_lat_error")) p.lateral.low_curvature_weight.lat_error = params.number("lateral/mpc_low_curvature_weight_lat_error");
  if (params.has("lateral/mpc_low_curvature_weight_heading_error")) p.lateral.low_curvature_weight.heading_error = params.number("lateral/mpc_low_curvature_weight_heading_error");
  if (params.has("lateral/mpc_low_curvature_weight_heading_error_squared_vel")) p.lateral.low_curvature_weight.heading_error_squared_vel = params.number("lateral/mpc_low_curvature_weight_heading_error_squared_vel");
  if (params.has("lateral/mpc_low_curvature_weight_steering_input")) p.lateral.low_curvature_weight.steering_input = params.number("lateral/mpc_low_curvature_weight_steering_input");
  if (params.has("lateral/mpc_low_curvature_weight_steering_input_squared_vel")) p.lateral.low_curvature_weight.steering_input_squared_vel = params.number("lateral/mpc_low_curvature_weight_steering_input_squared_vel");
  if (params.has("lateral/mpc_low_curvature_weight_lat_jerk")) p.lateral.low_curvature_weight.lat_jerk = params.number("lateral/mpc_low_curvature_weight_lat_jerk");
  if (params.has("lateral/mpc_low_curvature_weight_steer_rate")) p.lateral.low_curvature_weight.steer_rate = params.number("lateral/mpc_low_curvature_weight_steer_rate");
  if (params.has("lateral/mpc_low_curvature_weight_steer_acc")) p.lateral.low_curvature_weight.steer_acc = params.number("lateral/mpc_low_curvature_weight_steer_acc");
  if (params.has("lateral/mpc_low_curvature_thresh_curvature")) p.lateral.low_curvature_thresh_curvature = params.number("lateral/mpc_low_curvature_thresh_curvature");
  if (params.has("lateral/mpc_weight_terminal_lat_error")) p.lateral.nominal_weight.terminal_lat_error = params.number("lateral/mpc_weight_terminal_lat_error");
  if (params.has("lateral/mpc_weight_terminal_heading_error")) p.lateral.nominal_weight.terminal_heading_error = params.number("lateral/mpc_weight_terminal_heading_error");
  if (params.has("lateral/mpc_zero_ff_steer_deg")) p.lateral.zero_ff_steer_deg = params.number("lateral/mpc_zero_ff_steer_deg");
  if (params.has("lateral/mpc_acceleration_limit")) p.lateral.acceleration_limit = params.number("lateral/mpc_acceleration_limit");
  if (params.has("lateral/mpc_velocity_time_constant")) p.lateral.velocity_time_constant = params.number("lateral/mpc_velocity_time_constant");
  if (params.has("lateral/mpc_min_prediction_length")) p.lateral.min_prediction_length = params.number("lateral/mpc_min_prediction_length");
  if (params.has("lateral/input_delay")) p.lateral.input_delay = params.number("lateral/input_delay");
  if (params.has("lateral/vehicle_model_steer_tau")) p.lateral.steer_tau = params.number("lateral/vehicle_model_steer_tau");
  if (params.has("lateral/steering_lpf_cutoff_hz")) p.steering_lpf_cutoff_hz = params.number("lateral/steering_lpf_cutoff_hz");
  if (params.has("lateral/error_deriv_lpf_cutoff_hz")) p.error_deriv_lpf_cutoff_hz = params.number("lateral/error_deriv_lpf_cutoff_hz");
  if (params.has("lateral/stop_state_entry_ego_speed")) p.stop_state_entry_ego_speed = params.number("lateral/stop_state_entry_ego_speed");
  if (params.has("lateral/stop_state_entry_target_speed")) p.stop_state_entry_target_speed = params.number("lateral/stop_state_entry_target_speed");
  if (params.has("lateral/converged_steer_rad")) p.converged_steer_rad = params.number("lateral/converged_steer_rad");
  if (params.has("lateral/keep_steer_control_until_converged")) p.keep_steer_control_until_converged = params.flag("lateral/keep_steer_control_until_converged");
  if (params.has("lateral/new_traj_duration_time")) p.new_traj_duration_time = params.number("lateral/new_traj_duration_time");
  if (params.has("lateral/new_traj_end_dist")) p.new_traj_end_dist = params.number("lateral/new_traj_end_dist");
  if (params.has("lateral/mpc_converged_threshold_rps")) p.mpc_converged_threshold_rps = params.number("lateral/mpc_converged_threshold_rps");
  if (params.has("longitudinal/delay_compensation_time")) p.longitudinal.delay_compensation_time = params.number("longitudinal/delay_compensation_time");
  if (params.has("longitudinal/enable_smooth_stop")) p.longitudinal.enable_smooth_stop = params.flag("longitudinal/enable_smooth_stop");
  if (params.has("longitudinal/enable_overshoot_emergency")) p.longitudinal.enable_overshoot_emergency = params.flag("longitudinal/enable_overshoot_emergency");
  if (params.has("longitudinal/enable_large_tracking_error_emergency")) p.longitudinal.enable_large_tracking_error_emergency = params.flag("longitudinal/enable_large_tracking_error_emergency");
  if (params.has("longitudinal/enable_slope_compensation")) p.longitudinal.enable_slope_compensation = params.flag("longitudinal/enable_slope_compensation");
  if (params.has("longitudinal/enable_keep_stopped_until_steer_convergence")) p.longitudinal.enable_keep_stopped_until_steer_convergence = params.flag("longitudinal/enable_keep_stopped_until_steer_convergence");
  if (params.has("longitudinal/drive_state_stop_dist")) p.longitudinal.state_transition_params.drive_state_stop_dist = params.number("longitudinal/drive_state_stop_dist");
  if (params.has("longitudinal/drive_state_offset_stop_dist")) p.longitudinal.state_transition_params.drive_state_offset_stop_dist = params.number("longitudinal/drive_state_offset_stop_dist");
  if (params.has("longitudinal/stopping_state_stop_dist")) p.longitudinal.state_transition_params.stopping_state_stop_dist = params.number("longitudinal/stopping_state_stop_dist");
  if (params.has("longitudinal/stopped_state_entry_duration_time")) p.longitudinal.state_transition_params.stopped_state_entry_duration_time = params.number("longitudinal/stopped_state_entry_duration_time");
  if (params.has("longitudinal/stopped_state_entry_vel")) p.longitudinal.state_transition_params.stopped_state_entry_vel = params.number("longitudinal/stopped_state_entry_vel");
  if (params.has("longitudinal/stopped_state_entry_acc")) p.longitudinal.state_transition_params.stopped_state_entry_acc = params.number("longitudinal/stopped_state_entry_acc");
  if (params.has("longitudinal/emergency_state_overshoot_stop_dist")) p.longitudinal.state_transition_params.emergency_state_overshoot_stop_dist = params.number("longitudinal/emergency_state_overshoot_stop_dist");
  if (params.has("longitudinal/kp")) p.longitudinal.pid_gains.kp = params.number("longitudinal/kp");
  if (params.has("longitudinal/ki")) p.longitudinal.pid_gains.ki = params.number("longitudinal/ki");
  if (params.has("longitudinal/kd")) p.longitudinal.pid_gains.kd = params.number("longitudinal/kd");
  if (params.has("longitudinal/max_out")) p.longitudinal.pid_limits.max_out = params.number("longitudinal/max_out");
  if (params.has("longitudinal/min_out")) p.longitudinal.pid_limits.min_out = params.number("longitudinal/min_out");
  if (params.has("longitudinal/max_p_effort")) p.longitudinal.pid_limits.max_p_effort = params.number("longitudinal/max_p_effort");
  if (params.has("longitudinal/min_p_effort")) p.longitudinal.pid_limits.min_p_effort = params.number("longitudinal/min_p_effort");
  if (params.has("longitudinal/max_i_effort")) p.longitudinal.pid_limits.max_i_effort = params.number("longitudinal/max_i_effort");
  if (params.has("longitudinal/min_i_effort")) p.longitudinal.pid_limits.min_i_effort = params.number("longitudinal/min_i_effort");
  if (params.has("longitudinal/max_d_effort")) p.longitudinal.pid_limits.max_d_effort = params.number("longitudinal/max_d_effort");
  if (params.has("longitudinal/min_d_effort")) p.longitudinal.pid_limits.min_d_effort = params.number("longitudinal/min_d_effort");
  if (params.has("longitudinal/lpf_vel_error_gain")) p.longitudinal.lpf_vel_error_gain = params.number("longitudinal/lpf_vel_error_gain");
  if (params.has("longitudinal/enable_integration_at_low_speed")) p.longitudinal.enable_integration_at_low_speed = params.flag("longitudinal/enable_integration_at_low_speed");
  if (params.has("longitudinal/current_vel_threshold_pid_integration")) p.longitudinal.current_vel_threshold_pid_integrate = params.number("longitudinal/current_vel_threshold_pid_integration");
  if (params.has("longitudinal/time_threshold_before_pid_integration")) p.longitudinal.time_threshold_before_pid_integrate = params.number("longitudinal/time_threshold_before_pid_integration");
  if (params.has("longitudinal/ff_scale_min")) p.longitudinal.ff_scale_min = params.number("longitudinal/ff_scale_min");
  if (params.has("longitudinal/ff_scale_max")) p.longitudinal.ff_scale_max = params.number("longitudinal/ff_scale_max");
  if (params.has("longitudinal/enable_brake_keeping_before_stop")) p.longitudinal.enable_brake_keeping_before_stop = params.flag("longitudinal/enable_brake_keeping_before_stop");
  if (params.has("longitudinal/brake_keeping_acc")) p.longitudinal.brake_keeping_acc = params.number("longitudinal/brake_keeping_acc");
  if (params.has("longitudinal/smooth_stop_max_strong_acc")) p.longitudinal.smooth_stop_params.max_strong_acc = params.number("longitudinal/smooth_stop_max_strong_acc");
  if (params.has("longitudinal/smooth_stop_min_strong_acc")) p.longitudinal.smooth_stop_params.min_strong_acc = params.number("longitudinal/smooth_stop_min_strong_acc");
  if (params.has("longitudinal/smooth_stop_weak_acc")) p.longitudinal.smooth_stop_params.weak_acc = params.number("longitudinal/smooth_stop_weak_acc");
  if (params.has("longitudinal/smooth_stop_weak_stop_acc")) p.longitudinal.smooth_stop_params.weak_stop_acc = params.number("longitudinal/smooth_stop_weak_stop_acc");
  if (params.has("longitudinal/smooth_stop_strong_stop_acc")) p.longitudinal.smooth_stop_params.strong_stop_acc = params.number("longitudinal/smooth_stop_strong_stop_acc");
  if (params.has("longitudinal/smooth_stop_max_fast_vel")) p.longitudinal.smooth_stop_params.min_fast_vel = params.number("longitudinal/smooth_stop_max_fast_vel");
  if (params.has("longitudinal/smooth_stop_min_running_vel")) p.longitudinal.smooth_stop_params.min_running_vel = params.number("longitudinal/smooth_stop_min_running_vel");
  if (params.has("longitudinal/smooth_stop_min_running_acc")) p.longitudinal.smooth_stop_params.min_running_acc = params.number("longitudinal/smooth_stop_min_running_acc");
  if (params.has("longitudinal/smooth_stop_weak_stop_time")) p.longitudinal.smooth_stop_params.weak_stop_time = params.number("longitudinal/smooth_stop_weak_stop_time");
  if (params.has("longitudinal/smooth_stop_weak_stop_dist")) p.longitudinal.smooth_stop_params.weak_stop_dist = params.number("longitudinal/smooth_stop_weak_stop_dist");
  if (params.has("longitudinal/smooth_stop_strong_stop_dist")) p.longitudinal.smooth_stop_params.strong_stop_dist = params.number("longitudinal/smooth_stop_strong_stop_dist");
  if (params.has("longitudinal/stopped_vel")) p.longitudinal.stopped_state_params.vel = params.number("longitudinal/stopped_vel");
  if (params.has("longitudinal/stopped_acc")) p.longitudinal.stopped_state_params.acc = params.number("longitudinal/stopped_acc");
  if (params.has("longitudinal/emergency_vel")) p.longitudinal.emergency_state_params.vel = params.number("longitudinal/emergency_vel");
  if (params.has("longitudinal/emergency_acc")) p.longitudinal.emergency_state_params.acc = params.number("longitudinal/emergency_acc");
  if (params.has("longitudinal/emergency_jerk")) p.longitudinal.emergency_state_params.jerk = params.number("longitudinal/emergency_jerk");
  if (params.has("longitudinal/lpf_acc_error_gain")) p.longitudinal.lpf_acc_error_gain = params.number("longitudinal/lpf_acc_error_gain");
  if (params.has("longitudinal/acc_feedback_gain")) p.longitudinal.acc_feedback_gain = params.number("longitudinal/acc_feedback_gain");
  if (params.has("longitudinal/max_acc")) p.longitudinal.max_acc = params.number("longitudinal/max_acc");
  if (params.has("longitudinal/min_acc")) p.longitudinal.min_acc = params.number("longitudinal/min_acc");
  if (params.has("longitudinal/max_jerk")) p.longitudinal.max_jerk = params.number("longitudinal/max_jerk");
  if (params.has("longitudinal/min_jerk")) p.longitudinal.min_jerk = params.number("longitudinal/min_jerk");
  if (params.has("longitudinal/max_acc_cmd_diff")) p.longitudinal.max_acc_cmd_diff = params.number("longitudinal/max_acc_cmd_diff");
  if (params.has("longitudinal/adaptive_trajectory_velocity_th")) p.longitudinal.adaptive_trajectory_velocity_th = params.number("longitudinal/adaptive_trajectory_velocity_th");
  if (params.has("longitudinal/lpf_pitch_gain")) p.longitudinal.lpf_pitch_gain = params.number("longitudinal/lpf_pitch_gain");
  if (params.has("longitudinal/max_pitch_rad")) p.longitudinal.max_pitch_rad = params.number("longitudinal/max_pitch_rad");
  if (params.has("longitudinal/min_pitch_rad")) p.longitudinal.min_pitch_rad = params.number("longitudinal/min_pitch_rad");
  if (params.has("ego_nearest_dist_threshold")) p.longitudinal.ego_nearest_dist_threshold = params.number("ego_nearest_dist_threshold");
  if (params.has("ego_nearest_yaw_threshold")) p.longitudinal.ego_nearest_yaw_threshold = params.number("ego_nearest_yaw_threshold");
  if (params.has("wheelbase_m")) p.wheelbase_m = params.number("wheelbase_m");
  if (params.has("maximum_steering_angle_rad")) p.maximum_steering_angle_rad = params.number("maximum_steering_angle_rad");
  if (params.has("nominal_dt_sec")) p.nominal_dt_sec = params.number("nominal_dt_sec");
  if (params.has("minimum_control_dt_sec")) p.minimum_control_dt_sec = params.number("minimum_control_dt_sec");
  if (params.has("maximum_control_dt_sec")) p.maximum_control_dt_sec = params.number("maximum_control_dt_sec");
  if (params.has("acceleration_to_pedal_mps2")) p.acceleration_to_pedal_mps2 = params.number("acceleration_to_pedal_mps2");
  if (params.has("braking_to_pedal_mps2")) p.braking_to_pedal_mps2 = params.number("braking_to_pedal_mps2");
  if (params.has("maximum_accel_pedal")) p.maximum_accel_pedal = params.number("maximum_accel_pedal");
  if (params.has("maximum_brake_pedal")) p.maximum_brake_pedal = params.number("maximum_brake_pedal");
  for (double gain : {p.longitudinal.lpf_vel_error_gain, p.longitudinal.lpf_acc_error_gain, p.longitudinal.lpf_pitch_gain})
    if (gain < 0.0 || gain > 1.0) throw std::invalid_argument("Universe filter gain must be in [0,1]");
  auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
  if (!positive(p.wheelbase_m) || !positive(p.maximum_steering_angle_rad) ||
      !positive(p.nominal_dt_sec) || !positive(p.minimum_control_dt_sec) ||
      p.maximum_control_dt_sec < p.minimum_control_dt_sec ||
      !positive(p.acceleration_to_pedal_mps2) || !positive(p.braking_to_pedal_mps2) ||
      !positive(p.maximum_accel_pedal) || p.maximum_accel_pedal > 1 ||
      !positive(p.maximum_brake_pedal) || p.maximum_brake_pedal > 1 ||
      p.lateral.prediction_horizon < 2 || !positive(p.lateral.prediction_dt) ||
      !positive(p.trajectory_filter.traj_resample_dist) || !positive(p.lateral.steer_tau) ||
      !positive(p.steering_lpf_cutoff_hz) || p.steering_lpf_cutoff_hz >= 0.5/p.nominal_dt_sec ||
      !positive(p.error_deriv_lpf_cutoff_hz) || p.error_deriv_lpf_cutoff_hz >= 0.5/p.nominal_dt_sec ||
      !positive(p.new_traj_duration_time) || p.new_traj_end_dist < 0 ||
      p.trajectory_filter.curvature_smoothing_num_traj < 1 || p.trajectory_filter.curvature_smoothing_num_ref_steer < 1 ||
      p.longitudinal.pid_limits.min_out > p.longitudinal.pid_limits.max_out ||
      p.longitudinal.pid_limits.min_p_effort > p.longitudinal.pid_limits.max_p_effort ||
      p.longitudinal.pid_limits.min_i_effort > p.longitudinal.pid_limits.max_i_effort ||
      p.longitudinal.pid_limits.min_d_effort > p.longitudinal.pid_limits.max_d_effort ||
      p.lateral.input_delay < 0 || p.longitudinal.delay_compensation_time < 0 ||
      p.longitudinal.min_acc > p.longitudinal.max_acc || p.longitudinal.min_jerk > p.longitudinal.max_jerk)
    throw std::invalid_argument("invalid Universe controller parameters");
  p.lateral.low_curvature_weight.terminal_lat_error = p.lateral.nominal_weight.terminal_lat_error;
  p.lateral.low_curvature_weight.terminal_heading_error = p.lateral.nominal_weight.terminal_heading_error;
  p.longitudinal.wheel_base = p.wheelbase_m;
  p.longitudinal.longitudinal_ctrl_period = p.nominal_dt_sec;
  p.longitudinal.minimum_control_dt_sec = p.minimum_control_dt_sec;
  p.longitudinal.maximum_control_dt_sec = p.maximum_control_dt_sec;
  return p;

}
}  // namespace control
