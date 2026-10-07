#pragma once
#include <control/universe/parameters.hpp>
#include <node_pipeline/sdk.hpp>
#include <stdexcept>
namespace control { namespace parameters {
namespace universe_detail {
template<class T> inline void readParameter(const node_pipeline::Parameters& params, const char* key, T& value) {
  if (params.contains(key)) value = params.require<T>(key);
}
}
struct AutowareUniverse {
  universe::Parameters operator()(const node_pipeline::Parameters& nh) const {
    using universe_detail::readParameter;
  auto p = universe::defaultParameters();
  readParameter(nh,"lateral/traj_resample_dist", p.trajectory_filter.traj_resample_dist);
  readParameter(nh,"lateral/use_steer_prediction", p.use_steer_prediction);
  readParameter(nh,"lateral/enable_path_smoothing", p.trajectory_filter.enable_path_smoothing);
  readParameter(nh,"lateral/path_filter_moving_ave_num", p.trajectory_filter.path_filter_moving_ave_num);
  readParameter(nh,"lateral/curvature_smoothing_num_traj", p.trajectory_filter.curvature_smoothing_num_traj);
  readParameter(nh,"lateral/curvature_smoothing_num_ref_steer", p.trajectory_filter.curvature_smoothing_num_ref_steer);
  readParameter(nh,"lateral/extend_trajectory_for_end_yaw_control", p.trajectory_filter.extend_trajectory_for_end_yaw_control);
  readParameter(nh,"lateral/mpc_prediction_horizon", p.lateral.prediction_horizon);
  readParameter(nh,"lateral/mpc_prediction_dt", p.lateral.prediction_dt);
  readParameter(nh,"lateral/mpc_weight_lat_error", p.lateral.nominal_weight.lat_error);
  readParameter(nh,"lateral/mpc_weight_heading_error", p.lateral.nominal_weight.heading_error);
  readParameter(nh,"lateral/mpc_weight_heading_error_squared_vel", p.lateral.nominal_weight.heading_error_squared_vel);
  readParameter(nh,"lateral/mpc_weight_steering_input", p.lateral.nominal_weight.steering_input);
  readParameter(nh,"lateral/mpc_weight_steering_input_squared_vel", p.lateral.nominal_weight.steering_input_squared_vel);
  readParameter(nh,"lateral/mpc_weight_lat_jerk", p.lateral.nominal_weight.lat_jerk);
  readParameter(nh,"lateral/mpc_weight_steer_rate", p.lateral.nominal_weight.steer_rate);
  readParameter(nh,"lateral/mpc_weight_steer_acc", p.lateral.nominal_weight.steer_acc);
  readParameter(nh,"lateral/mpc_low_curvature_weight_lat_error", p.lateral.low_curvature_weight.lat_error);
  readParameter(nh,"lateral/mpc_low_curvature_weight_heading_error", p.lateral.low_curvature_weight.heading_error);
  readParameter(nh,"lateral/mpc_low_curvature_weight_heading_error_squared_vel", p.lateral.low_curvature_weight.heading_error_squared_vel);
  readParameter(nh,"lateral/mpc_low_curvature_weight_steering_input", p.lateral.low_curvature_weight.steering_input);
  readParameter(nh,"lateral/mpc_low_curvature_weight_steering_input_squared_vel", p.lateral.low_curvature_weight.steering_input_squared_vel);
  readParameter(nh,"lateral/mpc_low_curvature_weight_lat_jerk", p.lateral.low_curvature_weight.lat_jerk);
  readParameter(nh,"lateral/mpc_low_curvature_weight_steer_rate", p.lateral.low_curvature_weight.steer_rate);
  readParameter(nh,"lateral/mpc_low_curvature_weight_steer_acc", p.lateral.low_curvature_weight.steer_acc);
  readParameter(nh,"lateral/mpc_low_curvature_thresh_curvature", p.lateral.low_curvature_thresh_curvature);
  readParameter(nh,"lateral/mpc_weight_terminal_lat_error", p.lateral.nominal_weight.terminal_lat_error);
  readParameter(nh,"lateral/mpc_weight_terminal_heading_error", p.lateral.nominal_weight.terminal_heading_error);
  readParameter(nh,"lateral/mpc_zero_ff_steer_deg", p.lateral.zero_ff_steer_deg);
  readParameter(nh,"lateral/mpc_acceleration_limit", p.lateral.acceleration_limit);
  readParameter(nh,"lateral/mpc_velocity_time_constant", p.lateral.velocity_time_constant);
  readParameter(nh,"lateral/mpc_min_prediction_length", p.lateral.min_prediction_length);
  readParameter(nh,"lateral/input_delay", p.lateral.input_delay);
  readParameter(nh,"lateral/vehicle_model_steer_tau", p.lateral.steer_tau);
  readParameter(nh,"lateral/steering_lpf_cutoff_hz", p.steering_lpf_cutoff_hz);
  readParameter(nh,"lateral/error_deriv_lpf_cutoff_hz", p.error_deriv_lpf_cutoff_hz);
  readParameter(nh,"lateral/stop_state_entry_ego_speed", p.stop_state_entry_ego_speed);
  readParameter(nh,"lateral/stop_state_entry_target_speed", p.stop_state_entry_target_speed);
  readParameter(nh,"lateral/converged_steer_rad", p.converged_steer_rad);
  readParameter(nh,"lateral/keep_steer_control_until_converged", p.keep_steer_control_until_converged);
  readParameter(nh,"lateral/new_traj_duration_time", p.new_traj_duration_time);
  readParameter(nh,"lateral/new_traj_end_dist", p.new_traj_end_dist);
  readParameter(nh,"lateral/mpc_converged_threshold_rps", p.mpc_converged_threshold_rps);
  readParameter(nh,"longitudinal/delay_compensation_time", p.longitudinal.delay_compensation_time);
  readParameter(nh,"longitudinal/enable_smooth_stop", p.longitudinal.enable_smooth_stop);
  readParameter(nh,"longitudinal/enable_overshoot_emergency", p.longitudinal.enable_overshoot_emergency);
  readParameter(nh,"longitudinal/enable_large_tracking_error_emergency", p.longitudinal.enable_large_tracking_error_emergency);
  readParameter(nh,"longitudinal/enable_slope_compensation", p.longitudinal.enable_slope_compensation);
  readParameter(nh,"longitudinal/enable_keep_stopped_until_steer_convergence", p.longitudinal.enable_keep_stopped_until_steer_convergence);
  readParameter(nh,"longitudinal/drive_state_stop_dist", p.longitudinal.state_transition_params.drive_state_stop_dist);
  readParameter(nh,"longitudinal/drive_state_offset_stop_dist", p.longitudinal.state_transition_params.drive_state_offset_stop_dist);
  readParameter(nh,"longitudinal/stopping_state_stop_dist", p.longitudinal.state_transition_params.stopping_state_stop_dist);
  readParameter(nh,"longitudinal/stopped_state_entry_duration_time", p.longitudinal.state_transition_params.stopped_state_entry_duration_time);
  readParameter(nh,"longitudinal/stopped_state_entry_vel", p.longitudinal.state_transition_params.stopped_state_entry_vel);
  readParameter(nh,"longitudinal/stopped_state_entry_acc", p.longitudinal.state_transition_params.stopped_state_entry_acc);
  readParameter(nh,"longitudinal/emergency_state_overshoot_stop_dist", p.longitudinal.state_transition_params.emergency_state_overshoot_stop_dist);
  readParameter(nh,"longitudinal/kp", p.longitudinal.pid_gains.kp);
  readParameter(nh,"longitudinal/ki", p.longitudinal.pid_gains.ki);
  readParameter(nh,"longitudinal/kd", p.longitudinal.pid_gains.kd);
  readParameter(nh,"longitudinal/max_out", p.longitudinal.pid_limits.max_out);
  readParameter(nh,"longitudinal/min_out", p.longitudinal.pid_limits.min_out);
  readParameter(nh,"longitudinal/max_p_effort", p.longitudinal.pid_limits.max_p_effort);
  readParameter(nh,"longitudinal/min_p_effort", p.longitudinal.pid_limits.min_p_effort);
  readParameter(nh,"longitudinal/max_i_effort", p.longitudinal.pid_limits.max_i_effort);
  readParameter(nh,"longitudinal/min_i_effort", p.longitudinal.pid_limits.min_i_effort);
  readParameter(nh,"longitudinal/max_d_effort", p.longitudinal.pid_limits.max_d_effort);
  readParameter(nh,"longitudinal/min_d_effort", p.longitudinal.pid_limits.min_d_effort);
  readParameter(nh,"longitudinal/lpf_vel_error_gain", p.longitudinal.lpf_vel_error_gain);
  readParameter(nh,"longitudinal/enable_integration_at_low_speed", p.longitudinal.enable_integration_at_low_speed);
  readParameter(nh,"longitudinal/current_vel_threshold_pid_integration", p.longitudinal.current_vel_threshold_pid_integrate);
  readParameter(nh,"longitudinal/time_threshold_before_pid_integration", p.longitudinal.time_threshold_before_pid_integrate);
  readParameter(nh,"longitudinal/ff_scale_min", p.longitudinal.ff_scale_min);
  readParameter(nh,"longitudinal/ff_scale_max", p.longitudinal.ff_scale_max);
  readParameter(nh,"longitudinal/enable_brake_keeping_before_stop", p.longitudinal.enable_brake_keeping_before_stop);
  readParameter(nh,"longitudinal/brake_keeping_acc", p.longitudinal.brake_keeping_acc);
  readParameter(nh,"longitudinal/smooth_stop_max_strong_acc", p.longitudinal.smooth_stop_params.max_strong_acc);
  readParameter(nh,"longitudinal/smooth_stop_min_strong_acc", p.longitudinal.smooth_stop_params.min_strong_acc);
  readParameter(nh,"longitudinal/smooth_stop_weak_acc", p.longitudinal.smooth_stop_params.weak_acc);
  readParameter(nh,"longitudinal/smooth_stop_weak_stop_acc", p.longitudinal.smooth_stop_params.weak_stop_acc);
  readParameter(nh,"longitudinal/smooth_stop_strong_stop_acc", p.longitudinal.smooth_stop_params.strong_stop_acc);
  readParameter(nh,"longitudinal/smooth_stop_max_fast_vel", p.longitudinal.smooth_stop_params.min_fast_vel);
  readParameter(nh,"longitudinal/smooth_stop_min_running_vel", p.longitudinal.smooth_stop_params.min_running_vel);
  readParameter(nh,"longitudinal/smooth_stop_min_running_acc", p.longitudinal.smooth_stop_params.min_running_acc);
  readParameter(nh,"longitudinal/smooth_stop_weak_stop_time", p.longitudinal.smooth_stop_params.weak_stop_time);
  readParameter(nh,"longitudinal/smooth_stop_weak_stop_dist", p.longitudinal.smooth_stop_params.weak_stop_dist);
  readParameter(nh,"longitudinal/smooth_stop_strong_stop_dist", p.longitudinal.smooth_stop_params.strong_stop_dist);
  readParameter(nh,"longitudinal/stopped_vel", p.longitudinal.stopped_state_params.vel);
  readParameter(nh,"longitudinal/stopped_acc", p.longitudinal.stopped_state_params.acc);
  readParameter(nh,"longitudinal/emergency_vel", p.longitudinal.emergency_state_params.vel);
  readParameter(nh,"longitudinal/emergency_acc", p.longitudinal.emergency_state_params.acc);
  readParameter(nh,"longitudinal/emergency_jerk", p.longitudinal.emergency_state_params.jerk);
  readParameter(nh,"longitudinal/lpf_acc_error_gain", p.longitudinal.lpf_acc_error_gain);
  readParameter(nh,"longitudinal/acc_feedback_gain", p.longitudinal.acc_feedback_gain);
  readParameter(nh,"longitudinal/max_acc", p.longitudinal.max_acc);
  readParameter(nh,"longitudinal/min_acc", p.longitudinal.min_acc);
  readParameter(nh,"longitudinal/max_jerk", p.longitudinal.max_jerk);
  readParameter(nh,"longitudinal/min_jerk", p.longitudinal.min_jerk);
  readParameter(nh,"longitudinal/max_acc_cmd_diff", p.longitudinal.max_acc_cmd_diff);
  readParameter(nh,"longitudinal/adaptive_trajectory_velocity_th", p.longitudinal.adaptive_trajectory_velocity_th);
  readParameter(nh,"longitudinal/lpf_pitch_gain", p.longitudinal.lpf_pitch_gain);
  readParameter(nh,"longitudinal/max_pitch_rad", p.longitudinal.max_pitch_rad);
  readParameter(nh,"longitudinal/min_pitch_rad", p.longitudinal.min_pitch_rad);
  readParameter(nh,"ego_nearest_dist_threshold", p.longitudinal.ego_nearest_dist_threshold);
  readParameter(nh,"ego_nearest_yaw_threshold", p.longitudinal.ego_nearest_yaw_threshold);
  readParameter(nh,"wheelbase_m", p.wheelbase_m);
  readParameter(nh,"maximum_steering_angle_rad", p.maximum_steering_angle_rad);
  readParameter(nh,"nominal_dt_sec", p.nominal_dt_sec);
  readParameter(nh,"minimum_control_dt_sec", p.minimum_control_dt_sec);
  readParameter(nh,"maximum_control_dt_sec", p.maximum_control_dt_sec);
  readParameter(nh,"acceleration_to_pedal_mps2", p.acceleration_to_pedal_mps2);
  readParameter(nh,"braking_to_pedal_mps2", p.braking_to_pedal_mps2);
  readParameter(nh,"maximum_accel_pedal", p.maximum_accel_pedal);
  readParameter(nh,"maximum_brake_pedal", p.maximum_brake_pedal);
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
};
}}  // namespace control::parameters
