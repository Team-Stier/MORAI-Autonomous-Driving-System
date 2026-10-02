#pragma once
#include <control/tracking/parameters.hpp>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <cmath>
#include <stdexcept>
namespace control { namespace tracking {
namespace config_detail {
using XmlValue = XmlRpc::XmlRpcValue;
void requirePositive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

void requireNonNegative(const char* name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

XmlValue requiredParameter(const ros::NodeHandle& node, const char* name) {
  XmlValue value;
  if (!node.getParam(name, value)) {
    throw std::invalid_argument(std::string("required private parameter '~") +
                                name + "' is missing");
  }
  return value;
}

std::string requiredString(const ros::NodeHandle& node, const char* name) {
  const XmlValue value = requiredParameter(node, name);
  if (value.getType() != XmlValue::TypeString) {
    throw std::invalid_argument(std::string("~") + name +
                                " must be a string");
  }
  return static_cast<std::string>(value);
}

double requiredDouble(const ros::NodeHandle& node, const char* name) {
  const XmlValue value = requiredParameter(node, name);
  if (value.getType() == XmlValue::TypeDouble) {
    const double result = static_cast<double>(value);
    if (!std::isfinite(result)) throw std::invalid_argument(std::string(name) + " must be finite");
    return result;
  }
  if (value.getType() == XmlValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::invalid_argument(std::string("~") + name + " must be numeric");
}

int requiredInt(const ros::NodeHandle& node, const char* name) {
  const XmlValue value = requiredParameter(node, name);
  if (value.getType() != XmlValue::TypeInt) {
    throw std::invalid_argument(std::string("~") + name +
                                " must be an integer");
  }
  return static_cast<int>(value);
}

bool requiredBool(const ros::NodeHandle& node, const char* name) {
  const XmlValue value = requiredParameter(node, name);
  if (value.getType() != XmlValue::TypeBoolean) {
    throw std::invalid_argument(std::string("~") + name +
                                " must be a boolean");
  }
  return static_cast<bool>(value);
}

}
inline Parameters loadParameters(const ros::NodeHandle& params) {
  using namespace config_detail;
  constexpr double kDegreesToRadians = 0.017453292519943295;
  constexpr double kKilometresPerHourToMetresPerSecond = 1.0 / 3.6;
  Parameters config;
  config.minimum_dt_sec = requiredDouble(params, "minimum_control_dt_sec");
  config.maximum_dt_sec = requiredDouble(params, "maximum_control_dt_sec");
  config.speed_filter_time_constant_sec = requiredDouble(params, "speed_filter_time_constant_sec");
  config.curvature_preview_distance_m = requiredDouble(params, "lookahead_curvature_preview_distance_m");
  config.curvature_sample_spacing_m = requiredDouble(params, "curvature_sample_spacing_m");
  config.wheel_corridor.vehicle_width_m = requiredDouble(params, "vehicle_width_m");
  config.wheel_corridor.lane_half_width_m = requiredDouble(params, "lane_half_width_m");
  config.lateral_controller = requiredString(params, "lateral_controller");
  config.longitudinal_controller = requiredString(params, "longitudinal_controller");
  config.nominal_dt_sec = requiredDouble(params, "nominal_dt_sec");
  const double maximum_steering_angle_deg = requiredDouble(params, "maximum_steering_angle_deg");
  const double mpc_maximum_steering_rate_deg_per_sec = requiredDouble(params, "mpc_maximum_steering_rate_deg_per_sec");
  const double stanley_maximum_steering_rate_deg_per_sec = requiredDouble(params, "stanley_maximum_steering_rate_deg_per_sec");
  const double hybrid_maximum_steering_rate_deg_per_sec = requiredDouble(params, "hybrid_maximum_steering_rate_deg_per_sec");
  const double hybrid_low_curvature_maximum_steering_rate_deg_per_sec = requiredDouble(params, "hybrid_low_curvature_maximum_steering_rate_deg_per_sec");
  const double hybrid_full_steering_rate_curvature_m_inv = requiredDouble(params, "hybrid_full_steering_rate_curvature_m_inv");
  const double hybrid_transition_reference_speed_kph = requiredDouble(params, "hybrid_transition_reference_speed_kph");
  const double speed_coast_overspeed_kph = requiredDouble(params, "speed_coast_overspeed_kph");
  const double speed_brake_overspeed_kph = requiredDouble(params, "speed_brake_overspeed_kph");
  const double hard_brake_activation_speed_kph = requiredDouble(params, "hard_brake_activation_speed_kph");
  const double lane_clearance_recovery_start_m = requiredDouble(params, "lane_clearance_recovery_start_m");
  const double lane_clearance_recovery_full_m = requiredDouble(params, "lane_clearance_recovery_full_m");
  config.pure_pursuit.wheelbase_m = requiredDouble(params, "wheelbase_m");
  config.mpc.wheelbase = config.pure_pursuit.wheelbase_m;
  config.stanley.wheelbase_m = config.pure_pursuit.wheelbase_m;
  config.hybrid.lane_clearance_recovery_start_m = lane_clearance_recovery_start_m;
  config.hybrid.lane_clearance_recovery_full_m = lane_clearance_recovery_full_m;
  config.pure_pursuit.lookahead_base_m = requiredDouble(params, "lookahead_base_m");
  config.pure_pursuit.lookahead_speed_gain_sec = requiredDouble(params, "lookahead_speed_gain_sec");
  config.pure_pursuit.lookahead_curvature_gain_m = requiredDouble(params, "lookahead_curvature_gain_m");
  config.pure_pursuit.lookahead_min_m = requiredDouble(params, "lookahead_min_m");
  config.pure_pursuit.lookahead_max_m = requiredDouble(params, "lookahead_max_m");
  config.pure_pursuit.minimum_target_distance_m = requiredDouble(params, "minimum_target_distance_m");
  config.mpc.solver = requiredString(params, "mpc_solver");
  config.mpc.prediction_time = requiredDouble(params, "mpc_prediction_time_sec");
  config.mpc.horizon = requiredInt(params, "mpc_horizon_steps");
  config.mpc.optimizer_iterations = requiredInt(params, "mpc_optimizer_iterations");
  config.mpc.resample_ds = requiredDouble(params, "mpc_resample_distance_m");
  config.mpc.w_lateral = requiredDouble(params, "mpc_weight_lateral");
  config.mpc.w_heading = requiredDouble(params, "mpc_weight_heading");
  config.mpc.w_steering = requiredDouble(params, "mpc_weight_steering");
  config.mpc.w_rate = requiredDouble(params, "mpc_weight_steering_rate");
  config.mpc.w_rate_change = requiredDouble(params, "mpc_weight_steering_rate_change");
  config.mpc.w_terminal_lateral = requiredDouble(params, "mpc_weight_terminal_lateral");
  config.mpc.w_terminal_heading = requiredDouble(params, "mpc_weight_terminal_heading");
  config.mpc.adaptation_start_curvature = requiredDouble(params, "mpc_adaptation_start_curvature_m_inv");
  config.mpc.adaptation_full_curvature = requiredDouble(params, "mpc_adaptation_full_curvature_m_inv");
  config.mpc.curve_heading_weight_multiplier = requiredDouble(params, "mpc_curve_heading_weight_multiplier");
  config.mpc.curve_steering_weight_multiplier = requiredDouble(params, "mpc_curve_steering_weight_multiplier");
  config.mpc.curve_rate_change_weight_multiplier = requiredDouble(
      params, "mpc_curve_rate_change_weight_multiplier");
  config.mpc.straight_rate_weight_multiplier = requiredDouble(params, "mpc_straight_rate_weight_multiplier");
  config.mpc.straight_rate_change_weight_multiplier = requiredDouble(
      params, "mpc_straight_rate_change_weight_multiplier");
  config.mpc.yaw_rate_damping_gain = requiredDouble(params, "mpc_yaw_rate_damping_gain");
  config.mpc.yaw_rate_damping_fade_curvature = requiredDouble(
      params, "mpc_yaw_rate_damping_fade_curvature_m_inv");
  config.mpc.swept_path_compensation_enabled = requiredBool(params, "mpc_swept_path_compensation_enabled");
  config.mpc.swept_path_vehicle_width = requiredDouble(params, "mpc_swept_path_vehicle_width_m");
  config.mpc.swept_path_gain = requiredDouble(params, "mpc_swept_path_compensation_gain");
  config.mpc.swept_path_activation_start_curvature = requiredDouble(
      params, "mpc_swept_path_activation_start_curvature_m_inv");
  config.mpc.swept_path_activation_full_curvature = requiredDouble(
      params, "mpc_swept_path_activation_full_curvature_m_inv");
  config.mpc.swept_path_maximum_offset = requiredDouble(params, "mpc_swept_path_maximum_offset_m");
  config.mpc.swept_path_curvature_smoothing_window = requiredInt(params, "mpc_swept_path_curvature_smoothing_window_points");
  config.mpc.straight_cte_filter_time_constant = requiredDouble(params, "mpc_straight_cte_filter_time_constant_sec");
  config.mpc.straight_cte_filter_minimum_speed = requiredDouble(params, "mpc_straight_cte_filter_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_cte_filter_maximum_curvature = requiredDouble(
      params, "mpc_straight_cte_filter_maximum_curvature_m_inv");
  config.mpc.straight_cte_maximum_rate = requiredDouble(
      params, "mpc_straight_cte_maximum_rate_mps");
  config.mpc.straight_steering_rate_filter_time_constant = requiredDouble(
      params, "mpc_straight_steering_rate_filter_time_constant_sec");
  config.mpc.straight_steering_rate_filter_minimum_speed = requiredDouble(
      params, "mpc_straight_steering_rate_filter_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_steering_rate_filter_maximum_curvature = requiredDouble(
      params,
      "mpc_straight_steering_rate_filter_maximum_curvature_m_inv");
  config.mpc.straight_spatial_fit_enabled = requiredBool(params, "mpc_straight_spatial_fit_enabled");
  config.mpc.straight_spatial_fit_start_speed = requiredDouble(
      params, "mpc_straight_spatial_fit_start_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_spatial_fit_full_speed = requiredDouble(
      params, "mpc_straight_spatial_fit_full_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_spatial_fit_full_curvature = requiredDouble(
      params, "mpc_straight_spatial_fit_full_curvature_m_inv");
  config.mpc.straight_spatial_fit_off_curvature = requiredDouble(
      params, "mpc_straight_spatial_fit_off_curvature_m_inv");
  config.mpc.straight_spatial_fit_base_preview = requiredDouble(
      params, "mpc_straight_spatial_fit_base_preview_m");
  config.mpc.straight_spatial_fit_speed_gain = requiredDouble(
      params, "mpc_straight_spatial_fit_speed_gain_sec");
  config.mpc.straight_spatial_fit_maximum_preview = requiredDouble(
      params, "mpc_straight_spatial_fit_maximum_preview_m");
  config.mpc.straight_spatial_fit_lateral_weight_multiplier = requiredDouble(
      params,
      "mpc_straight_spatial_fit_lateral_weight_multiplier");
  config.mpc.curve_feedforward_seed_gain = requiredDouble(params, "mpc_curve_feedforward_seed_gain");
  config.mpc.curve_feedforward_seed_lookahead = requiredDouble(params, "mpc_curve_feedforward_seed_lookahead_m");
  config.mpc.straight_yaw_rate_steering_estimate_gain = requiredDouble(
      params, "mpc_straight_yaw_rate_steering_estimate_gain");
  config.mpc.straight_yaw_rate_steering_estimate_minimum_speed = requiredDouble(
          params,
          "mpc_straight_yaw_rate_steering_estimate_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_yaw_rate_steering_estimate_filter_time_constant = requiredDouble(
          params,
          "mpc_straight_yaw_rate_steering_estimate_filter_time_constant_sec");
  config.mpc.curve_yaw_rate_steering_estimate_gain = requiredDouble(
      params, "mpc_curve_yaw_rate_steering_estimate_gain");
  config.mpc.curve_yaw_rate_steering_estimate_minimum_speed = requiredDouble(
          params,
          "mpc_curve_yaw_rate_steering_estimate_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.curve_yaw_rate_steering_estimate_activation_start_curvature = requiredDouble(
          params,
          "mpc_curve_yaw_rate_steering_estimate_activation_start_curvature_m_inv");
  config.mpc.curve_yaw_rate_steering_estimate_activation_full_curvature = requiredDouble(
          params,
          "mpc_curve_yaw_rate_steering_estimate_activation_full_curvature_m_inv");
  config.mpc.curve_yaw_rate_steering_estimate_blend_time_constant = requiredDouble(
          params,
          "mpc_curve_yaw_rate_steering_estimate_blend_time_constant_sec");
  config.mpc.straight_dynamic_model_enabled = requiredBool(params, "mpc_straight_dynamic_model_enabled");
  config.mpc.straight_dynamic_model_minimum_speed = requiredDouble(params,
                     "mpc_straight_dynamic_model_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_dynamic_model_maximum_curvature = requiredDouble(
      params, "mpc_straight_dynamic_model_maximum_curvature_m_inv");
  config.mpc.straight_dynamic_sideslip_weight = requiredDouble(
      params, "mpc_weight_straight_dynamic_sideslip");
  config.mpc.straight_dynamic_yaw_rate_weight = requiredDouble(
      params, "mpc_weight_straight_dynamic_yaw_rate");
  config.mpc.optimizer_initial_step = requiredDouble(params, "mpc_optimizer_initial_step_deg_per_sec") *
      kDegreesToRadians;
  config.mpc.optimizer_min_step = requiredDouble(params, "mpc_optimizer_min_step_deg_per_sec") *
      kDegreesToRadians;
  config.mpc.optimizer_step_decay = requiredDouble(params, "mpc_optimizer_step_decay");
  config.mpc.optimizer_gradient_epsilon = requiredDouble(params, "mpc_optimizer_gradient_epsilon_deg_per_sec") *
      kDegreesToRadians;
  config.mpc.optimizer_gradient_step = requiredDouble(params, "mpc_optimizer_gradient_step");
  config.mpc.optimizer_gradient_tolerance = requiredDouble(params, "mpc_optimizer_gradient_tolerance");
  config.mpc.optimizer_line_search_steps = requiredInt(params, "mpc_optimizer_line_search_steps");
  config.mpc.optimizer_use_warm_start = requiredBool(params, "mpc_use_warm_start");
  config.mpc.optimizer_multi_resolution = requiredBool(params, "mpc_use_multi_resolution");
  config.mpc.use_previous_control_rate_change = requiredBool(params, "mpc_use_previous_control");
  config.mpc.optimizer_model_actuator = requiredBool(params, "mpc_model_actuator");
  config.mpc.actuator_delay = requiredDouble(params, "mpc_actuator_delay_sec");
  config.mpc.actuator_time_constant = requiredDouble(params, "mpc_actuator_time_constant_sec");
  config.mpc.time_based_horizon = requiredBool(params, "mpc_time_based_horizon");
  config.mpc.adaptive_prediction_time = requiredBool(params, "mpc_adaptive_prediction_time");
  config.mpc.minimum_prediction_time = requiredDouble(params, "mpc_minimum_prediction_time_sec");
  config.mpc.maximum_prediction_time = requiredDouble(params, "mpc_maximum_prediction_time_sec");
  config.mpc.minimum_horizon_steps = requiredInt(params, "mpc_minimum_horizon_steps");
  config.mpc.maximum_horizon_steps = requiredInt(params, "mpc_maximum_horizon_steps");
  config.mpc.minimum_preview_distance = requiredDouble(params, "mpc_minimum_preview_distance_m");
  config.mpc.prediction_speed_gain = requiredDouble(params, "mpc_prediction_speed_gain_sec_per_mps");
  config.stanley.gain = requiredDouble(params, "stanley_gain");
  config.stanley.softening_speed_mps = requiredDouble(params, "stanley_softening_speed_mps");
  config.stanley.minimum_control_speed_mps = requiredDouble(params, "stanley_minimum_control_speed_mps");
  config.stanley.heading_window_m = requiredDouble(params, "stanley_heading_window_m");
  config.stanley.heading_error_gain = requiredDouble(params, "stanley_heading_error_gain");
  config.stanley.curvature_feedforward_gain = requiredDouble(params, "stanley_curvature_feedforward_gain");
  config.stanley.curvature_preview_distance_m = requiredDouble(params, "stanley_curvature_preview_distance_m");
  config.stanley.yaw_rate_damping_gain_sec = requiredDouble(params, "stanley_yaw_rate_damping_gain_sec");
  config.stanley.yaw_rate_damping_nonlinear_gain_sec2 = requiredDouble(
      params, "stanley_yaw_rate_damping_nonlinear_gain_sec2");
  config.hybrid.imm.mass_kg = requiredDouble(params, "hybrid_mass_kg");
  config.hybrid.imm.yaw_inertia_kgm2 = requiredDouble(params, "hybrid_yaw_inertia_kgm2");
  config.hybrid.imm.front_cornering_stiffness_n_per_rad = requiredDouble(
      params, "hybrid_front_cornering_stiffness_n_per_rad");
  config.hybrid.imm.rear_cornering_stiffness_n_per_rad = requiredDouble(
      params, "hybrid_rear_cornering_stiffness_n_per_rad");
  config.hybrid.imm.front_axle_to_cg_m = requiredDouble(params, "hybrid_front_axle_to_cg_m");
  config.hybrid.imm.rear_axle_to_cg_m = requiredDouble(params, "hybrid_rear_axle_to_cg_m");
  config.mpc.dynamic_mass = config.hybrid.imm.mass_kg;
  config.mpc.dynamic_yaw_inertia = config.hybrid.imm.yaw_inertia_kgm2;
  config.mpc.dynamic_front_cornering_stiffness = config.hybrid.imm.front_cornering_stiffness_n_per_rad;
  config.mpc.dynamic_rear_cornering_stiffness = config.hybrid.imm.rear_cornering_stiffness_n_per_rad;
  config.mpc.dynamic_front_axle_to_cg = config.hybrid.imm.front_axle_to_cg_m;
  config.mpc.dynamic_rear_axle_to_cg = config.hybrid.imm.rear_axle_to_cg_m;
  config.hybrid.imm.process_noise_sideslip = requiredDouble(params, "hybrid_process_noise_sideslip");
  config.hybrid.imm.process_noise_yaw_rate = requiredDouble(params, "hybrid_process_noise_yaw_rate");
  config.hybrid.imm.measurement_noise_sideslip = requiredDouble(params, "hybrid_measurement_noise_sideslip");
  config.hybrid.imm.measurement_noise_yaw_rate = requiredDouble(params, "hybrid_measurement_noise_yaw_rate");
  config.hybrid.imm.initial_covariance_sideslip = requiredDouble(params, "hybrid_initial_covariance_sideslip");
  config.hybrid.imm.initial_covariance_yaw_rate = requiredDouble(params, "hybrid_initial_covariance_yaw_rate");
  config.hybrid.imm.initial_pure_pursuit_probability = requiredDouble(
      params, "hybrid_initial_pure_pursuit_probability");
  config.hybrid.imm.initial_stanley_probability = requiredDouble(params, "hybrid_initial_stanley_probability");
  config.hybrid.imm.stanley_probability_min = requiredDouble(params, "hybrid_stanley_probability_min");
  config.hybrid.imm.stanley_probability_max = requiredDouble(params, "hybrid_stanley_probability_max");
  config.hybrid.imm.transition_pure_pursuit_to_pure_pursuit = requiredDouble(
          params,
          "hybrid_transition_pure_pursuit_to_pure_pursuit");
  config.hybrid.imm.transition_pure_pursuit_to_stanley = requiredDouble(params,
                     "hybrid_transition_pure_pursuit_to_stanley");
  config.hybrid.imm.transition_stanley_to_pure_pursuit = requiredDouble(params,
                     "hybrid_transition_stanley_to_pure_pursuit");
  config.hybrid.imm.transition_stanley_to_stanley = requiredDouble(params,
                     "hybrid_transition_stanley_to_stanley");
  config.hybrid.imm.transition_speed_gain = requiredDouble(params, "hybrid_transition_speed_gain");
  config.hybrid.imm.transition_reference_speed_mps = hybrid_transition_reference_speed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.hybrid.imm.minimum_model_speed_mps = requiredDouble(params, "hybrid_minimum_model_speed_mps");
  config.hybrid.pure_pursuit_cross_track_correction_gain = requiredDouble(
          params,
          "hybrid_pure_pursuit_cross_track_correction_gain");
  config.hybrid.curve_preview_stanley_weight_start_m_inv = requiredDouble(
          params,
          "hybrid_curve_preview_stanley_weight_start_m_inv");
  config.hybrid.curve_preview_stanley_weight_full_m_inv = requiredDouble(
          params,
          "hybrid_curve_preview_stanley_weight_full_m_inv");
  config.hybrid.curve_preview_stanley_minimum_weight = requiredDouble(
          params,
          "hybrid_curve_preview_stanley_minimum_weight");
  config.hybrid.heading_lag_stanley_weight_start_rad = requiredDouble(
          params,
          "hybrid_heading_lag_stanley_weight_start_deg") *
      kDegreesToRadians;
  config.hybrid.heading_lag_stanley_weight_full_rad = requiredDouble(
          params,
          "hybrid_heading_lag_stanley_weight_full_deg") *
      kDegreesToRadians;
  config.hybrid.heading_lag_stanley_minimum_weight = requiredDouble(
          params,
          "hybrid_heading_lag_stanley_minimum_weight");
  config.hybrid.candidate_conflict_curvature_threshold_m_inv = requiredDouble(
          params,
          "hybrid_candidate_conflict_curvature_threshold_m_inv");
  config.hybrid.candidate_conflict_cross_track_threshold_m = requiredDouble(
          params,
          "hybrid_candidate_conflict_cross_track_threshold_m");
  config.hybrid.cross_track_recovery_full_scale_m = requiredDouble(
          params,
          "hybrid_cross_track_recovery_full_scale_m");
  config.hybrid.cross_track_recovery_heading_error_suppression_start_rad = requiredDouble(
          params,
          "hybrid_cross_track_recovery_heading_error_suppression_start_deg") *
      kDegreesToRadians;
  config.hybrid.cross_track_recovery_heading_error_suppression_full_rad = requiredDouble(
          params,
          "hybrid_cross_track_recovery_heading_error_suppression_full_deg") *
      kDegreesToRadians;
  config.hybrid.cross_track_recovery_heading_error_maximum_suppression_ratio = requiredDouble(
          params,
          "hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio");
  config.hybrid.steering_return_rate_multiplier = requiredDouble(
      params, "hybrid_steering_return_rate_multiplier");
  config.pid.kp = requiredDouble(params, "speed_kp");
  config.pid.ki = requiredDouble(params, "speed_ki");
  config.pid.kd = requiredDouble(params, "speed_kd");
  config.pid.integral_limit = requiredDouble(params, "speed_integral_limit");
  config.pid.integral_unwind_rate_per_sec = requiredDouble(
      params, "speed_integral_unwind_rate_per_sec");
  config.pid.error_deadband_mps = requiredDouble(params, "speed_error_deadband_mps");
  config.pid.accel_feedforward_gain_per_mps = requiredDouble(params, "speed_accel_feedforward_gain_per_mps");
  config.pid.coast_overspeed_threshold_mps = speed_coast_overspeed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.pid.brake_overspeed_threshold_mps = speed_brake_overspeed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.pid.hard_brake_activation_speed_mps = hard_brake_activation_speed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.pid.minimum_hard_brake_command = requiredDouble(params, "minimum_hard_brake_command");
  config.pid.maximum_accel = requiredDouble(params, "maximum_accel_command");
  config.pid.maximum_brake = requiredDouble(params, "maximum_brake_command");
  config.pid.command_rate_limit_per_sec = requiredDouble(
      params, "longitudinal_command_rate_limit_per_sec");
  config.longitudinal_mpc.solver = requiredString(params, "longitudinal_mpc_solver");
  config.longitudinal_mpc.horizon_steps = requiredInt(params, "longitudinal_mpc_horizon_steps");
  config.longitudinal_mpc.optimizer_iterations = requiredInt(params, "longitudinal_mpc_optimizer_iterations");
  config.longitudinal_mpc.line_search_steps = requiredInt(params, "longitudinal_mpc_line_search_steps");
  config.longitudinal_mpc.prediction_dt_sec = requiredDouble(params, "longitudinal_mpc_prediction_dt_sec");
  config.longitudinal_mpc.gradient_epsilon = requiredDouble(params, "longitudinal_mpc_gradient_epsilon");
  config.longitudinal_mpc.gradient_step = requiredDouble(params, "longitudinal_mpc_gradient_step");
  config.longitudinal_mpc.gradient_tolerance = requiredDouble(params, "longitudinal_mpc_gradient_tolerance");
  config.longitudinal_mpc.coordinate_initial_step = requiredDouble(
      params, "longitudinal_mpc_coordinate_initial_step");
  config.longitudinal_mpc.coordinate_minimum_step = requiredDouble(
      params, "longitudinal_mpc_coordinate_minimum_step");
  config.longitudinal_mpc.command_step_decay = requiredDouble(params, "longitudinal_mpc_command_step_decay");
  config.longitudinal_mpc.maximum_speed_mps = requiredDouble(params, "maximum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.longitudinal_mpc.hard_brake_activation_speed_mps = config.pid.hard_brake_activation_speed_mps;
  config.longitudinal_mpc.minimum_hard_brake_command = config.pid.minimum_hard_brake_command;
  config.longitudinal_mpc.maximum_accel_command = config.pid.maximum_accel;
  config.longitudinal_mpc.maximum_brake_command = config.pid.maximum_brake;
  config.longitudinal_mpc.command_rate_limit_per_sec = config.pid.command_rate_limit_per_sec;
  config.longitudinal_mpc.acceleration_gain_mps2 = requiredDouble(
      params, "longitudinal_mpc_acceleration_gain_mps2");
  config.longitudinal_mpc.braking_gain_mps2 = requiredDouble(params, "longitudinal_mpc_braking_gain_mps2");
  config.longitudinal_mpc.actuator_time_constant_sec = requiredDouble(
      params, "longitudinal_mpc_actuator_time_constant_sec");
  config.longitudinal_mpc.maximum_acceleration_mps2 = requiredDouble(
      params, "longitudinal_mpc_maximum_acceleration_mps2");
  config.longitudinal_mpc.maximum_deceleration_mps2 = requiredDouble(
      params, "longitudinal_mpc_maximum_deceleration_mps2");
  config.longitudinal_mpc.maximum_jerk_mps3 = requiredDouble(params, "longitudinal_mpc_maximum_jerk_mps3");
  config.longitudinal_mpc.acceleration_estimate_filter_time_constant_sec = requiredDouble(params,
                     "longitudinal_mpc_acceleration_filter_tau_sec");
  config.longitudinal_mpc.speed_weight = requiredDouble(params, "longitudinal_mpc_weight_speed");
  config.longitudinal_mpc.acceleration_weight = requiredDouble(params, "longitudinal_mpc_weight_acceleration");
  config.longitudinal_mpc.jerk_weight = requiredDouble(params, "longitudinal_mpc_weight_jerk");
  config.longitudinal_mpc.command_weight = requiredDouble(params, "longitudinal_mpc_weight_command");
  config.longitudinal_mpc.command_rate_weight = requiredDouble(params, "longitudinal_mpc_weight_command_rate");
  config.longitudinal_mpc.terminal_speed_weight = requiredDouble(
      params, "longitudinal_mpc_weight_terminal_speed");
  config.pure_pursuit.maximum_steering_angle_rad = maximum_steering_angle_deg * kDegreesToRadians;
  config.mpc.dt = config.nominal_dt_sec;
  config.mpc.max_steering = config.pure_pursuit.maximum_steering_angle_rad;
  config.mpc.max_steering_rate = mpc_maximum_steering_rate_deg_per_sec * kDegreesToRadians;
  config.stanley.maximum_steering_angle_rad = config.pure_pursuit.maximum_steering_angle_rad;
  config.stanley.maximum_steering_rate_rad_per_sec = stanley_maximum_steering_rate_deg_per_sec * kDegreesToRadians;
  config.hybrid.pure_pursuit = config.pure_pursuit;
  config.hybrid.stanley = config.stanley;
  config.hybrid.maximum_steering_angle_rad = config.pure_pursuit.maximum_steering_angle_rad;
  config.hybrid.maximum_steering_rate_rad_per_sec = hybrid_maximum_steering_rate_deg_per_sec * kDegreesToRadians;
  config.hybrid.low_curvature_maximum_steering_rate_rad_per_sec = hybrid_low_curvature_maximum_steering_rate_deg_per_sec *
      kDegreesToRadians;
  config.hybrid.full_steering_rate_curvature_m_inv = hybrid_full_steering_rate_curvature_m_inv;
  if (config.lateral_controller != "mpc" && config.lateral_controller != "hybrid" &&
      config.lateral_controller != "stanley" && config.lateral_controller != "pure_pursuit")
    throw std::invalid_argument("invalid lateral_controller");
  if (config.longitudinal_controller != "mpc" && config.longitudinal_controller != "pid")
    throw std::invalid_argument("invalid longitudinal_controller");
  requirePositive("nominal_dt_sec", config.nominal_dt_sec);
  if (maximum_steering_angle_deg <= 0 || maximum_steering_angle_deg >= 90 ||
      config.pid.maximum_accel > 1 || config.pid.maximum_brake > 1 ||
      config.longitudinal_mpc.maximum_speed_mps > 60.0 / 3.6)
    throw std::invalid_argument("invalid steering, pedal, or speed bound");
  config.wheel_corridor.wheelbase_m = config.pure_pursuit.wheelbase_m;
  requirePositive("minimum_control_dt_sec", config.minimum_dt_sec);
  requirePositive("maximum_control_dt_sec", config.maximum_dt_sec);
  requireNonNegative("speed_filter_time_constant_sec", config.speed_filter_time_constant_sec);
  requirePositive("lookahead_curvature_preview_distance_m", config.curvature_preview_distance_m);
  requirePositive("curvature_sample_spacing_m", config.curvature_sample_spacing_m);
  if (config.minimum_dt_sec > config.nominal_dt_sec || config.nominal_dt_sec > config.maximum_dt_sec ||
      config.pure_pursuit.lookahead_min_m > config.pure_pursuit.lookahead_max_m ||
      std::abs(config.hybrid.imm.front_axle_to_cg_m + config.hybrid.imm.rear_axle_to_cg_m - config.pure_pursuit.wheelbase_m) > 1e-6 ||
      config.wheel_corridor.lane_half_width_m <= 0.5 * config.wheel_corridor.vehicle_width_m)
    throw std::invalid_argument("invalid timing, lookahead, or vehicle geometry");
  return config;
}
} }  // namespace control::tracking
