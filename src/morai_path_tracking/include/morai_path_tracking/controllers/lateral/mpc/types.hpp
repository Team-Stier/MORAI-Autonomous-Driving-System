#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <cstdint>
#include <vector>

namespace morai_mpc {

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }
inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double normalizeAngle(double a) {
  if (!std::isfinite(a)) return std::numeric_limits<double>::quiet_NaN();
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}
inline bool finite(double v) { return std::isfinite(v); }
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

struct Point2d { double x{0.0}, y{0.0}; };
struct Pose2d { double x{0.0}, y{0.0}, yaw{0.0}; };
struct ReferencePoint { double x{0.0}, y{0.0}, s{0.0}, heading{0.0}, curvature{0.0}; };
struct VehicleState {
  double x{0.0}, y{0.0}, yaw{0.0}, speed{0.0}, steering{0.0};
  double lateral_velocity{0.0}, yaw_rate{0.0};
};
struct MpcState {
  double lateral_error{0.0}, heading_error{0.0}, steering{0.0};
  double sideslip{0.0}, yaw_rate{0.0};
};

struct MpcCommand {
  double steering_rate{0.0};
  double raw_steering_rate{0.0};
  bool success{false};
  int iterations{0};
  double solve_time_ms{0.0};
  double cost{std::numeric_limits<double>::infinity()};
  std::string message;
};

struct MpcConfig {
  std::string solver{"projected_gradient"};
  double wheelbase{3.0}, speed{5.0}, dt{0.05}, prediction_time{1.5};
  int horizon{30}, optimizer_iterations{4};
  double max_steering{deg2rad(40.0)}, max_steering_rate{deg2rad(60.0)};
  double resample_ds{0.5};
  double w_lateral{10.0}, w_heading{6.0}, w_steering{0.5};
  double w_rate{0.15}, w_rate_change{0.8};
  double w_terminal_lateral{25.0}, w_terminal_heading{15.0};
  double adaptation_start_curvature{0.003}, adaptation_full_curvature{0.020};
  double curve_heading_weight_multiplier{1.8};
  double curve_steering_weight_multiplier{4.0};
  double curve_rate_change_weight_multiplier{0.35};
  double straight_rate_weight_multiplier{2.0};
  double straight_rate_change_weight_multiplier{1.5};
  double yaw_rate_damping_gain{0.35};
  double yaw_rate_damping_fade_curvature{0.020};
  bool swept_path_compensation_enabled{false};
  double swept_path_vehicle_width{1.892};
  double swept_path_gain{1.0};
  double swept_path_activation_start_curvature{0.005};
  double swept_path_activation_full_curvature{0.020};
  double swept_path_maximum_offset{0.30};
  int swept_path_curvature_smoothing_window{5};
  double straight_cte_filter_time_constant{0.0};
  double straight_cte_filter_minimum_speed{0.0};
  double straight_cte_filter_maximum_curvature{0.003};
  double straight_cte_maximum_rate{0.0};
  double straight_steering_rate_filter_time_constant{0.0};
  double straight_steering_rate_filter_minimum_speed{0.0};
  double straight_steering_rate_filter_maximum_curvature{0.003};
  bool straight_spatial_fit_enabled{false};
  double straight_spatial_fit_start_speed{0.0};
  double straight_spatial_fit_full_speed{0.0};
  double straight_spatial_fit_full_curvature{0.001};
  double straight_spatial_fit_off_curvature{0.006};
  double straight_spatial_fit_base_preview{4.0};
  double straight_spatial_fit_speed_gain{0.25};
  double straight_spatial_fit_maximum_preview{10.0};
  double straight_spatial_fit_lateral_weight_multiplier{0.65};
  double curve_feedforward_seed_gain{0.0};
  double curve_feedforward_seed_lookahead{0.0};
  double straight_yaw_rate_steering_estimate_gain{0.0};
  double straight_yaw_rate_steering_estimate_minimum_speed{0.0};
  double straight_yaw_rate_steering_estimate_filter_time_constant{0.0};
  double curve_yaw_rate_steering_estimate_gain{0.0};
  double curve_yaw_rate_steering_estimate_minimum_speed{0.0};
  double curve_yaw_rate_steering_estimate_activation_start_curvature{0.015};
  double curve_yaw_rate_steering_estimate_activation_full_curvature{0.040};
  double curve_yaw_rate_steering_estimate_blend_time_constant{0.0};
  bool straight_dynamic_model_enabled{false};
  double straight_dynamic_model_minimum_speed{0.0};
  double straight_dynamic_model_maximum_curvature{0.003};
  double dynamic_mass{2000.0}, dynamic_yaw_inertia{4000.0};
  double dynamic_front_cornering_stiffness{60000.0};
  double dynamic_rear_cornering_stiffness{60000.0};
  double dynamic_front_axle_to_cg{1.5}, dynamic_rear_axle_to_cg{1.5};
  double straight_dynamic_sideslip_weight{1.0};
  double straight_dynamic_yaw_rate_weight{4.0};
  double optimizer_initial_step{deg2rad(30.0)};
  double optimizer_min_step{deg2rad(0.5)}, optimizer_step_decay{0.5};
  double optimizer_gradient_epsilon{deg2rad(0.25)};
  double optimizer_gradient_step{0.05};
  double optimizer_gradient_tolerance{1.0e-5};
  int optimizer_line_search_steps{8};
  bool optimizer_use_warm_start{true}, optimizer_multi_resolution{false};
  bool use_previous_control_rate_change{false};
  bool optimizer_model_actuator{true};
  double actuator_delay{0.0}, actuator_time_constant{0.0};
  double noise_position_std{0.0}, noise_yaw_std{0.0}, noise_speed_std{0.0};
  std::uint32_t random_seed{42};
  double max_time{25.0};
  bool time_based_horizon{false}, adaptive_prediction_time{false};
  double minimum_prediction_time{1.0}, maximum_prediction_time{2.0};
  int minimum_horizon_steps{12}, maximum_horizon_steps{60};
  double minimum_preview_distance{0.0}, prediction_speed_gain{0.0};
};

inline int effectiveHorizon(const MpcConfig& c, double speed) {
  if(!c.time_based_horizon)return c.horizon;
  double target=std::max(c.minimum_prediction_time,c.prediction_time);
  if(c.adaptive_prediction_time)target=clamp(c.prediction_time+c.prediction_speed_gain*std::max(0.0,std::abs(speed)-5.0),c.minimum_prediction_time,c.maximum_prediction_time);
  if(c.minimum_preview_distance>0&&std::abs(speed)>0.1)target=std::max(target,c.minimum_preview_distance/std::abs(speed));
  target=std::min(target,c.maximum_prediction_time);
  return std::max(c.minimum_horizon_steps,std::min(c.maximum_horizon_steps,static_cast<int>(std::ceil(target/c.dt))));
}

struct SimulationSample {
  double time{0.0}; VehicleState vehicle; ReferencePoint reference;
  double lateral_error{0.0}, heading_error{0.0}, steering_rate_command{0.0};
  double delayed_steering_rate_command{0.0}, applied_steering_rate{0.0};
  double commanded_steering_angle{0.0};
  double path_progress{0.0};
  VehicleState measured_vehicle;
  int solver_iterations{0}; bool solver_success{false}; double solver_time_ms{0.0};
};

struct SimulationResult {
  std::string scenario; std::vector<SimulationSample> samples;
  double rms_lateral_error{0.0}, max_abs_lateral_error{0.0};
  double rms_heading_error{0.0}, max_abs_steering{0.0}, max_abs_steering_rate{0.0};
  double average_solver_time_ms{0.0}, max_solver_time_ms{0.0};
  int solver_failures{0}; double final_lateral_error{0.0}, progress{0.0}; bool completed{false};
};

inline bool valid(const Point2d& p) { return finite(p.x) && finite(p.y); }
inline bool valid(const VehicleState& s) {
  return finite(s.x) && finite(s.y) && finite(s.yaw) && finite(s.speed) &&
         finite(s.steering) && finite(s.lateral_velocity) &&
         finite(s.yaw_rate);
}
inline bool valid(const MpcConfig& c) {
  return (c.solver == "osqp" || c.solver == "projected_gradient" || c.solver == "coordinate_search") &&
    finite(c.wheelbase) && c.wheelbase > 0 && finite(c.speed) && c.speed >= 0 &&
    finite(c.dt) && c.dt > 0 && c.horizon > 1 && finite(c.max_steering) && c.max_steering > 0 &&
    finite(c.max_steering_rate) && c.max_steering_rate > 0 && c.optimizer_iterations > 0 &&
    finite(c.optimizer_initial_step) && c.optimizer_initial_step > 0 &&
    finite(c.optimizer_min_step) && c.optimizer_min_step > 0 &&
    finite(c.optimizer_step_decay) && c.optimizer_step_decay > 0 && c.optimizer_step_decay < 1 &&
    finite(c.optimizer_gradient_epsilon) && c.optimizer_gradient_epsilon > 0 &&
    finite(c.optimizer_gradient_step) && c.optimizer_gradient_step > 0 &&
    finite(c.optimizer_gradient_tolerance) && c.optimizer_gradient_tolerance >= 0 &&
    c.optimizer_line_search_steps > 0 &&
    finite(c.adaptation_start_curvature) && c.adaptation_start_curvature>=0 &&
    finite(c.adaptation_full_curvature) && c.adaptation_full_curvature>c.adaptation_start_curvature &&
    finite(c.curve_heading_weight_multiplier) && c.curve_heading_weight_multiplier>0 &&
    finite(c.curve_steering_weight_multiplier) && c.curve_steering_weight_multiplier>0 &&
    finite(c.curve_rate_change_weight_multiplier) && c.curve_rate_change_weight_multiplier>0 &&
    finite(c.straight_rate_weight_multiplier) && c.straight_rate_weight_multiplier>0 &&
    finite(c.straight_rate_change_weight_multiplier) && c.straight_rate_change_weight_multiplier>0 &&
    finite(c.yaw_rate_damping_gain) && c.yaw_rate_damping_gain>=0 &&
    finite(c.yaw_rate_damping_fade_curvature) && c.yaw_rate_damping_fade_curvature>0 &&
    finite(c.swept_path_vehicle_width) && c.swept_path_vehicle_width>0 &&
    finite(c.swept_path_gain) && c.swept_path_gain>=0 &&
    finite(c.swept_path_activation_start_curvature) && c.swept_path_activation_start_curvature>=0 &&
    finite(c.swept_path_activation_full_curvature) &&
    c.swept_path_activation_full_curvature>c.swept_path_activation_start_curvature &&
    finite(c.swept_path_maximum_offset) && c.swept_path_maximum_offset>=0 &&
    c.swept_path_curvature_smoothing_window>0 &&
    c.swept_path_curvature_smoothing_window%2==1 &&
    finite(c.straight_cte_filter_time_constant) && c.straight_cte_filter_time_constant>=0 &&
    finite(c.straight_cte_filter_minimum_speed) && c.straight_cte_filter_minimum_speed>=0 &&
    finite(c.straight_cte_filter_maximum_curvature) && c.straight_cte_filter_maximum_curvature>=0 &&
    finite(c.straight_cte_maximum_rate) && c.straight_cte_maximum_rate>=0 &&
    finite(c.straight_steering_rate_filter_time_constant) && c.straight_steering_rate_filter_time_constant>=0 &&
    finite(c.straight_steering_rate_filter_minimum_speed) && c.straight_steering_rate_filter_minimum_speed>=0 &&
    finite(c.straight_steering_rate_filter_maximum_curvature) && c.straight_steering_rate_filter_maximum_curvature>=0 &&
    finite(c.straight_spatial_fit_start_speed) && c.straight_spatial_fit_start_speed>=0 &&
    finite(c.straight_spatial_fit_full_speed) && c.straight_spatial_fit_full_speed>=c.straight_spatial_fit_start_speed &&
    finite(c.straight_spatial_fit_full_curvature) && c.straight_spatial_fit_full_curvature>=0 &&
    finite(c.straight_spatial_fit_off_curvature) && c.straight_spatial_fit_off_curvature>c.straight_spatial_fit_full_curvature &&
    finite(c.straight_spatial_fit_base_preview) && c.straight_spatial_fit_base_preview>0 &&
    finite(c.straight_spatial_fit_speed_gain) && c.straight_spatial_fit_speed_gain>=0 &&
    finite(c.straight_spatial_fit_maximum_preview) && c.straight_spatial_fit_maximum_preview>=c.straight_spatial_fit_base_preview &&
    finite(c.straight_spatial_fit_lateral_weight_multiplier) && c.straight_spatial_fit_lateral_weight_multiplier>0 && c.straight_spatial_fit_lateral_weight_multiplier<=1 &&
    finite(c.curve_feedforward_seed_gain) && c.curve_feedforward_seed_gain>=0 && c.curve_feedforward_seed_gain<=1 &&
    finite(c.curve_feedforward_seed_lookahead) && c.curve_feedforward_seed_lookahead>=0 &&
    finite(c.straight_yaw_rate_steering_estimate_gain) && c.straight_yaw_rate_steering_estimate_gain>=0 && c.straight_yaw_rate_steering_estimate_gain<=1 &&
    finite(c.straight_yaw_rate_steering_estimate_minimum_speed) && c.straight_yaw_rate_steering_estimate_minimum_speed>=0 &&
    finite(c.straight_yaw_rate_steering_estimate_filter_time_constant) && c.straight_yaw_rate_steering_estimate_filter_time_constant>=0 &&
    finite(c.curve_yaw_rate_steering_estimate_gain) && c.curve_yaw_rate_steering_estimate_gain>=0 && c.curve_yaw_rate_steering_estimate_gain<=1 &&
    finite(c.curve_yaw_rate_steering_estimate_minimum_speed) && c.curve_yaw_rate_steering_estimate_minimum_speed>=0 &&
    finite(c.curve_yaw_rate_steering_estimate_activation_start_curvature) && c.curve_yaw_rate_steering_estimate_activation_start_curvature>=0 &&
    finite(c.curve_yaw_rate_steering_estimate_activation_full_curvature) && c.curve_yaw_rate_steering_estimate_activation_full_curvature>c.curve_yaw_rate_steering_estimate_activation_start_curvature &&
    finite(c.curve_yaw_rate_steering_estimate_blend_time_constant) && c.curve_yaw_rate_steering_estimate_blend_time_constant>=0 &&
    finite(c.straight_dynamic_model_minimum_speed) && c.straight_dynamic_model_minimum_speed>=0 &&
    finite(c.straight_dynamic_model_maximum_curvature) && c.straight_dynamic_model_maximum_curvature>=0 &&
    finite(c.dynamic_mass) && c.dynamic_mass>0 &&
    finite(c.dynamic_yaw_inertia) && c.dynamic_yaw_inertia>0 &&
    finite(c.dynamic_front_cornering_stiffness) && c.dynamic_front_cornering_stiffness>0 &&
    finite(c.dynamic_rear_cornering_stiffness) && c.dynamic_rear_cornering_stiffness>0 &&
    finite(c.dynamic_front_axle_to_cg) && c.dynamic_front_axle_to_cg>0 &&
    finite(c.dynamic_rear_axle_to_cg) && c.dynamic_rear_axle_to_cg>0 &&
    finite(c.straight_dynamic_sideslip_weight) && c.straight_dynamic_sideslip_weight>=0 &&
    finite(c.straight_dynamic_yaw_rate_weight) && c.straight_dynamic_yaw_rate_weight>=0 &&
    finite(c.actuator_delay) && c.actuator_delay >= 0 && finite(c.actuator_time_constant) && c.actuator_time_constant >= 0 &&
    finite(c.noise_position_std) && c.noise_position_std >= 0 && finite(c.noise_yaw_std) && c.noise_yaw_std >= 0 &&
    finite(c.noise_speed_std) && c.noise_speed_std >= 0 && finite(c.minimum_prediction_time) && c.minimum_prediction_time>0 &&
    finite(c.maximum_prediction_time) && c.maximum_prediction_time>=c.minimum_prediction_time && c.minimum_horizon_steps>1 &&
    c.maximum_horizon_steps>=c.minimum_horizon_steps && finite(c.minimum_preview_distance) && c.minimum_preview_distance>=0 &&
    finite(c.prediction_speed_gain) && c.prediction_speed_gain>=0;
}

}  // namespace morai_mpc
