#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <boost/bind/bind.hpp>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/names.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include "morai_path_tracking/common/control_timing_bounds.hpp"
#include "morai_path_tracking/planning/curvature_speed_planner.hpp"
#include "morai_path_tracking/planning/wheel_corridor.hpp"
#include "morai_path_tracking/controllers/lateral/hybrid_controller.hpp"
#include "morai_path_tracking/controllers/lateral/mpc/mpc_lateral_controller.hpp"
#include "morai_path_tracking/controllers/longitudinal/pid_controller.hpp"
#include "morai_path_tracking/controllers/longitudinal/mpc/longitudinal_mpc.hpp"
#include "morai_path_tracking/controllers/lateral/pure_pursuit.hpp"
#include "morai_path_tracking/controllers/lateral/stanley_controller.hpp"
#include "morai_path_tracking/ControllerStatus.h"

namespace morai_path_tracking {
namespace {

using XmlValue = XmlRpc::XmlRpcValue;

constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kKilometresPerHourToMetresPerSecond = 1.0 / 3.6;
constexpr char kPurePursuitController[] = "pure_pursuit";
constexpr char kStanleyController[] = "stanley";
constexpr char kHybridController[] = "hybrid";
constexpr char kMpcController[] = "mpc";
constexpr char kPidController[] = "pid";

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
    return static_cast<double>(value);
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

void requireRosName(const char* name, const std::string& value) {
  std::string error;
  if (value.empty() || !ros::names::validate(value, error)) {
    throw std::invalid_argument(std::string(name) +
                                " must be a valid ROS name: " + error);
  }
}

ros::WallDuration periodFromRate(double control_rate_hz) {
  requirePositive("control_rate_hz", control_rate_hz);
  const double period_sec = 1.0 / control_rate_hz;
  if (!std::isfinite(period_sec) || period_sec <= 0.0 ||
      period_sec > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    throw std::invalid_argument(
        "control_rate_hz produces an unrepresentable WallTimer period");
  }
  const ros::WallDuration period(period_sec);
  if (period.toNSec() <= 0) {
    throw std::invalid_argument(
        "control_rate_hz produces a non-positive WallTimer period");
  }
  return period;
}

struct ControllerConfig {
  std::string local_path_topic;
  std::string ego_topic;
  std::string vehicle_status_topic;
  std::string command_topic;
  std::string controller_status_topic;
  std::string lookahead_point_topic;
  std::string stanley_projection_point_topic;
  std::string mpc_projection_point_topic;
  std::string mpc_compensated_path_topic;
  std::string expected_frame_id;
  std::string expected_velocity_frame_id;
  ros::WallDuration control_period;
  double path_timeout_sec{0.25};
  double ego_timeout_sec{0.25};
  double vehicle_status_timeout_sec{0.25};
  double maximum_input_skew_sec{0.0};
  int input_sync_queue_size{10};
  ControlTimingBounds control_timing_bounds{0.005, 0.10};
  double safe_brake_command{0.50};
  double speed_filter_time_constant_sec{0.0};
  std::string lateral_controller;
  std::string longitudinal_controller;
  PurePursuitConfig pure_pursuit;
  StanleyConfig stanley;
  HybridConfig hybrid;
  morai_mpc::MpcConfig mpc;
  double mpc_test_speed_limit_mps{5.0};
  CurvatureSpeedPlannerConfig curvature_speed_planner;
  WheelCorridorConfig wheel_corridor;
  LaneClearanceSpeedConfig lane_clearance_speed;
  HeadingErrorSpeedConfig heading_error_speed;
  PidConfig pid;
  LongitudinalMpcConfig longitudinal_mpc;
  bool longitudinal_mpc_fallback_to_pid{true};
};

struct LateralControlOutput {
  bool valid{false};
  bool has_tracking_target{false};
  bool has_stanley_projection{false};
  Point2d target;
  double lookahead_m{0.0};
  double steering_angle_rad{0.0};
  double cross_track_error_m{0.0};
  double mpc_raw_cross_track_error_m{0.0};
  double heading_error_rad{0.0};
  double reference_curvature_m_inv{0.0};
  double reference_yaw_rate_radps{0.0};
  double yaw_rate_error_radps{0.0};
  double curvature_feedforward_steering_rad{0.0};
  double heading_feedback_steering_rad{0.0};
  double cross_track_feedback_steering_rad{0.0};
  double applied_yaw_rate_damping_gain_sec{0.0};
  double yaw_rate_damping_steering_rad{0.0};
  double requested_steering_angle_rad{0.0};
  double pure_pursuit_steering_angle_rad{0.0};
  double hybrid_corrected_pure_pursuit_steering_angle_rad{0.0};
  double stanley_steering_angle_rad{0.0};
  double hybrid_pure_pursuit_probability{0.0};
  double hybrid_stanley_probability{0.0};
  double hybrid_effective_pure_pursuit_weight{0.0};
  double hybrid_effective_stanley_weight{0.0};
  bool hybrid_candidate_conflict_guard_active{false};
  bool hybrid_candidate_conflict_stanley_override_active{false};
  bool hybrid_cross_track_recovery_active{false};
  double hybrid_cross_track_recovery_weight{0.0};
  bool hybrid_cross_track_recovery_heading_suppression_active{false};
  double hybrid_cross_track_recovery_heading_suppression_weight{0.0};
  bool hybrid_lane_clearance_recovery_active{false};
  double lane_clearance_recovery_urgency{0.0};
  bool hybrid_curve_preview_stanley_recovery_active{false};
  double hybrid_curve_preview_stanley_recovery_weight{0.0};
  bool hybrid_heading_lag_stanley_recovery_active{false};
  double hybrid_heading_lag_stanley_recovery_weight{0.0};
  double hybrid_applied_maximum_steering_rate_rad_per_sec{0.0};
  double measured_sideslip_angle_rad{0.0};
  double pure_pursuit_innovation_norm{0.0};
  double stanley_innovation_norm{0.0};
  Point2d stanley_projection;
  bool mpc_solver_success{false};
  int mpc_solver_iterations{0};
  double mpc_solver_time_ms{0.0};
  double mpc_solver_cost{0.0};
  double mpc_steering_rate_rad_per_sec{0.0};
  double mpc_raw_steering_rate_rad_per_sec{0.0};
  double mpc_yaw_rate_steering_estimate_blend{0.0};
  double mpc_modeled_steering_angle_rad{0.0};
  Point2d mpc_projection;
  std::vector<morai_mpc::Point2d> mpc_compensated_path;
  std::string error;
};

ControllerConfig loadConfig(const ros::NodeHandle& private_node) {
  ControllerConfig config;
  config.local_path_topic = requiredString(private_node, "local_path_topic");
  config.ego_topic = requiredString(private_node, "ego_topic");
  config.vehicle_status_topic =
      requiredString(private_node, "vehicle_status_topic");
  config.command_topic = requiredString(private_node, "command_topic");
  config.controller_status_topic =
      requiredString(private_node, "controller_status_topic");
  config.lookahead_point_topic =
      requiredString(private_node, "lookahead_point_topic");
  config.stanley_projection_point_topic =
      requiredString(private_node, "stanley_projection_point_topic");
  private_node.param<std::string>("mpc_projection_point_topic",
                                  config.mpc_projection_point_topic,
                                  "/control/mpc_projection_point");
  private_node.param<std::string>("mpc_compensated_path_topic",
                                  config.mpc_compensated_path_topic,
                                  "/control/mpc_compensated_path");
  config.expected_frame_id = requiredString(private_node, "expected_frame_id");
  config.expected_velocity_frame_id =
      requiredString(private_node, "expected_velocity_frame_id");
  const double control_rate_hz =
      requiredDouble(private_node, "control_rate_hz");
  config.path_timeout_sec = requiredDouble(private_node, "path_timeout_sec");
  config.ego_timeout_sec =
      requiredDouble(private_node, "ego_timeout_sec");
  config.vehicle_status_timeout_sec =
      requiredDouble(private_node, "vehicle_status_timeout_sec");
  config.maximum_input_skew_sec =
      requiredDouble(private_node, "maximum_input_skew_sec");
  config.input_sync_queue_size =
      requiredInt(private_node, "input_sync_queue_size");
  const double minimum_control_dt_sec =
      requiredDouble(private_node, "minimum_control_dt_sec");
  const double maximum_control_dt_sec =
      requiredDouble(private_node, "maximum_control_dt_sec");
  config.safe_brake_command =
      requiredDouble(private_node, "safe_brake_command");
  config.lateral_controller =
      requiredString(private_node, "lateral_controller");
  // Legacy diagnostic launch files predate longitudinal MPC. Keep their
  // behavior as PID unless the runtime YAML explicitly selects MPC.
  private_node.param<std::string>("longitudinal_controller",
                                  config.longitudinal_controller,
                                  kPidController);
  config.pure_pursuit.wheelbase_m =
      requiredDouble(private_node, "wheelbase_m");
  config.mpc.wheelbase = config.pure_pursuit.wheelbase_m;
  config.stanley.wheelbase_m = config.pure_pursuit.wheelbase_m;
  config.wheel_corridor.wheelbase_m = config.pure_pursuit.wheelbase_m;
  config.wheel_corridor.vehicle_width_m =
      requiredDouble(private_node, "vehicle_width_m");
  config.wheel_corridor.lane_half_width_m =
      requiredDouble(private_node, "lane_half_width_m");
  const double lane_clearance_recovery_start_m =
      requiredDouble(private_node, "lane_clearance_recovery_start_m");
  const double lane_clearance_recovery_full_m =
      requiredDouble(private_node, "lane_clearance_recovery_full_m");
  const double lane_clearance_recovery_speed_kph =
      requiredDouble(private_node, "lane_clearance_recovery_speed_kph");
  config.hybrid.lane_clearance_recovery_start_m =
      lane_clearance_recovery_start_m;
  config.hybrid.lane_clearance_recovery_full_m =
      lane_clearance_recovery_full_m;
  config.lane_clearance_speed.recovery_start_m =
      lane_clearance_recovery_start_m;
  config.lane_clearance_speed.recovery_full_m =
      lane_clearance_recovery_full_m;
  config.lane_clearance_speed.minimum_speed_mps =
      lane_clearance_recovery_speed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.heading_error_speed.recovery_start_rad =
      requiredDouble(private_node, "heading_error_speed_limit_start_deg") *
      kDegreesToRadians;
  config.heading_error_speed.recovery_full_rad =
      requiredDouble(private_node, "heading_error_speed_limit_full_deg") *
      kDegreesToRadians;
  config.heading_error_speed.minimum_speed_mps =
      requiredDouble(private_node, "heading_error_recovery_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.pure_pursuit.lookahead_base_m =
      requiredDouble(private_node, "lookahead_base_m");
  config.pure_pursuit.lookahead_speed_gain_sec =
      requiredDouble(private_node, "lookahead_speed_gain_sec");
  config.pure_pursuit.lookahead_curvature_gain_m =
      requiredDouble(private_node, "lookahead_curvature_gain_m");
  config.pure_pursuit.lookahead_min_m =
      requiredDouble(private_node, "lookahead_min_m");
  config.pure_pursuit.lookahead_max_m =
      requiredDouble(private_node, "lookahead_max_m");
  config.pure_pursuit.minimum_target_distance_m =
      requiredDouble(private_node, "minimum_target_distance_m");
  const double maximum_steering_angle_deg =
      requiredDouble(private_node, "maximum_steering_angle_deg");
  double mpc_maximum_steering_rate_deg_per_sec = 60.0;
  double mpc_test_speed_limit_kph = 18.0;
  if (config.lateral_controller == kMpcController) {
    config.mpc.solver = requiredString(private_node, "mpc_solver");
    config.mpc.prediction_time =
      requiredDouble(private_node, "mpc_prediction_time_sec");
  config.mpc.horizon = requiredInt(private_node, "mpc_horizon_steps");
  config.mpc.optimizer_iterations =
      requiredInt(private_node, "mpc_optimizer_iterations");
  config.mpc.resample_ds =
      requiredDouble(private_node, "mpc_resample_distance_m");
  config.mpc.w_lateral = requiredDouble(private_node, "mpc_weight_lateral");
  config.mpc.w_heading = requiredDouble(private_node, "mpc_weight_heading");
  config.mpc.w_steering =
      requiredDouble(private_node, "mpc_weight_steering");
  config.mpc.w_rate =
      requiredDouble(private_node, "mpc_weight_steering_rate");
  config.mpc.w_rate_change =
      requiredDouble(private_node, "mpc_weight_steering_rate_change");
  config.mpc.w_terminal_lateral =
      requiredDouble(private_node, "mpc_weight_terminal_lateral");
  config.mpc.w_terminal_heading =
      requiredDouble(private_node, "mpc_weight_terminal_heading");
  config.mpc.adaptation_start_curvature =
      requiredDouble(private_node, "mpc_adaptation_start_curvature_m_inv");
  config.mpc.adaptation_full_curvature =
      requiredDouble(private_node, "mpc_adaptation_full_curvature_m_inv");
  config.mpc.curve_heading_weight_multiplier =
      requiredDouble(private_node, "mpc_curve_heading_weight_multiplier");
  config.mpc.curve_steering_weight_multiplier =
      requiredDouble(private_node, "mpc_curve_steering_weight_multiplier");
  config.mpc.curve_rate_change_weight_multiplier = requiredDouble(
      private_node, "mpc_curve_rate_change_weight_multiplier");
  config.mpc.straight_rate_weight_multiplier =
      requiredDouble(private_node, "mpc_straight_rate_weight_multiplier");
  config.mpc.straight_rate_change_weight_multiplier = requiredDouble(
      private_node, "mpc_straight_rate_change_weight_multiplier");
  config.mpc.yaw_rate_damping_gain =
      requiredDouble(private_node, "mpc_yaw_rate_damping_gain");
  config.mpc.yaw_rate_damping_fade_curvature = requiredDouble(
      private_node, "mpc_yaw_rate_damping_fade_curvature_m_inv");
  config.mpc.swept_path_compensation_enabled =
      requiredBool(private_node, "mpc_swept_path_compensation_enabled");
  config.mpc.swept_path_vehicle_width =
      requiredDouble(private_node, "mpc_swept_path_vehicle_width_m");
  config.mpc.swept_path_gain =
      requiredDouble(private_node, "mpc_swept_path_compensation_gain");
  config.mpc.swept_path_activation_start_curvature = requiredDouble(
      private_node, "mpc_swept_path_activation_start_curvature_m_inv");
  config.mpc.swept_path_activation_full_curvature = requiredDouble(
      private_node, "mpc_swept_path_activation_full_curvature_m_inv");
  config.mpc.swept_path_maximum_offset =
      requiredDouble(private_node, "mpc_swept_path_maximum_offset_m");
  config.mpc.swept_path_curvature_smoothing_window =
      requiredInt(private_node, "mpc_swept_path_curvature_smoothing_window_points");
  config.mpc.straight_cte_filter_time_constant =
      requiredDouble(private_node, "mpc_straight_cte_filter_time_constant_sec");
  config.mpc.straight_cte_filter_minimum_speed =
      requiredDouble(private_node, "mpc_straight_cte_filter_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_cte_filter_maximum_curvature = requiredDouble(
      private_node, "mpc_straight_cte_filter_maximum_curvature_m_inv");
  config.mpc.straight_cte_maximum_rate = requiredDouble(
      private_node, "mpc_straight_cte_maximum_rate_mps");
  config.mpc.straight_steering_rate_filter_time_constant = requiredDouble(
      private_node, "mpc_straight_steering_rate_filter_time_constant_sec");
  config.mpc.straight_steering_rate_filter_minimum_speed = requiredDouble(
      private_node, "mpc_straight_steering_rate_filter_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_steering_rate_filter_maximum_curvature = requiredDouble(
      private_node,
      "mpc_straight_steering_rate_filter_maximum_curvature_m_inv");
  config.mpc.straight_spatial_fit_enabled =
      requiredBool(private_node, "mpc_straight_spatial_fit_enabled");
  config.mpc.straight_spatial_fit_start_speed = requiredDouble(
      private_node, "mpc_straight_spatial_fit_start_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_spatial_fit_full_speed = requiredDouble(
      private_node, "mpc_straight_spatial_fit_full_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_spatial_fit_full_curvature = requiredDouble(
      private_node, "mpc_straight_spatial_fit_full_curvature_m_inv");
  config.mpc.straight_spatial_fit_off_curvature = requiredDouble(
      private_node, "mpc_straight_spatial_fit_off_curvature_m_inv");
  config.mpc.straight_spatial_fit_base_preview = requiredDouble(
      private_node, "mpc_straight_spatial_fit_base_preview_m");
  config.mpc.straight_spatial_fit_speed_gain = requiredDouble(
      private_node, "mpc_straight_spatial_fit_speed_gain_sec");
  config.mpc.straight_spatial_fit_maximum_preview = requiredDouble(
      private_node, "mpc_straight_spatial_fit_maximum_preview_m");
  config.mpc.straight_spatial_fit_lateral_weight_multiplier = requiredDouble(
      private_node,
      "mpc_straight_spatial_fit_lateral_weight_multiplier");
  config.mpc.curve_feedforward_seed_gain =
      requiredDouble(private_node, "mpc_curve_feedforward_seed_gain");
  config.mpc.curve_feedforward_seed_lookahead =
      requiredDouble(private_node, "mpc_curve_feedforward_seed_lookahead_m");
  config.mpc.straight_yaw_rate_steering_estimate_gain = requiredDouble(
      private_node, "mpc_straight_yaw_rate_steering_estimate_gain");
  config.mpc.straight_yaw_rate_steering_estimate_minimum_speed =
      requiredDouble(
          private_node,
          "mpc_straight_yaw_rate_steering_estimate_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_yaw_rate_steering_estimate_filter_time_constant =
      requiredDouble(
          private_node,
          "mpc_straight_yaw_rate_steering_estimate_filter_time_constant_sec");
  config.mpc.curve_yaw_rate_steering_estimate_gain = requiredDouble(
      private_node, "mpc_curve_yaw_rate_steering_estimate_gain");
  config.mpc.curve_yaw_rate_steering_estimate_minimum_speed =
      requiredDouble(
          private_node,
          "mpc_curve_yaw_rate_steering_estimate_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.curve_yaw_rate_steering_estimate_activation_start_curvature =
      requiredDouble(
          private_node,
          "mpc_curve_yaw_rate_steering_estimate_activation_start_curvature_m_inv");
  config.mpc.curve_yaw_rate_steering_estimate_activation_full_curvature =
      requiredDouble(
          private_node,
          "mpc_curve_yaw_rate_steering_estimate_activation_full_curvature_m_inv");
  config.mpc.curve_yaw_rate_steering_estimate_blend_time_constant =
      requiredDouble(
          private_node,
          "mpc_curve_yaw_rate_steering_estimate_blend_time_constant_sec");
  config.mpc.straight_dynamic_model_enabled =
      requiredBool(private_node, "mpc_straight_dynamic_model_enabled");
  config.mpc.straight_dynamic_model_minimum_speed =
      requiredDouble(private_node,
                     "mpc_straight_dynamic_model_minimum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  config.mpc.straight_dynamic_model_maximum_curvature = requiredDouble(
      private_node, "mpc_straight_dynamic_model_maximum_curvature_m_inv");
  config.mpc.straight_dynamic_sideslip_weight = requiredDouble(
      private_node, "mpc_weight_straight_dynamic_sideslip");
  config.mpc.straight_dynamic_yaw_rate_weight = requiredDouble(
      private_node, "mpc_weight_straight_dynamic_yaw_rate");
  config.mpc.optimizer_initial_step =
      requiredDouble(private_node, "mpc_optimizer_initial_step_deg_per_sec") *
      kDegreesToRadians;
  config.mpc.optimizer_min_step =
      requiredDouble(private_node, "mpc_optimizer_min_step_deg_per_sec") *
      kDegreesToRadians;
  config.mpc.optimizer_step_decay =
      requiredDouble(private_node, "mpc_optimizer_step_decay");
  config.mpc.optimizer_gradient_epsilon =
      requiredDouble(private_node, "mpc_optimizer_gradient_epsilon_deg_per_sec") *
      kDegreesToRadians;
  config.mpc.optimizer_gradient_step =
      requiredDouble(private_node, "mpc_optimizer_gradient_step");
  config.mpc.optimizer_gradient_tolerance =
      requiredDouble(private_node, "mpc_optimizer_gradient_tolerance");
  config.mpc.optimizer_line_search_steps =
      requiredInt(private_node, "mpc_optimizer_line_search_steps");
  config.mpc.optimizer_use_warm_start =
      requiredBool(private_node, "mpc_use_warm_start");
  config.mpc.optimizer_multi_resolution =
      requiredBool(private_node, "mpc_use_multi_resolution");
  config.mpc.use_previous_control_rate_change =
      requiredBool(private_node, "mpc_use_previous_control");
  config.mpc.optimizer_model_actuator =
      requiredBool(private_node, "mpc_model_actuator");
  config.mpc.actuator_delay =
      requiredDouble(private_node, "mpc_actuator_delay_sec");
  config.mpc.actuator_time_constant =
      requiredDouble(private_node, "mpc_actuator_time_constant_sec");
  config.mpc.time_based_horizon =
      requiredBool(private_node, "mpc_time_based_horizon");
  config.mpc.adaptive_prediction_time =
      requiredBool(private_node, "mpc_adaptive_prediction_time");
  config.mpc.minimum_prediction_time =
      requiredDouble(private_node, "mpc_minimum_prediction_time_sec");
  config.mpc.maximum_prediction_time =
      requiredDouble(private_node, "mpc_maximum_prediction_time_sec");
  config.mpc.minimum_horizon_steps =
      requiredInt(private_node, "mpc_minimum_horizon_steps");
  config.mpc.maximum_horizon_steps =
      requiredInt(private_node, "mpc_maximum_horizon_steps");
  config.mpc.minimum_preview_distance =
      requiredDouble(private_node, "mpc_minimum_preview_distance_m");
  config.mpc.prediction_speed_gain =
      requiredDouble(private_node, "mpc_prediction_speed_gain_sec_per_mps");
    mpc_maximum_steering_rate_deg_per_sec = requiredDouble(
      private_node, "mpc_maximum_steering_rate_deg_per_sec");
    mpc_test_speed_limit_kph =
      requiredDouble(private_node, "mpc_test_speed_limit_kph");
  }
  config.stanley.gain = requiredDouble(private_node, "stanley_gain");
  config.stanley.softening_speed_mps =
      requiredDouble(private_node, "stanley_softening_speed_mps");
  config.stanley.minimum_control_speed_mps =
      requiredDouble(private_node, "stanley_minimum_control_speed_mps");
  config.stanley.heading_window_m =
      requiredDouble(private_node, "stanley_heading_window_m");
  config.stanley.heading_error_gain =
      requiredDouble(private_node, "stanley_heading_error_gain");
  config.stanley.curvature_feedforward_gain =
      requiredDouble(private_node, "stanley_curvature_feedforward_gain");
  config.stanley.curvature_preview_distance_m =
      requiredDouble(private_node, "stanley_curvature_preview_distance_m");
  config.stanley.yaw_rate_damping_gain_sec =
      requiredDouble(private_node, "stanley_yaw_rate_damping_gain_sec");
  config.stanley.yaw_rate_damping_nonlinear_gain_sec2 = requiredDouble(
      private_node, "stanley_yaw_rate_damping_nonlinear_gain_sec2");
  const double stanley_maximum_steering_rate_deg_per_sec = requiredDouble(
      private_node, "stanley_maximum_steering_rate_deg_per_sec");
  config.hybrid.imm.mass_kg =
      requiredDouble(private_node, "hybrid_mass_kg");
  config.hybrid.imm.yaw_inertia_kgm2 =
      requiredDouble(private_node, "hybrid_yaw_inertia_kgm2");
  config.hybrid.imm.front_cornering_stiffness_n_per_rad = requiredDouble(
      private_node, "hybrid_front_cornering_stiffness_n_per_rad");
  config.hybrid.imm.rear_cornering_stiffness_n_per_rad = requiredDouble(
      private_node, "hybrid_rear_cornering_stiffness_n_per_rad");
  config.hybrid.imm.front_axle_to_cg_m =
      requiredDouble(private_node, "hybrid_front_axle_to_cg_m");
  config.hybrid.imm.rear_axle_to_cg_m =
      requiredDouble(private_node, "hybrid_rear_axle_to_cg_m");
  config.mpc.dynamic_mass = config.hybrid.imm.mass_kg;
  config.mpc.dynamic_yaw_inertia = config.hybrid.imm.yaw_inertia_kgm2;
  config.mpc.dynamic_front_cornering_stiffness =
      config.hybrid.imm.front_cornering_stiffness_n_per_rad;
  config.mpc.dynamic_rear_cornering_stiffness =
      config.hybrid.imm.rear_cornering_stiffness_n_per_rad;
  config.mpc.dynamic_front_axle_to_cg =
      config.hybrid.imm.front_axle_to_cg_m;
  config.mpc.dynamic_rear_axle_to_cg =
      config.hybrid.imm.rear_axle_to_cg_m;
  config.hybrid.imm.process_noise_sideslip =
      requiredDouble(private_node, "hybrid_process_noise_sideslip");
  config.hybrid.imm.process_noise_yaw_rate =
      requiredDouble(private_node, "hybrid_process_noise_yaw_rate");
  config.hybrid.imm.measurement_noise_sideslip =
      requiredDouble(private_node, "hybrid_measurement_noise_sideslip");
  config.hybrid.imm.measurement_noise_yaw_rate =
      requiredDouble(private_node, "hybrid_measurement_noise_yaw_rate");
  config.hybrid.imm.initial_covariance_sideslip =
      requiredDouble(private_node, "hybrid_initial_covariance_sideslip");
  config.hybrid.imm.initial_covariance_yaw_rate =
      requiredDouble(private_node, "hybrid_initial_covariance_yaw_rate");
  config.hybrid.imm.initial_pure_pursuit_probability = requiredDouble(
      private_node, "hybrid_initial_pure_pursuit_probability");
  config.hybrid.imm.initial_stanley_probability =
      requiredDouble(private_node, "hybrid_initial_stanley_probability");
  config.hybrid.imm.stanley_probability_min =
      requiredDouble(private_node, "hybrid_stanley_probability_min");
  config.hybrid.imm.stanley_probability_max =
      requiredDouble(private_node, "hybrid_stanley_probability_max");
  config.hybrid.imm.transition_pure_pursuit_to_pure_pursuit =
      requiredDouble(
          private_node,
          "hybrid_transition_pure_pursuit_to_pure_pursuit");
  config.hybrid.imm.transition_pure_pursuit_to_stanley =
      requiredDouble(private_node,
                     "hybrid_transition_pure_pursuit_to_stanley");
  config.hybrid.imm.transition_stanley_to_pure_pursuit =
      requiredDouble(private_node,
                     "hybrid_transition_stanley_to_pure_pursuit");
  config.hybrid.imm.transition_stanley_to_stanley =
      requiredDouble(private_node,
                     "hybrid_transition_stanley_to_stanley");
  config.hybrid.imm.transition_speed_gain =
      requiredDouble(private_node, "hybrid_transition_speed_gain");
  const double hybrid_transition_reference_speed_kph = requiredDouble(
      private_node, "hybrid_transition_reference_speed_kph");
  config.hybrid.imm.transition_reference_speed_mps =
      hybrid_transition_reference_speed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.hybrid.imm.minimum_model_speed_mps =
      requiredDouble(private_node, "hybrid_minimum_model_speed_mps");
  config.hybrid.pure_pursuit_cross_track_correction_gain =
      requiredDouble(
          private_node,
          "hybrid_pure_pursuit_cross_track_correction_gain");
  config.hybrid.curve_preview_stanley_weight_start_m_inv =
      requiredDouble(
          private_node,
          "hybrid_curve_preview_stanley_weight_start_m_inv");
  config.hybrid.curve_preview_stanley_weight_full_m_inv =
      requiredDouble(
          private_node,
          "hybrid_curve_preview_stanley_weight_full_m_inv");
  config.hybrid.curve_preview_stanley_minimum_weight =
      requiredDouble(
          private_node,
          "hybrid_curve_preview_stanley_minimum_weight");
  config.hybrid.heading_lag_stanley_weight_start_rad =
      requiredDouble(
          private_node,
          "hybrid_heading_lag_stanley_weight_start_deg") *
      kDegreesToRadians;
  config.hybrid.heading_lag_stanley_weight_full_rad =
      requiredDouble(
          private_node,
          "hybrid_heading_lag_stanley_weight_full_deg") *
      kDegreesToRadians;
  config.hybrid.heading_lag_stanley_minimum_weight =
      requiredDouble(
          private_node,
          "hybrid_heading_lag_stanley_minimum_weight");
  config.hybrid.candidate_conflict_curvature_threshold_m_inv =
      requiredDouble(
          private_node,
          "hybrid_candidate_conflict_curvature_threshold_m_inv");
  config.hybrid.candidate_conflict_cross_track_threshold_m =
      requiredDouble(
          private_node,
          "hybrid_candidate_conflict_cross_track_threshold_m");
  config.hybrid.cross_track_recovery_full_scale_m =
      requiredDouble(
          private_node,
          "hybrid_cross_track_recovery_full_scale_m");
  config.hybrid
      .cross_track_recovery_heading_error_suppression_start_rad =
      requiredDouble(
          private_node,
          "hybrid_cross_track_recovery_heading_error_suppression_start_deg") *
      kDegreesToRadians;
  config.hybrid
      .cross_track_recovery_heading_error_suppression_full_rad =
      requiredDouble(
          private_node,
          "hybrid_cross_track_recovery_heading_error_suppression_full_deg") *
      kDegreesToRadians;
  config.hybrid
      .cross_track_recovery_heading_error_maximum_suppression_ratio =
      requiredDouble(
          private_node,
          "hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio");
  const double hybrid_maximum_steering_rate_deg_per_sec = requiredDouble(
      private_node, "hybrid_maximum_steering_rate_deg_per_sec");
  const double hybrid_low_curvature_maximum_steering_rate_deg_per_sec =
      requiredDouble(
          private_node,
          "hybrid_low_curvature_maximum_steering_rate_deg_per_sec");
  const double hybrid_full_steering_rate_curvature_m_inv = requiredDouble(
      private_node, "hybrid_full_steering_rate_curvature_m_inv");
  config.hybrid.steering_return_rate_multiplier = requiredDouble(
      private_node, "hybrid_steering_return_rate_multiplier");
  const double target_speed_kph =
      requiredDouble(private_node, "target_speed_kph");
  const double minimum_curve_speed_kph =
      requiredDouble(private_node, "minimum_curve_speed_kph");
  config.curvature_speed_planner.configured_target_speed_mps =
      target_speed_kph * kKilometresPerHourToMetresPerSecond;
  config.curvature_speed_planner.minimum_curve_speed_mps =
      minimum_curve_speed_kph * kKilometresPerHourToMetresPerSecond;
  config.curvature_speed_planner.maximum_lateral_acceleration_mps2 =
      requiredDouble(private_node, "maximum_lateral_acceleration_mps2");
  config.curvature_speed_planner.curvature_speed_reduction_gain_m =
      requiredDouble(private_node, "curvature_speed_reduction_gain_m");
  config.curvature_speed_planner.preview_distance_m =
      requiredDouble(private_node, "curvature_preview_distance_m");
  config.curvature_speed_planner.lookahead_curvature_preview_distance_m =
      requiredDouble(private_node, "lookahead_curvature_preview_distance_m");
  config.curvature_speed_planner.curvature_sample_spacing_m =
      requiredDouble(private_node, "curvature_sample_spacing_m");
  config.curvature_speed_planner.curve_approach_deceleration_mps2 =
      requiredDouble(private_node, "curve_approach_deceleration_mps2");
  config.curvature_speed_planner.curvature_epsilon_m_inv =
      requiredDouble(private_node, "curvature_epsilon_m_inv");
  config.curvature_speed_planner.target_speed_acceleration_limit_mps2 =
      requiredDouble(private_node, "target_speed_acceleration_limit_mps2");
  config.curvature_speed_planner
      .curve_target_speed_acceleration_limit_mps2 =
      requiredDouble(
          private_node,
          "curve_target_speed_acceleration_limit_mps2");
  config.curvature_speed_planner.target_speed_deceleration_limit_mps2 =
      requiredDouble(private_node, "target_speed_deceleration_limit_mps2");
  config.curvature_speed_planner.target_speed_filter_time_constant_sec =
      requiredDouble(private_node, "target_speed_filter_time_constant_sec");
  config.speed_filter_time_constant_sec =
      requiredDouble(private_node, "speed_filter_time_constant_sec");
  config.pid.kp = requiredDouble(private_node, "speed_kp");
  config.pid.ki = requiredDouble(private_node, "speed_ki");
  config.pid.kd = requiredDouble(private_node, "speed_kd");
  config.pid.integral_limit =
      requiredDouble(private_node, "speed_integral_limit");
  config.pid.integral_unwind_rate_per_sec = requiredDouble(
      private_node, "speed_integral_unwind_rate_per_sec");
  config.pid.error_deadband_mps =
      requiredDouble(private_node, "speed_error_deadband_mps");
  config.pid.accel_feedforward_gain_per_mps =
      requiredDouble(private_node, "speed_accel_feedforward_gain_per_mps");
  const double speed_coast_overspeed_kph =
      requiredDouble(private_node, "speed_coast_overspeed_kph");
  const double speed_brake_overspeed_kph =
      requiredDouble(private_node, "speed_brake_overspeed_kph");
  const double hard_brake_activation_speed_kph =
      requiredDouble(private_node, "hard_brake_activation_speed_kph");
  config.pid.coast_overspeed_threshold_mps =
      speed_coast_overspeed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.pid.brake_overspeed_threshold_mps =
      speed_brake_overspeed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.pid.hard_brake_activation_speed_mps =
      hard_brake_activation_speed_kph *
      kKilometresPerHourToMetresPerSecond;
  config.pid.minimum_hard_brake_command =
      requiredDouble(private_node, "minimum_hard_brake_command");
  config.pid.maximum_accel =
      requiredDouble(private_node, "maximum_accel_command");
  config.pid.maximum_brake =
      requiredDouble(private_node, "maximum_brake_command");
  config.pid.command_rate_limit_per_sec = requiredDouble(
      private_node, "longitudinal_command_rate_limit_per_sec");
  if (config.longitudinal_controller == kMpcController) {
    config.longitudinal_mpc.solver =
      requiredString(private_node, "longitudinal_mpc_solver");
  config.longitudinal_mpc.horizon_steps =
      requiredInt(private_node, "longitudinal_mpc_horizon_steps");
  config.longitudinal_mpc.optimizer_iterations =
      requiredInt(private_node, "longitudinal_mpc_optimizer_iterations");
  config.longitudinal_mpc.line_search_steps =
      requiredInt(private_node, "longitudinal_mpc_line_search_steps");
  config.longitudinal_mpc.prediction_dt_sec =
      requiredDouble(private_node, "longitudinal_mpc_prediction_dt_sec");
  config.longitudinal_mpc.gradient_epsilon =
      requiredDouble(private_node, "longitudinal_mpc_gradient_epsilon");
  config.longitudinal_mpc.gradient_step =
      requiredDouble(private_node, "longitudinal_mpc_gradient_step");
  config.longitudinal_mpc.gradient_tolerance =
      requiredDouble(private_node, "longitudinal_mpc_gradient_tolerance");
  config.longitudinal_mpc.coordinate_initial_step = requiredDouble(
      private_node, "longitudinal_mpc_coordinate_initial_step");
  config.longitudinal_mpc.coordinate_minimum_step = requiredDouble(
      private_node, "longitudinal_mpc_coordinate_minimum_step");
  config.longitudinal_mpc.command_step_decay =
      requiredDouble(private_node, "longitudinal_mpc_command_step_decay");
  config.longitudinal_mpc.maximum_speed_mps =
      requiredDouble(private_node, "maximum_speed_kph") *
      kKilometresPerHourToMetresPerSecond;
  if (config.longitudinal_mpc.maximum_speed_mps >
      60.0 * kKilometresPerHourToMetresPerSecond + 1.0e-12) {
    throw std::invalid_argument("maximum_speed_kph must not exceed 60");
  }
  config.longitudinal_mpc.hard_brake_activation_speed_mps =
      config.pid.hard_brake_activation_speed_mps;
  config.longitudinal_mpc.minimum_hard_brake_command =
      config.pid.minimum_hard_brake_command;
  config.longitudinal_mpc.maximum_accel_command = config.pid.maximum_accel;
  config.longitudinal_mpc.maximum_brake_command = config.pid.maximum_brake;
  config.longitudinal_mpc.command_rate_limit_per_sec =
      config.pid.command_rate_limit_per_sec;
  config.longitudinal_mpc.acceleration_gain_mps2 = requiredDouble(
      private_node, "longitudinal_mpc_acceleration_gain_mps2");
  config.longitudinal_mpc.braking_gain_mps2 =
      requiredDouble(private_node, "longitudinal_mpc_braking_gain_mps2");
  config.longitudinal_mpc.actuator_time_constant_sec = requiredDouble(
      private_node, "longitudinal_mpc_actuator_time_constant_sec");
  config.longitudinal_mpc.maximum_acceleration_mps2 = requiredDouble(
      private_node, "longitudinal_mpc_maximum_acceleration_mps2");
  config.longitudinal_mpc.maximum_deceleration_mps2 = requiredDouble(
      private_node, "longitudinal_mpc_maximum_deceleration_mps2");
  config.longitudinal_mpc.maximum_jerk_mps3 =
      requiredDouble(private_node, "longitudinal_mpc_maximum_jerk_mps3");
  config.longitudinal_mpc.acceleration_estimate_filter_time_constant_sec =
      requiredDouble(private_node,
                     "longitudinal_mpc_acceleration_filter_tau_sec");
  config.longitudinal_mpc.speed_weight =
      requiredDouble(private_node, "longitudinal_mpc_weight_speed");
  config.longitudinal_mpc.acceleration_weight =
      requiredDouble(private_node, "longitudinal_mpc_weight_acceleration");
  config.longitudinal_mpc.jerk_weight =
      requiredDouble(private_node, "longitudinal_mpc_weight_jerk");
  config.longitudinal_mpc.command_weight =
      requiredDouble(private_node, "longitudinal_mpc_weight_command");
  config.longitudinal_mpc.command_rate_weight =
      requiredDouble(private_node, "longitudinal_mpc_weight_command_rate");
  config.longitudinal_mpc.terminal_speed_weight = requiredDouble(
      private_node, "longitudinal_mpc_weight_terminal_speed");
  config.longitudinal_mpc_fallback_to_pid =
      requiredBool(private_node, "longitudinal_mpc_fallback_to_pid");
  }

  requireRosName("local_path_topic", config.local_path_topic);
  requireRosName("ego_topic", config.ego_topic);
  requireRosName("vehicle_status_topic", config.vehicle_status_topic);
  requireRosName("command_topic", config.command_topic);
  requireRosName("controller_status_topic", config.controller_status_topic);
  requireRosName("lookahead_point_topic", config.lookahead_point_topic);
  requireRosName("stanley_projection_point_topic",
                 config.stanley_projection_point_topic);
  requireRosName("mpc_projection_point_topic",
                 config.mpc_projection_point_topic);
  if (config.expected_frame_id.empty() ||
      config.expected_velocity_frame_id.empty()) {
    throw std::invalid_argument("expected frame IDs must not be empty");
  }
  if (config.lateral_controller != kMpcController &&
      config.lateral_controller != kPurePursuitController &&
      config.lateral_controller != kStanleyController &&
      config.lateral_controller != kHybridController) {
    throw std::invalid_argument(
        "lateral_controller must be 'mpc' or 'hybrid' "
        "('pure_pursuit' and 'stanley' are legacy diagnostic modes)");
  }
  if (config.longitudinal_controller != kMpcController &&
      config.longitudinal_controller != kPidController) {
    throw std::invalid_argument(
        "longitudinal_controller must be 'mpc' or 'pid'");
  }
  config.control_period = periodFromRate(control_rate_hz);
  config.control_timing_bounds =
      ControlTimingBounds(minimum_control_dt_sec, maximum_control_dt_sec);
  if (!config.control_timing_bounds.contains(config.control_period.toSec())) {
    throw std::invalid_argument(
        "control_rate_hz period must be within control dt bounds");
  }
  requirePositive("path_timeout_sec", config.path_timeout_sec);
  requirePositive("ego_timeout_sec", config.ego_timeout_sec);
  requirePositive("vehicle_status_timeout_sec",
                  config.vehicle_status_timeout_sec);
  requireNonNegative("maximum_input_skew_sec", config.maximum_input_skew_sec);
  if (config.input_sync_queue_size <= 0) {
    throw std::invalid_argument("input_sync_queue_size must be positive");
  }
  if (!std::isfinite(config.safe_brake_command) ||
      config.safe_brake_command < 0.0 || config.safe_brake_command > 1.0) {
    throw std::invalid_argument(
        "safe_brake_command must be finite and in [0, 1]");
  }
  requirePositive("maximum_steering_angle_deg", maximum_steering_angle_deg);
  if (maximum_steering_angle_deg >= 90.0) {
    throw std::invalid_argument("maximum_steering_angle_deg must be below 90");
  }
  config.pure_pursuit.maximum_steering_angle_rad =
      maximum_steering_angle_deg * kDegreesToRadians;
  config.mpc.dt = config.control_period.toSec();
  config.mpc.max_steering =
      config.pure_pursuit.maximum_steering_angle_rad;
  requirePositive("mpc_maximum_steering_rate_deg_per_sec",
                  mpc_maximum_steering_rate_deg_per_sec);
  config.mpc.max_steering_rate =
      mpc_maximum_steering_rate_deg_per_sec * kDegreesToRadians;
  requirePositive("mpc_test_speed_limit_kph", mpc_test_speed_limit_kph);
  if (mpc_test_speed_limit_kph > 60.0) {
    throw std::invalid_argument("mpc_test_speed_limit_kph must not exceed 60");
  }
  config.mpc_test_speed_limit_mps =
      mpc_test_speed_limit_kph * kKilometresPerHourToMetresPerSecond;
  config.stanley.maximum_steering_angle_rad =
      config.pure_pursuit.maximum_steering_angle_rad;
  requirePositive("stanley_maximum_steering_rate_deg_per_sec",
                  stanley_maximum_steering_rate_deg_per_sec);
  config.stanley.maximum_steering_rate_rad_per_sec =
      stanley_maximum_steering_rate_deg_per_sec * kDegreesToRadians;
  requirePositive("hybrid_transition_reference_speed_kph",
                  hybrid_transition_reference_speed_kph);
  requirePositive("hybrid_maximum_steering_rate_deg_per_sec",
                  hybrid_maximum_steering_rate_deg_per_sec);
  requirePositive(
      "hybrid_low_curvature_maximum_steering_rate_deg_per_sec",
      hybrid_low_curvature_maximum_steering_rate_deg_per_sec);
  if (hybrid_low_curvature_maximum_steering_rate_deg_per_sec >
      hybrid_maximum_steering_rate_deg_per_sec) {
    throw std::invalid_argument(
        "hybrid_low_curvature_maximum_steering_rate_deg_per_sec must be "
        "no greater than hybrid_maximum_steering_rate_deg_per_sec");
  }
  requirePositive("hybrid_full_steering_rate_curvature_m_inv",
                  hybrid_full_steering_rate_curvature_m_inv);
  if (!std::isfinite(config.hybrid.steering_return_rate_multiplier) ||
      config.hybrid.steering_return_rate_multiplier < 1.0) {
    throw std::invalid_argument(
        "hybrid_steering_return_rate_multiplier must be finite and >= 1");
  }
  config.hybrid.pure_pursuit = config.pure_pursuit;
  config.hybrid.stanley = config.stanley;
  config.hybrid.maximum_steering_angle_rad =
      config.pure_pursuit.maximum_steering_angle_rad;
  config.hybrid.maximum_steering_rate_rad_per_sec =
      hybrid_maximum_steering_rate_deg_per_sec * kDegreesToRadians;
  config.hybrid.low_curvature_maximum_steering_rate_rad_per_sec =
      hybrid_low_curvature_maximum_steering_rate_deg_per_sec *
      kDegreesToRadians;
  config.hybrid.full_steering_rate_curvature_m_inv =
      hybrid_full_steering_rate_curvature_m_inv;
  if (std::abs(config.hybrid.imm.front_axle_to_cg_m +
                   config.hybrid.imm.rear_axle_to_cg_m -
               config.pure_pursuit.wheelbase_m) >
      1.0e-6) {
    throw std::invalid_argument(
        "hybrid axle-to-CG distances must sum to wheelbase_m");
  }
  requireNonNegative("target_speed_kph", target_speed_kph);
  requireNonNegative("minimum_curve_speed_kph", minimum_curve_speed_kph);
  if (config.wheel_corridor.lane_half_width_m <=
      0.5 * config.wheel_corridor.vehicle_width_m) {
    throw std::invalid_argument(
        "lane_half_width_m must exceed half vehicle_width_m");
  }
  requireNonNegative("speed_filter_time_constant_sec",
                     config.speed_filter_time_constant_sec);
  if (config.pid.maximum_accel > 1.0 || config.pid.maximum_brake > 1.0) {
    throw std::invalid_argument(
        "maximum_accel_command and maximum_brake_command must be in [0, 1]");
  }

  // The core constructors enforce all finite/range constraints and the
  // Pure Pursuit lookahead cross-field relation before the timer starts.
  LongitudinalPid pid_validation(config.pid);
  LongitudinalMpc longitudinal_mpc_validation(config.longitudinal_mpc);
  CurvatureSpeedPlanner speed_planner_validation(
      config.curvature_speed_planner);
  StanleyController stanley_validation(config.stanley);
  const StanleyResult stanley_validation_result =
      stanley_validation.calculate({{0.0, 0.0}, {1.0, 0.0}}, 0.0, 0.0,
                                   config.control_period.toSec());
  if (!stanley_validation_result.valid) {
    throw std::invalid_argument(stanley_validation_result.error);
  }
  HybridController hybrid_validation(config.hybrid);
  const HybridResult hybrid_validation_result =
      hybrid_validation.calculate(
          {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}}, 5.0, 0.0, 0.0,
          0.0, 0.0, config.control_period.toSec());
  if (!hybrid_validation_result.valid) {
    throw std::invalid_argument(hybrid_validation_result.error);
  }
  MpcLateralController mpc_validation(config.mpc);
  const MpcLateralResult mpc_validation_result = mpc_validation.calculate(
      {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}}, 5.0, 0.0, 0.0,
      config.control_period.toSec());
  if (!mpc_validation_result.valid) {
    throw std::invalid_argument(mpc_validation_result.error);
  }
  const WheelCorridorResult wheel_corridor_validation =
      estimateWheelCorridor({{0.0, 0.0}, {10.0, 0.0}},
                            config.wheel_corridor);
  if (!wheel_corridor_validation.valid) {
    throw std::invalid_argument(wheel_corridor_validation.error);
  }
  const LaneClearanceSpeedLimit lane_speed_validation =
      computeLaneClearanceSpeedLimit(
          wheel_corridor_validation.minimum_clearance_m,
          config.curvature_speed_planner.configured_target_speed_mps,
          config.lane_clearance_speed);
  if (!lane_speed_validation.valid) {
    throw std::invalid_argument(lane_speed_validation.error);
  }
  const HeadingErrorSpeedLimit heading_speed_validation =
      computeHeadingErrorSpeedLimit(
          0.0, config.curvature_speed_planner.configured_target_speed_mps,
          config.heading_error_speed);
  if (!heading_speed_validation.valid) {
    throw std::invalid_argument(heading_speed_validation.error);
  }
  (void)pid_validation;
  (void)longitudinal_mpc_validation;
  (void)speed_planner_validation;
  (void)wheel_corridor_validation;
  (void)lane_speed_validation;
  (void)heading_speed_validation;
  (void)computePurePursuit({}, 0.0, 0.0, config.pure_pursuit);
  return config;
}

bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion) {
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w) &&
         std::isfinite(quaternion.x * quaternion.x +
                       quaternion.y * quaternion.y +
                       quaternion.z * quaternion.z +
                       quaternion.w * quaternion.w) &&
         (quaternion.x * quaternion.x + quaternion.y * quaternion.y +
              quaternion.z * quaternion.z + quaternion.w * quaternion.w) >
             0.0;
}

}  // namespace

class PathTrackingControllerNode {
 public:
  using InputSyncPolicy =
      message_filters::sync_policies::ApproximateTime<nav_msgs::Path,
                                                       geometry_msgs::PoseStamped>;

  PathTrackingControllerNode()
      : private_node_("~"),
        config_(loadConfig(private_node_)),
        pid_(config_.pid),
        longitudinal_mpc_(config_.longitudinal_mpc),
        curvature_speed_planner_(config_.curvature_speed_planner),
        stanley_controller_(config_.stanley),
        hybrid_controller_(config_.hybrid),
        mpc_controller_(config_.mpc),
        path_subscriber_(node_, config_.local_path_topic, 1U),
        ego_subscriber_(node_, config_.ego_topic, 1U),
        input_synchronizer_(
            InputSyncPolicy(
                static_cast<std::uint32_t>(config_.input_sync_queue_size)),
            path_subscriber_, ego_subscriber_) {
    publisher_ = node_.advertise<morai_msgs::CtrlCmd>(
        config_.command_topic, 1U);
    controller_status_publisher_ =
        node_.advertise<ControllerStatus>(config_.controller_status_topic, 10U);
    lookahead_point_publisher_ = node_.advertise<geometry_msgs::PointStamped>(
        config_.lookahead_point_topic, 10U);
    stanley_projection_point_publisher_ =
        node_.advertise<geometry_msgs::PointStamped>(
            config_.stanley_projection_point_topic, 10U);
    mpc_projection_point_publisher_ =
        node_.advertise<geometry_msgs::PointStamped>(
            config_.mpc_projection_point_topic, 10U);
    mpc_compensated_path_publisher_ = node_.advertise<nav_msgs::Path>(
        config_.mpc_compensated_path_topic, 1U);
    vehicle_status_subscriber_ = node_.subscribe(
        config_.vehicle_status_topic, 1U,
        &PathTrackingControllerNode::onVehicleStatus, this);
    input_synchronizer_.setMaxIntervalDuration(
        ros::Duration(config_.maximum_input_skew_sec));
    input_synchronizer_.registerCallback(
        boost::bind(&PathTrackingControllerNode::onSynchronizedInputs, this,
                    boost::placeholders::_1, boost::placeholders::_2));
    timer_ = node_.createWallTimer(config_.control_period,
                                   &PathTrackingControllerNode::onTimer, this);
    ROS_INFO(
        "Path tracking controller: lateral=%s/%s, longitudinal=%s/%s, configured_target=%.3f km/h "
        "(%.3f m/s), speed_source=%s, speed_filter_tau=%.3f s",
        config_.lateral_controller.c_str(),
        config_.mpc.solver.c_str(), config_.longitudinal_controller.c_str(),
        config_.longitudinal_mpc.solver.c_str(),
        config_.curvature_speed_planner.configured_target_speed_mps * 3.6,
        config_.curvature_speed_planner.configured_target_speed_mps,
        config_.vehicle_status_topic.c_str(),
        config_.speed_filter_time_constant_sec);
  }

 private:
  void onSynchronizedInputs(
      const nav_msgs::Path::ConstPtr& path,
      const geometry_msgs::PoseStamped::ConstPtr& ego) {
    if (latest_ego_) {
      const double dt = (ego->header.stamp - latest_ego_->header.stamp).toSec();
      if (dt > 0 && dt <= config_.ego_timeout_sec && finiteQuaternion(ego->pose.orientation)) {
        const double yaw = tf2::getYaw(ego->pose.orientation);
        const double vx_map = (ego->pose.position.x - latest_ego_->pose.position.x) / dt;
        const double vy_map = (ego->pose.position.y - latest_ego_->pose.position.y) / dt;
        lateral_velocity_mps_ = -std::sin(yaw) * vx_map + std::cos(yaw) * vy_map;
      } else if (dt != 0) {
        lateral_velocity_mps_ = std::numeric_limits<double>::quiet_NaN();
      }
      // 같은 stamp는 미분하지 않고 이전 추정값을 유지한다.
    }
    latest_path_ = path;
    latest_ego_ = ego;
    synchronized_input_receipt_time_ = ros::SteadyTime::now();
  }

  void onVehicleStatus(
      const interfaces::CompetitionVehicleStatus::ConstPtr& message) {
    const ros::SteadyTime now = ros::SteadyTime::now();
    const double raw_speed = message->velocity_x_mps;
    if (std::isfinite(raw_speed) &&
        config_.speed_filter_time_constant_sec > 0.0 &&
        has_vehicle_status_ &&
        std::isfinite(filtered_velocity_x_mps_)) {
      const double dt_sec = (now - vehicle_status_receipt_time_).toSec();
      if (std::isfinite(dt_sec) && dt_sec > 0.0 &&
          dt_sec <= config_.vehicle_status_timeout_sec) {
        const double alpha =
            1.0 - std::exp(-dt_sec /
                           config_.speed_filter_time_constant_sec);
        filtered_velocity_x_mps_ +=
            alpha * (raw_speed - filtered_velocity_x_mps_);
      } else {
        filtered_velocity_x_mps_ = raw_speed;
      }
    } else {
      filtered_velocity_x_mps_ = raw_speed;
    }
    latest_vehicle_status_ = message;
    vehicle_status_receipt_time_ = now;
    has_vehicle_status_ = true;
  }

  bool validInputs(const ros::SteadyTime& now, std::vector<Point2d>* path,
                   double* speed_mps, double* lateral_velocity_mps,
                   double* yaw_rate_radps,
                   std::string* reason) const {
    if (!latest_path_ || !latest_ego_) {
      *reason = "WAITING_FOR_SYNCHRONIZED_PATH_EGO";
      return false;
    }
    if (!has_vehicle_status_ || !latest_vehicle_status_) {
      *reason = "WAITING_FOR_COMPETITION_VEHICLE_STATUS";
      return false;
    }
    const nav_msgs::Path& path_message = *latest_path_;
    const geometry_msgs::PoseStamped& ego_message = *latest_ego_;
    if (path_message.header.frame_id != config_.expected_frame_id ||
        ego_message.header.frame_id != config_.expected_frame_id ||
        path_message.header.stamp.isZero() ||
        ego_message.header.stamp.isZero()) {
      *reason = "INVALID_PATH_EGO_FRAME_OR_STAMP";
      return false;
    }

    const double synchronized_receipt_age =
        (now - synchronized_input_receipt_time_).toSec();
    if (!std::isfinite(synchronized_receipt_age) ||
        synchronized_receipt_age < 0.0 ||
        synchronized_receipt_age > config_.path_timeout_sec ||
        synchronized_receipt_age > config_.ego_timeout_sec) {
      *reason = "STALE_SYNCHRONIZED_PATH_EGO";
      return false;
    }
    const double status_receipt_age =
        (now - vehicle_status_receipt_time_).toSec();
    if (!std::isfinite(status_receipt_age) || status_receipt_age < 0.0 ||
        status_receipt_age > config_.vehicle_status_timeout_sec) {
      *reason = "STALE_COMPETITION_VEHICLE_STATUS";
      return false;
    }

    const ros::Time ros_now = ros::Time::now();
    const double path_stamp_age = (ros_now - path_message.header.stamp).toSec();
    const double ego_stamp_age =
        (ros_now - ego_message.header.stamp).toSec();
    const double skew =
        std::abs((path_message.header.stamp - ego_message.header.stamp).toSec());
    if (!std::isfinite(path_stamp_age) || !std::isfinite(ego_stamp_age) ||
        !std::isfinite(skew) || path_stamp_age < 0.0 ||
        ego_stamp_age < 0.0 || path_stamp_age > config_.path_timeout_sec ||
        ego_stamp_age > config_.ego_timeout_sec ||
        skew > config_.maximum_input_skew_sec) {
      *reason = "INVALID_PATH_EGO_SOURCE_TIME";
      return false;
    }

    const interfaces::CompetitionVehicleStatus& vehicle_status =
        *latest_vehicle_status_;
    if (vehicle_status.header.frame_id !=
            config_.expected_velocity_frame_id ||
        vehicle_status.header.stamp.isZero() ||
        !std::isfinite(filtered_velocity_x_mps_)) {
      *reason = "INVALID_COMPETITION_VEHICLE_STATUS";
      return false;
    }

    const geometry_msgs::Point& position = ego_message.pose.position;
    const geometry_msgs::Quaternion& orientation =
        ego_message.pose.orientation;
    if (!std::isfinite(lateral_velocity_mps_)) {
      *reason = "WAITING_FOR_EGO_VELOCITY";
      return false;
    }
    const double lateral_velocity = lateral_velocity_mps_;
    const double yaw_rate = vehicle_status.angular_velocity_z_radps;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !finiteQuaternion(orientation) ||
        !std::isfinite(filtered_velocity_x_mps_) ||
        !std::isfinite(lateral_velocity) ||
        !std::isfinite(yaw_rate)) {
      *reason = "INVALID_EGO_POSE";
      return false;
    }
    const double yaw = tf2::getYaw(orientation);
    if (!std::isfinite(yaw)) {
      *reason = "INVALID_EGO_YAW";
      return false;
    }

    path->clear();
    path->reserve(path_message.poses.size());
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (const geometry_msgs::PoseStamped& pose : path_message.poses) {
      const double point_x = pose.pose.position.x;
      const double point_y = pose.pose.position.y;
      if (!std::isfinite(point_x) || !std::isfinite(point_y)) {
        *reason = "INVALID_LOCAL_PATH_POINT";
        return false;
      }
      const double dx = point_x - position.x;
      const double dy = point_y - position.y;
      const double x_body = cos_yaw * dx + sin_yaw * dy;
      const double y_body = -sin_yaw * dx + cos_yaw * dy;
      if (!std::isfinite(x_body) || !std::isfinite(y_body)) {
        *reason = "INVALID_TRANSFORMED_PATH_POINT";
        return false;
      }
      path->push_back({x_body, y_body});
    }
    *speed_mps = filtered_velocity_x_mps_;
    *lateral_velocity_mps = lateral_velocity;
    *yaw_rate_radps = yaw_rate;
    reason->clear();
    return true;
  }

  void publishControllerStatus(
      const ros::Time& stamp, bool active, const std::string& state,
      double measured_speed_mps, double measured_yaw_rate_radps,
      const CurvatureSpeedPlan* speed_plan,
      const WheelCorridorResult* wheel_corridor,
      const LaneClearanceSpeedLimit* lane_speed_limit,
      const HeadingErrorSpeedLimit* heading_speed_limit,
      const LateralControlOutput* lateral, double accel, double brake,
      const std::string& longitudinal_state,
      const LongitudinalMpcResult* longitudinal_mpc_result = nullptr,
      bool longitudinal_mpc_fallback_active = false) {
    ControllerStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = config_.expected_velocity_frame_id;
    status.active = active;
    status.state = state;
    status.lateral_controller = config_.lateral_controller;
    status.lateral_mpc_solver = config_.mpc.solver;
    status.longitudinal_controller = config_.longitudinal_controller;
    status.longitudinal_mpc_solver = config_.longitudinal_mpc.solver;
    status.configured_target_speed_mps =
        config_.curvature_speed_planner.configured_target_speed_mps;
    status.lane_half_width_m = config_.wheel_corridor.lane_half_width_m;
    status.vehicle_width_m = config_.wheel_corridor.vehicle_width_m;
    status.speed_limiting_curve_distance_m = -1.0;
    if (speed_plan != nullptr) {
      status.raw_target_speed_mps = speed_plan->raw_target_speed_mps;
      status.filtered_target_speed_mps =
          speed_plan->filtered_target_speed_mps;
      status.target_speed_mps = speed_plan->target_speed_mps;
      status.preview_curvature_m_inv =
          speed_plan->preview_curvature_m_inv;
      status.speed_limiting_curve_distance_m =
          speed_plan->speed_limiting_curve_distance_m;
      status.lookahead_curvature_m_inv =
          speed_plan->lookahead_curvature_m_inv;
      status.curvature_speed_limit_mps =
          speed_plan->curvature_speed_limit_mps;
    }
    if (wheel_corridor != nullptr) {
      status.wheel_outer_offset_m =
          wheel_corridor->maximum_wheel_offset_m;
      status.wheel_minimum_clearance_m =
          wheel_corridor->minimum_clearance_m;
    }
    if (lane_speed_limit != nullptr) {
      status.lane_clearance_recovery_urgency =
          lane_speed_limit->urgency;
      status.lane_clearance_speed_limit_mps =
          lane_speed_limit->speed_limit_mps;
    }
    if (heading_speed_limit != nullptr) {
      status.heading_error_speed_limit_urgency =
          heading_speed_limit->urgency;
      status.heading_error_speed_limit_mps =
          heading_speed_limit->speed_limit_mps;
    }
    status.measured_velocity_x_mps =
        std::isfinite(measured_speed_mps) ? measured_speed_mps : 0.0;
    status.measured_yaw_rate_radps =
        std::isfinite(measured_yaw_rate_radps)
            ? measured_yaw_rate_radps
            : 0.0;
    status.speed_error_mps =
        status.target_speed_mps - status.measured_velocity_x_mps;
    status.speed_overshoot_mps =
        speed_plan == nullptr
            ? 0.0
            : std::max(0.0, status.measured_velocity_x_mps -
                                status.target_speed_mps);
    status.longitudinal_state = longitudinal_state;
    status.accel = accel;
    status.brake = brake;
    status.longitudinal_mpc_fallback_active =
        longitudinal_mpc_fallback_active;
    if (longitudinal_mpc_result != nullptr) {
      status.longitudinal_mpc_solver_success =
          longitudinal_mpc_result->valid;
      status.longitudinal_mpc_hard_speed_guard_active =
          longitudinal_mpc_result->hard_speed_guard_active;
      status.longitudinal_mpc_solver_iterations =
          longitudinal_mpc_result->iterations;
      status.longitudinal_mpc_solver_time_ms =
          longitudinal_mpc_result->solve_time_ms;
      status.longitudinal_mpc_solver_cost = longitudinal_mpc_result->cost;
      status.longitudinal_mpc_signed_command =
          longitudinal_mpc_result->signed_command;
      status.longitudinal_mpc_estimated_acceleration_mps2 =
          longitudinal_mpc_result->estimated_acceleration_mps2;
      status.longitudinal_mpc_predicted_maximum_speed_mps =
          longitudinal_mpc_result->predicted_maximum_speed_mps;
    }
    if (lateral != nullptr) {
      status.steering_angle_rad = lateral->steering_angle_rad;
      status.cross_track_error_m = lateral->cross_track_error_m;
      status.mpc_raw_cross_track_error_m =
          lateral->mpc_raw_cross_track_error_m;
      status.heading_error_rad = lateral->heading_error_rad;
      status.reference_curvature_m_inv =
          lateral->reference_curvature_m_inv;
      status.reference_yaw_rate_radps =
          lateral->reference_yaw_rate_radps;
      status.yaw_rate_error_radps = lateral->yaw_rate_error_radps;
      status.curvature_feedforward_steering_rad =
          lateral->curvature_feedforward_steering_rad;
      status.heading_feedback_steering_rad =
          lateral->heading_feedback_steering_rad;
      status.cross_track_feedback_steering_rad =
          lateral->cross_track_feedback_steering_rad;
      status.applied_yaw_rate_damping_gain_sec =
          lateral->applied_yaw_rate_damping_gain_sec;
      status.yaw_rate_damping_steering_rad =
          lateral->yaw_rate_damping_steering_rad;
      status.requested_steering_angle_rad =
          lateral->requested_steering_angle_rad;
      status.pure_pursuit_steering_angle_rad =
          lateral->pure_pursuit_steering_angle_rad;
      status.hybrid_corrected_pure_pursuit_steering_angle_rad =
          lateral
              ->hybrid_corrected_pure_pursuit_steering_angle_rad;
      status.stanley_steering_angle_rad =
          lateral->stanley_steering_angle_rad;
      status.hybrid_pure_pursuit_probability =
          lateral->hybrid_pure_pursuit_probability;
      status.hybrid_stanley_probability =
          lateral->hybrid_stanley_probability;
      status.hybrid_effective_pure_pursuit_weight =
          lateral->hybrid_effective_pure_pursuit_weight;
      status.hybrid_effective_stanley_weight =
          lateral->hybrid_effective_stanley_weight;
      status.hybrid_candidate_conflict_guard_active =
          lateral->hybrid_candidate_conflict_guard_active;
      status.hybrid_candidate_conflict_stanley_override_active =
          lateral->hybrid_candidate_conflict_stanley_override_active;
      status.hybrid_cross_track_recovery_active =
          lateral->hybrid_cross_track_recovery_active;
      status.hybrid_cross_track_recovery_weight =
          lateral->hybrid_cross_track_recovery_weight;
      status.hybrid_cross_track_recovery_heading_suppression_active =
          lateral
              ->hybrid_cross_track_recovery_heading_suppression_active;
      status.hybrid_cross_track_recovery_heading_suppression_weight =
          lateral
              ->hybrid_cross_track_recovery_heading_suppression_weight;
      status.hybrid_lane_clearance_recovery_active =
          lateral->hybrid_lane_clearance_recovery_active;
      status.hybrid_curve_preview_stanley_recovery_active =
          lateral->hybrid_curve_preview_stanley_recovery_active;
      status.hybrid_curve_preview_stanley_recovery_weight =
          lateral->hybrid_curve_preview_stanley_recovery_weight;
      status.hybrid_heading_lag_stanley_recovery_active =
          lateral->hybrid_heading_lag_stanley_recovery_active;
      status.hybrid_heading_lag_stanley_recovery_weight =
          lateral->hybrid_heading_lag_stanley_recovery_weight;
      status.hybrid_applied_maximum_steering_rate_rad_per_sec =
          lateral->hybrid_applied_maximum_steering_rate_rad_per_sec;
      status.lane_clearance_recovery_urgency =
          std::max(status.lane_clearance_recovery_urgency,
                   lateral->lane_clearance_recovery_urgency);
      status.measured_sideslip_angle_rad =
          lateral->measured_sideslip_angle_rad;
      status.pure_pursuit_innovation_norm =
          lateral->pure_pursuit_innovation_norm;
      status.stanley_innovation_norm =
          lateral->stanley_innovation_norm;
      status.stanley_projection_point_base.x =
          lateral->stanley_projection.x;
      status.stanley_projection_point_base.y =
          lateral->stanley_projection.y;
      status.stanley_projection_point_base.z = 0.0;
      status.mpc_solver_success = lateral->mpc_solver_success;
      status.mpc_solver_iterations = lateral->mpc_solver_iterations;
      status.mpc_solver_time_ms = lateral->mpc_solver_time_ms;
      status.mpc_solver_cost = lateral->mpc_solver_cost;
      status.mpc_steering_rate_rad_per_sec =
          lateral->mpc_steering_rate_rad_per_sec;
      status.mpc_raw_steering_rate_rad_per_sec =
          lateral->mpc_raw_steering_rate_rad_per_sec;
      status.mpc_yaw_rate_steering_estimate_blend =
          lateral->mpc_yaw_rate_steering_estimate_blend;
      status.mpc_modeled_steering_angle_rad =
          lateral->mpc_modeled_steering_angle_rad;
      status.mpc_projection_point_base.x = lateral->mpc_projection.x;
      status.mpc_projection_point_base.y = lateral->mpc_projection.y;
      status.mpc_projection_point_base.z = 0.0;
      if (lateral->has_tracking_target) {
        status.lookahead_distance_m = lateral->lookahead_m;
        status.lookahead_point_base.x = lateral->target.x;
        status.lookahead_point_base.y = lateral->target.y;
        status.lookahead_point_base.z = 0.0;
      }
    }
    controller_status_publisher_.publish(status);
  }

  void publishSafe(const std::string& reason) noexcept {
    try {
      pid_.reset();
      longitudinal_mpc_.reset();
      curvature_speed_planner_.reset();
      hybrid_controller_.reset();
      mpc_controller_.reset();
      previous_steering_angle_rad_ = 0.0;
      const ros::Time stamp = ros::Time::now();
      morai_msgs::CtrlCmd output;
      output.longlCmdType = 1;
      output.accel = 0.0F;
      output.brake = static_cast<float>(config_.safe_brake_command);
      output.steering = 0.0F;
      publisher_.publish(output);
      publishControllerStatus(
          stamp, false, reason, filtered_velocity_x_mps_, 0.0, nullptr,
          nullptr, nullptr, nullptr, nullptr, output.accel, output.brake,
          "SAFE_BRAKE");
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "failed to publish safe controller command: %s",
                         error.what());
    } catch (...) {
      ROS_ERROR_THROTTLE(1.0,
                         "failed to publish safe controller command: unknown exception");
    }
  }

  void onTimer(const ros::WallTimerEvent&) {
    try {
      const ros::SteadyTime now = ros::SteadyTime::now();
      const double dt_sec = has_last_timer_time_
                                ? (now - last_timer_time_).toSec()
                                : std::numeric_limits<double>::quiet_NaN();
      last_timer_time_ = now;
      has_last_timer_time_ = true;
      if (!config_.control_timing_bounds.contains(dt_sec)) {
        publishSafe("INVALID_CONTROL_DT");
        return;
      }

      std::vector<Point2d> vehicle_path;
      double speed_mps = 0.0;
      double lateral_velocity_mps = 0.0;
      double yaw_rate_radps = 0.0;
      std::string invalid_reason;
      if (!validInputs(now, &vehicle_path, &speed_mps,
                       &lateral_velocity_mps, &yaw_rate_radps,
                       &invalid_reason)) {
        publishSafe(invalid_reason);
        return;
      }

      const WheelCorridorResult wheel_corridor =
          estimateWheelCorridor(vehicle_path, config_.wheel_corridor);
      if (!wheel_corridor.valid) {
        publishSafe("INVALID_WHEEL_CORRIDOR");
        return;
      }
      const LaneClearanceSpeedLimit lane_speed_limit =
          computeLaneClearanceSpeedLimit(
              wheel_corridor.minimum_clearance_m,
              config_.curvature_speed_planner.configured_target_speed_mps,
              config_.lane_clearance_speed);
      if (!lane_speed_limit.valid) {
        publishSafe("INVALID_LANE_CLEARANCE_SPEED_LIMIT");
        return;
      }
      CurvatureSpeedPlan speed_plan =
          curvature_speed_planner_.update(vehicle_path, dt_sec);
      if (config_.lateral_controller == kMpcController) {
        speed_plan.target_speed_mps =
            std::min(speed_plan.target_speed_mps,
                     config_.mpc_test_speed_limit_mps);
      }
      speed_plan.target_speed_mps =
          std::min(speed_plan.target_speed_mps,
                   lane_speed_limit.speed_limit_mps);
      LateralControlOutput lateral;
      if (config_.lateral_controller == kMpcController) {
        std::vector<morai_mpc::Point2d> mpc_path;
        mpc_path.reserve(vehicle_path.size());
        for (const Point2d& point : vehicle_path) {
          mpc_path.push_back({point.x, point.y});
        }
        const MpcLateralResult mpc = mpc_controller_.calculate(
            mpc_path, speed_mps, lateral_velocity_mps, yaw_rate_radps,
            previous_steering_angle_rad_, dt_sec);
        lateral.valid = mpc.valid;
        lateral.has_tracking_target = mpc.valid;
        lateral.target = {mpc.projection.x, mpc.projection.y};
        lateral.has_stanley_projection = false;
        lateral.steering_angle_rad = mpc.steering_angle_rad;
        lateral.cross_track_error_m = mpc.cross_track_error_m;
        lateral.mpc_raw_cross_track_error_m =
            mpc.raw_cross_track_error_m;
        lateral.heading_error_rad = mpc.heading_error_rad;
        lateral.reference_curvature_m_inv =
            mpc.reference_curvature_m_inv;
        lateral.requested_steering_angle_rad = mpc.steering_angle_rad;
        lateral.mpc_solver_success = mpc.valid;
        lateral.mpc_solver_iterations = mpc.solver_iterations;
        lateral.mpc_solver_time_ms = mpc.solver_time_ms;
        lateral.mpc_solver_cost = mpc.solver_cost;
        lateral.mpc_steering_rate_rad_per_sec =
            mpc.steering_rate_rad_per_sec;
        lateral.mpc_raw_steering_rate_rad_per_sec =
            mpc.raw_steering_rate_rad_per_sec;
        lateral.mpc_yaw_rate_steering_estimate_blend =
            mpc.yaw_rate_steering_estimate_blend;
        lateral.mpc_modeled_steering_angle_rad =
            mpc.modeled_steering_angle_rad;
        lateral.mpc_projection = {mpc.projection.x, mpc.projection.y};
        lateral.mpc_compensated_path = mpc.compensated_path;
        lateral.error = mpc.error.empty() ? "INVALID_MPC_CONTROL" : mpc.error;
      } else if (config_.lateral_controller == kPurePursuitController) {
        const PurePursuitResult pure_pursuit = computePurePursuit(
            vehicle_path, speed_mps,
            speed_plan.lookahead_curvature_m_inv,
            config_.pure_pursuit);
        lateral.valid = pure_pursuit.valid;
        lateral.has_tracking_target = pure_pursuit.valid;
        lateral.target = pure_pursuit.target;
        lateral.lookahead_m = pure_pursuit.lookahead_m;
        lateral.steering_angle_rad =
            pure_pursuit.steering_angle_rad;
        lateral.error = "NO_VALID_LOOKAHEAD_TARGET";
      } else if (config_.lateral_controller == kStanleyController) {
        const StanleyResult stanley = stanley_controller_.calculate(
            vehicle_path, std::abs(speed_mps),
            yaw_rate_radps, previous_steering_angle_rad_, dt_sec);
        lateral.valid = stanley.valid;
        lateral.has_tracking_target = stanley.valid;
        lateral.target = stanley.target;
        lateral.steering_angle_rad = stanley.steering_angle_rad;
        lateral.cross_track_error_m = stanley.cross_track_error_m;
        lateral.heading_error_rad = stanley.heading_error_rad;
        lateral.reference_curvature_m_inv =
            stanley.reference_curvature_m_inv;
        lateral.reference_yaw_rate_radps =
            stanley.reference_yaw_rate_radps;
        lateral.yaw_rate_error_radps = stanley.yaw_rate_error_radps;
        lateral.curvature_feedforward_steering_rad =
            stanley.curvature_feedforward_steering_rad;
        lateral.heading_feedback_steering_rad =
            stanley.heading_feedback_steering_rad;
        lateral.cross_track_feedback_steering_rad =
            stanley.cross_track_feedback_steering_rad;
        lateral.applied_yaw_rate_damping_gain_sec =
            stanley.applied_yaw_rate_damping_gain_sec;
        lateral.yaw_rate_damping_steering_rad =
            stanley.yaw_rate_damping_steering_rad;
        lateral.requested_steering_angle_rad =
            stanley.requested_steering_angle_rad;
        lateral.stanley_steering_angle_rad =
            stanley.requested_steering_angle_rad;
        lateral.stanley_projection = stanley.target;
        lateral.has_stanley_projection = stanley.valid;
        lateral.error = stanley.error.empty() ? "INVALID_STANLEY_CONTROL"
                                              : stanley.error;
      } else {
        const HybridResult hybrid = hybrid_controller_.calculate(
            vehicle_path, std::abs(speed_mps), lateral_velocity_mps,
            yaw_rate_radps, speed_plan.lookahead_curvature_m_inv,
            wheel_corridor.minimum_clearance_m,
            previous_steering_angle_rad_, dt_sec);
        lateral.valid = hybrid.valid;
        lateral.has_tracking_target = hybrid.valid;
        lateral.target = hybrid.pure_pursuit_target;
        lateral.lookahead_m = hybrid.pure_pursuit.lookahead_m;
        lateral.steering_angle_rad = hybrid.steering_angle_rad;
        lateral.cross_track_error_m =
            hybrid.stanley.cross_track_error_m;
        lateral.heading_error_rad = hybrid.stanley.heading_error_rad;
        lateral.reference_curvature_m_inv =
            hybrid.stanley.reference_curvature_m_inv;
        lateral.reference_yaw_rate_radps =
            hybrid.stanley.reference_yaw_rate_radps;
        lateral.yaw_rate_error_radps =
            hybrid.stanley.yaw_rate_error_radps;
        lateral.curvature_feedforward_steering_rad =
            hybrid.stanley.curvature_feedforward_steering_rad;
        lateral.heading_feedback_steering_rad =
            hybrid.stanley.heading_feedback_steering_rad;
        lateral.cross_track_feedback_steering_rad =
            hybrid.stanley.cross_track_feedback_steering_rad;
        lateral.applied_yaw_rate_damping_gain_sec =
            hybrid.stanley.applied_yaw_rate_damping_gain_sec;
        lateral.yaw_rate_damping_steering_rad =
            hybrid.stanley.yaw_rate_damping_steering_rad;
        lateral.requested_steering_angle_rad =
            hybrid.requested_steering_angle_rad;
        lateral.pure_pursuit_steering_angle_rad =
            hybrid.pure_pursuit.steering_angle_rad;
        lateral.hybrid_corrected_pure_pursuit_steering_angle_rad =
            hybrid.corrected_pure_pursuit_steering_angle_rad;
        lateral.stanley_steering_angle_rad =
            hybrid.stanley.requested_steering_angle_rad;
        lateral.hybrid_pure_pursuit_probability =
            hybrid.imm.pure_pursuit_probability;
        lateral.hybrid_stanley_probability =
            hybrid.imm.stanley_probability;
        lateral.hybrid_effective_pure_pursuit_weight =
            hybrid.effective_pure_pursuit_weight;
        lateral.hybrid_effective_stanley_weight =
            hybrid.effective_stanley_weight;
        lateral.hybrid_candidate_conflict_guard_active =
            hybrid.candidate_conflict_guard_active;
        lateral.hybrid_candidate_conflict_stanley_override_active =
            hybrid.candidate_conflict_stanley_override_active;
        lateral.hybrid_cross_track_recovery_active =
            hybrid.cross_track_recovery_active;
        lateral.hybrid_cross_track_recovery_weight =
            hybrid.cross_track_recovery_weight;
        lateral.hybrid_cross_track_recovery_heading_suppression_active =
            hybrid
                .cross_track_recovery_heading_suppression_active;
        lateral.hybrid_cross_track_recovery_heading_suppression_weight =
            hybrid
                .cross_track_recovery_heading_suppression_weight;
        lateral.hybrid_lane_clearance_recovery_active =
            hybrid.lane_clearance_recovery_active;
        lateral.lane_clearance_recovery_urgency =
            hybrid.lane_clearance_recovery_urgency;
        lateral.hybrid_curve_preview_stanley_recovery_active =
            hybrid.curve_preview_stanley_recovery_active;
        lateral.hybrid_curve_preview_stanley_recovery_weight =
            hybrid.curve_preview_stanley_recovery_weight;
        lateral.hybrid_heading_lag_stanley_recovery_active =
            hybrid.heading_lag_stanley_recovery_active;
        lateral.hybrid_heading_lag_stanley_recovery_weight =
            hybrid.heading_lag_stanley_recovery_weight;
        lateral.hybrid_applied_maximum_steering_rate_rad_per_sec =
            hybrid.applied_maximum_steering_rate_rad_per_sec;
        lateral.measured_sideslip_angle_rad =
            hybrid.measured_sideslip_angle_rad;
        lateral.pure_pursuit_innovation_norm =
            hybrid.imm.pure_pursuit_innovation_norm;
        lateral.stanley_innovation_norm =
            hybrid.imm.stanley_innovation_norm;
        lateral.stanley_projection = hybrid.stanley_projection;
        lateral.has_stanley_projection = hybrid.valid;
        lateral.error = hybrid.error.empty() ? "INVALID_HYBRID_CONTROL"
                                             : hybrid.error;
      }
      if (!lateral.valid || !std::isfinite(lateral.steering_angle_rad)) {
        ROS_WARN_THROTTLE(1.0, "lateral control rejected: %s",
                          lateral.error.c_str());
        if (config_.lateral_controller == kMpcController) {
          publishSafe("INVALID_MPC_CONTROL");
        } else if (config_.lateral_controller == kPurePursuitController) {
          publishSafe("NO_VALID_LOOKAHEAD_TARGET");
        } else if (config_.lateral_controller == kStanleyController) {
          publishSafe("INVALID_STANLEY_CONTROL");
        } else {
          publishSafe("INVALID_HYBRID_CONTROL");
        }
        return;
      }
      const HeadingErrorSpeedLimit heading_speed_limit =
          computeHeadingErrorSpeedLimit(
              lateral.heading_error_rad, speed_plan.target_speed_mps,
              config_.heading_error_speed);
      if (!heading_speed_limit.valid) {
        publishSafe("INVALID_HEADING_ERROR_SPEED_LIMIT");
        return;
      }
      speed_plan.target_speed_mps =
          std::min(speed_plan.target_speed_mps,
                   heading_speed_limit.speed_limit_mps);
      LongitudinalMpcResult longitudinal_mpc_result;
      bool longitudinal_mpc_fallback_active = false;
      LongitudinalCommand longitudinal;
      if (config_.longitudinal_controller == kMpcController) {
        longitudinal_mpc_result = longitudinal_mpc_.update(
            speed_plan.target_speed_mps, speed_mps, dt_sec);
        if (longitudinal_mpc_result.valid) {
          longitudinal.accel = longitudinal_mpc_result.accel;
          longitudinal.brake = longitudinal_mpc_result.brake;
          longitudinal.state = longitudinal_mpc_result.state;
        } else if (config_.longitudinal_mpc_fallback_to_pid) {
          longitudinal_mpc_fallback_active = true;
          longitudinal =
              pid_.update(speed_plan.target_speed_mps, speed_mps, dt_sec);
          ROS_WARN_THROTTLE(1.0, "longitudinal MPC failed; PID fallback active: %s",
                            longitudinal_mpc_result.error.c_str());
        } else {
          publishSafe("INVALID_LONGITUDINAL_MPC_OUTPUT");
          return;
        }
      } else {
        longitudinal =
            pid_.update(speed_plan.target_speed_mps, speed_mps, dt_sec);
      }
      if (!std::isfinite(longitudinal.accel) ||
          !std::isfinite(longitudinal.brake) || longitudinal.accel < 0.0 ||
          longitudinal.brake < 0.0 ||
          (longitudinal.accel > 0.0 && longitudinal.brake > 0.0)) {
        publishSafe("INVALID_LONGITUDINAL_OUTPUT");
        return;
      }

      const ros::Time stamp = ros::Time::now();
      morai_msgs::CtrlCmd output;
      output.longlCmdType = 1;
      output.accel = static_cast<float>(longitudinal.accel);
      output.brake = static_cast<float>(longitudinal.brake);
      output.steering = static_cast<float>(
          lateral.steering_angle_rad /
          config_.pure_pursuit.maximum_steering_angle_rad);
      publisher_.publish(output);
      previous_steering_angle_rad_ = lateral.steering_angle_rad;

      if (lateral.has_tracking_target) {
        geometry_msgs::PointStamped lookahead;
        lookahead.header.stamp = stamp;
        lookahead.header.frame_id = config_.expected_velocity_frame_id;
        lookahead.point.x = lateral.target.x;
        lookahead.point.y = lateral.target.y;
        lookahead.point.z = 0.0;
        lookahead_point_publisher_.publish(lookahead);
      }
      if (lateral.has_stanley_projection) {
        geometry_msgs::PointStamped projection;
        projection.header.stamp = stamp;
        projection.header.frame_id = config_.expected_velocity_frame_id;
        projection.point.x = lateral.stanley_projection.x;
        projection.point.y = lateral.stanley_projection.y;
        projection.point.z = 0.0;
        stanley_projection_point_publisher_.publish(projection);
      }
      if (config_.lateral_controller == kMpcController && lateral.valid) {
        geometry_msgs::PointStamped projection;
        projection.header.stamp = stamp;
        projection.header.frame_id = config_.expected_velocity_frame_id;
        projection.point.x = lateral.mpc_projection.x;
        projection.point.y = lateral.mpc_projection.y;
        projection.point.z = 0.0;
        mpc_projection_point_publisher_.publish(projection);
        nav_msgs::Path compensated_path;
        compensated_path.header.stamp = stamp;
        compensated_path.header.frame_id = config_.expected_velocity_frame_id;
        compensated_path.poses.reserve(lateral.mpc_compensated_path.size());
        for (const morai_mpc::Point2d& point : lateral.mpc_compensated_path) {
          geometry_msgs::PoseStamped pose;
          pose.header = compensated_path.header;
          pose.pose.position.x = point.x;
          pose.pose.position.y = point.y;
          pose.pose.orientation.w = 1.0;
          compensated_path.poses.push_back(pose);
        }
        mpc_compensated_path_publisher_.publish(compensated_path);
      }
      publishControllerStatus(stamp, true, "ACTIVE", speed_mps,
                              yaw_rate_radps, &speed_plan, &wheel_corridor,
                              &lane_speed_limit, &heading_speed_limit, &lateral,
                              output.accel, output.brake,
                              longitudinalStateName(longitudinal.state),
                              config_.longitudinal_controller == kMpcController
                                  ? &longitudinal_mpc_result
                                  : nullptr,
                              longitudinal_mpc_fallback_active);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "controller cycle rejected: %s", error.what());
      publishSafe("CONTROLLER_EXCEPTION");
    } catch (...) {
      ROS_WARN_THROTTLE(1.0, "controller cycle rejected: unknown exception");
      publishSafe("UNKNOWN_CONTROLLER_EXCEPTION");
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ControllerConfig config_;
  LongitudinalPid pid_;
  LongitudinalMpc longitudinal_mpc_;
  CurvatureSpeedPlanner curvature_speed_planner_;
  StanleyController stanley_controller_;
  HybridController hybrid_controller_;
  MpcLateralController mpc_controller_;
  ros::Publisher publisher_;
  ros::Publisher controller_status_publisher_;
  ros::Publisher lookahead_point_publisher_;
  ros::Publisher stanley_projection_point_publisher_;
  ros::Publisher mpc_projection_point_publisher_;
  ros::Publisher mpc_compensated_path_publisher_;
  message_filters::Subscriber<nav_msgs::Path> path_subscriber_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> ego_subscriber_;
  message_filters::Synchronizer<InputSyncPolicy> input_synchronizer_;
  ros::Subscriber vehicle_status_subscriber_;
  double lateral_velocity_mps_{std::numeric_limits<double>::quiet_NaN()};
  ros::WallTimer timer_;
  nav_msgs::Path::ConstPtr latest_path_;
  geometry_msgs::PoseStamped::ConstPtr latest_ego_;
  interfaces::CompetitionVehicleStatus::ConstPtr
      latest_vehicle_status_;
  ros::SteadyTime synchronized_input_receipt_time_;
  ros::SteadyTime vehicle_status_receipt_time_;
  bool has_vehicle_status_{false};
  double filtered_velocity_x_mps_{0.0};
  ros::SteadyTime last_timer_time_;
  bool has_last_timer_time_{false};
  double previous_steering_angle_rad_{0.0};
};

}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_tracking_controller_node");
  try {
    morai_path_tracking::PathTrackingControllerNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start path tracking controller: %s", error.what());
    return 1;
  } catch (...) {
    ROS_FATAL("failed to start path tracking controller: unknown exception");
    return 1;
  }
  return 0;
}
