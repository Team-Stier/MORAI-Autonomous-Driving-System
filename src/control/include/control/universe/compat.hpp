#pragma once
// Plain calculation types replacing ROS 2 messages; no ROS 2 runtime dependency.
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <ros/time.h>
#include <boost/optional.hpp>
#include <control/universe/expected.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
namespace control { namespace universe {
template<class T> inline T clamp(const T& v, const T& lo, const T& hi) {
  return std::max(lo, std::min(v, hi));
}
}}
namespace autoware_planning_msgs { namespace msg {
struct TrajectoryPoint {
  geometry_msgs::Pose pose;
  double longitudinal_velocity_mps = 0.0, lateral_velocity_mps = 0.0;
  double acceleration_mps2 = 0.0, heading_rate_rps = 0.0;
  double front_wheel_angle_rad = 0.0, rear_wheel_angle_rad = 0.0;
  ros::Duration time_from_start;
};
struct Trajectory { std_msgs::Header header; std::vector<TrajectoryPoint> points; };
}}
namespace autoware_control_msgs { namespace msg {
struct Lateral { ros::Time stamp; float steering_tire_angle = 0.0f, steering_tire_rotation_rate = 0.0f; };
struct Longitudinal { ros::Time stamp; float velocity = 0.0f, acceleration = 0.0f; };
}}
namespace autoware_vehicle_msgs { namespace msg {
struct SteeringReport { ros::Time stamp; float steering_tire_angle = 0.0f; };
}}
namespace autoware_internal_debug_msgs { namespace msg {
struct Float32MultiArrayStamped { ros::Time stamp; std::vector<float> data; };
}}
namespace autoware_adapi_v1_msgs { namespace msg {
struct OperationModeState {
  static constexpr int AUTONOMOUS = 2;
  int mode = AUTONOMOUS;
  bool is_autoware_control_enabled = true;
};
}}
namespace autoware { namespace motion { namespace control { namespace trajectory_follower {
struct InputData {
  autoware_planning_msgs::msg::Trajectory current_trajectory;
  nav_msgs::Odometry current_odometry;
  geometry_msgs::AccelWithCovarianceStamped current_accel;
  autoware_adapi_v1_msgs::msg::OperationModeState current_operation_mode;
};
struct LateralHorizon { double time_step_ms = 0.0; std::vector<autoware_control_msgs::msg::Lateral> controls; };
struct LongitudinalHorizon { double time_step_ms = 0.0; std::vector<autoware_control_msgs::msg::Longitudinal> controls; };
struct LongitudinalOutput { autoware_control_msgs::msg::Longitudinal control_cmd; LongitudinalHorizon control_cmd_horizon; };
}}}}
