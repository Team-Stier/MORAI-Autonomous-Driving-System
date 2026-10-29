#include <gtest/gtest.h>
#include <control/cores/morai_mpc.hpp>
#include <algorithm>
#include <cmath>
#include <deque>

namespace {
constexpr double kDt = 1.0 / 30.0;
constexpr double kMaximumSteeringRad = 40.0 * M_PI / 180.0;

control::autoware_ai_mpc::Parameters parameters(double minimum_model_speed_mps) {
  control::autoware_ai_mpc::Parameters result;
  result.lateral.morai_straight_steer_change_scale = 1.5;
  result.lateral.morai_minimum_model_speed_mps = minimum_model_speed_mps;
  return result;
}

geometry_msgs::Quaternion yawQuaternion(double yaw_rad) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(yaw_rad / 2);
  q.w = std::cos(yaw_rad / 2);
  return q;
}

// Lane centre along map +x at y = 0, expressed in base_link of the vehicle pose.
interfaces::Path centreLine(double x_m, double y_m, double yaw_rad, double speed_mps, double stamp_sec) {
  interfaces::Path path;
  path.header.stamp = ros::Time(stamp_sec);
  const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
  for (int i = -1; i <= 60; ++i) {
    const double dx = std::floor(x_m) + i - x_m, dy = -y_m;
    geometry_msgs::Pose pose;
    pose.position.x = c * dx + s * dy;
    pose.position.y = -s * dx + c * dy;
    pose.orientation = yawQuaternion(-yaw_rad);
    interfaces::PathPoint point;
    point.pose.pose = pose;
    point.velocity = speed_mps;
    path.points.push_back(point);
    path.lateral_reference.push_back(pose);
  }
  return path;
}

// Counter-clockwise circle of radius 12 m around (0, 12), expressed in base_link.
interfaces::Path circle(double x_m, double y_m, double yaw_rad, double speed_mps, double stamp_sec) {
  constexpr double kRadius = 12.0;
  interfaces::Path path;
  path.header.stamp = ros::Time(stamp_sec);
  const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
  const double here = std::atan2(x_m, kRadius - y_m) * kRadius;
  for (int i = -1; i <= 30; ++i) {
    const double arc = std::floor(here) + i, angle = arc / kRadius;
    const double dx = kRadius * std::sin(angle) - x_m, dy = kRadius - kRadius * std::cos(angle) - y_m;
    geometry_msgs::Pose pose;
    pose.position.x = c * dx + s * dy;
    pose.position.y = -s * dx + c * dy;
    pose.orientation = yawQuaternion(angle - yaw_rad);
    interfaces::PathPoint point;
    point.pose.pose = pose;
    point.velocity = speed_mps;
    path.points.push_back(point);
    path.lateral_reference.push_back(pose);
  }
  return path;
}

struct Run {
  double maximum_stopping_steering = 0;  // normalized, while slowing and standing
  double departure_yaw_change_rad = 0;   // first 1.5 s after the start
  double maximum_left_m = -1e9;          // after the start
  double final_offset_m = 0;
};

// 0.4 m right of the centre heading 3.7 deg right, brakes from 4.3 m/s to a stop in 0.5 s, waits 1.5 s and starts.
template <class Core>
Run stopAndGo(Core& core) {
  double x = 0, y = -0.4, yaw = -3.7 * M_PI / 180.0, speed = 4.3, wheel = 0;
  std::deque<double> delayed(5, 0.0);  // 1/6 s steering delay
  Run run;
  double yaw_at_start = 0;
  for (int k = 0; k <= 12 * 30; ++k) {
    const double t = k * kDt;
    const double stop = 0.5, start = 2.0;
    if (t < stop) speed = 4.3 * (stop - t) / stop;
    else if (t < start) speed = 0;
    else speed = std::min(4.5, 1.5 * (t - start));
    const double target = t < start ? 0.0 : 4.5;
    nav_msgs::Odometry ego;
    ego.pose.pose.position.x = x;
    ego.pose.pose.position.y = y;
    ego.pose.pose.orientation = yawQuaternion(yaw);
    ego.twist.twist.linear.x = speed;
    interfaces::CompetitionVehicleStatus status;
    status.velocity_x_mps = speed;
    status.signed_velocity_mps = speed;
    const double steering = core.process(centreLine(x, y, yaw, target, 100 + t), ego, status).steering;
    if (t < start) run.maximum_stopping_steering = std::max(run.maximum_stopping_steering, std::abs(steering));
    if (std::abs(t - start) < kDt / 2) yaw_at_start = yaw;
    if (std::abs(t - start - 1.5) < kDt / 2) run.departure_yaw_change_rad = std::abs(yaw - yaw_at_start);
    if (t >= start) run.maximum_left_m = std::max(run.maximum_left_m, y);
    delayed.push_back(steering * kMaximumSteeringRad * 0.77);
    wheel += (delayed.front() - wheel) * kDt / (0.05 + kDt);
    delayed.pop_front();
    x += speed * std::cos(yaw) * kDt;
    y += speed * std::sin(yaw) * kDt;
    yaw += speed / 3.0 * std::tan(wheel) * kDt;
  }
  run.final_offset_m = std::abs(y);
  return run;
}
}  // namespace

TEST(MoraiMpc, HardStopKeepsSteeringNearDrivingCommand) {
  // With the former 0.5 m/s model floor this stop wound the command up to 0.98 and turned 8.7 deg on the start.
  control::MoraiMpc core(parameters(3.0));
  const auto run = stopAndGo(core);
  EXPECT_LT(run.maximum_stopping_steering, 0.45);
  EXPECT_LT(run.departure_yaw_change_rad, 6.0 * M_PI / 180.0);
  EXPECT_LT(run.maximum_left_m, 0.1);
  EXPECT_LT(run.final_offset_m, 0.05);
}

TEST(MoraiMpc, SlowCurveStaysOnPath) {
  control::MoraiMpc core(parameters(3.0));
  const double speed = 1.5;
  double x = 0, y = -0.3, yaw = 0, wheel = 0, maximum_error_m = 0;
  std::deque<double> delayed(5, 0.0);
  for (int k = 0; k <= 12 * 30; ++k) {
    const double t = k * kDt;
    nav_msgs::Odometry ego;
    ego.pose.pose.position.x = x;
    ego.pose.pose.position.y = y;
    ego.pose.pose.orientation = yawQuaternion(yaw);
    ego.twist.twist.linear.x = speed;
    interfaces::CompetitionVehicleStatus status;
    status.velocity_x_mps = speed;
    const double steering = core.process(circle(x, y, yaw, speed, 100 + t), ego, status).steering;
    if (t >= 4.0) maximum_error_m = std::max(maximum_error_m, std::abs(std::hypot(x, y - 12.0) - 12.0));
    delayed.push_back(steering * kMaximumSteeringRad * 0.77);
    wheel += (delayed.front() - wheel) * kDt / (0.05 + kDt);
    delayed.pop_front();
    x += speed * std::cos(yaw) * kDt;
    y += speed * std::sin(yaw) * kDt;
    yaw += speed / 3.0 * std::tan(wheel) * kDt;
  }
  EXPECT_LT(maximum_error_m, 0.1);
}

TEST(MoraiMpc, FloorLeavesFasterDrivingUnchanged) {
  control::MoraiMpc reference(parameters(0.5)), floored(parameters(3.0));
  double x = 0, y = -0.4, yaw = -3.7 * M_PI / 180.0;
  for (int k = 0; k < 90; ++k) {
    const double t = k * kDt, speed = 6.0;
    nav_msgs::Odometry ego;
    ego.pose.pose.position.x = x;
    ego.pose.pose.position.y = y;
    ego.pose.pose.orientation = yawQuaternion(yaw);
    ego.twist.twist.linear.x = speed;
    interfaces::CompetitionVehicleStatus status;
    status.velocity_x_mps = speed;
    const auto path = centreLine(x, y, yaw, speed, 100 + t);
    const double a = reference.process(path, ego, status).steering;
    const double b = floored.process(path, ego, status).steering;
    EXPECT_DOUBLE_EQ(a, b);
    x += speed * std::cos(yaw) * kDt;
    y += speed * std::sin(yaw) * kDt;
    yaw += speed / 3.0 * std::tan(a * kMaximumSteeringRad * 0.77) * kDt;
  }
}

TEST(MoraiMpc, FirstUseStartsFromMeasuredSteering) {
  // Switching in from integrated_mpc mid-curve: the wheels already turn left by the curve's angle.
  control::MoraiMpc core(parameters(3.0));
  const double wheel_rad = std::atan(3.0 / 12.0);
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w = 1.0;
  ego.twist.twist.linear.x = 6.0;
  interfaces::CompetitionVehicleStatus status;
  status.velocity_x_mps = 6.0;
  status.steering_angle_rad = -wheel_rad;  // MORAI reports wheel steering with the opposite ROS yaw sign.
  const double steering = core.process(circle(0.0, 0.0, 0.0, 6.0, 100.0), ego, status).steering;
  // Without the resync the rate limit holds the first command within 2 deg of zero.
  EXPECT_GT(steering * kMaximumSteeringRad, 0.2);
}

TEST(MoraiMpc, ReselectionStartsFromMeasuredSteering) {
  control::MoraiMpc core(parameters(3.0));
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w = 1.0;
  ego.twist.twist.linear.x = 6.0;
  interfaces::CompetitionVehicleStatus status;
  status.velocity_x_mps = 6.0;
  EXPECT_NEAR(core.process(centreLine(0.0, 0.0, 0.0, 6.0, 100.0), ego, status).steering, 0.0, 0.02);
  // Another Core steered for 5 s; the vehicle is now in a left curve.
  status.steering_angle_rad = -std::atan(3.0 / 12.0);
  const double steering = core.process(circle(0.0, 0.0, 0.0, 6.0, 105.0), ego, status).steering;
  EXPECT_GT(steering * kMaximumSteeringRad, 0.2);
}
