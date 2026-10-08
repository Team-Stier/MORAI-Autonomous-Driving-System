#include <control/parameters/path_tracking.hpp>
#include <control/parameters/safe_stop.hpp>
#include <control/validators/tracking_inputs.hpp>
#include <fstream>
#include <iterator>
#include <gtest/gtest.h>
#include <limits>
namespace {
control::tracking::Parameters parameters() {
  control::tracking::Parameters p;
  p.lateral_controller = "pure_pursuit";
  p.longitudinal_controller = "pid";
  p.mpc.solver = "projected_gradient";
  p.longitudinal_mpc.solver = "projected_gradient";
  p.pid.kp = 1;
  p.pid.ki = 0;
  p.pid.kd = 0;
  return p;
}
interfaces::Path path(double speed, double lateral = 0) {
  interfaces::Path result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  for (double x : {-10.0, 0.0, 10.0, 20.0}) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = x;
    point.pose.pose.position.y = lateral;
    point.pose.pose.orientation.w = 1;
    point.velocity = speed;
    result.points.push_back(point);
  }
  return result;
}
nav_msgs::Odometry ego() {
  nav_msgs::Odometry result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "map";
  result.child_frame_id = "base_link";
  result.pose.pose.orientation.w = 1;
  return result;
}
interfaces::CompetitionVehicleStatus status() {
  interfaces::CompetitionVehicleStatus result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  result.velocity_x_mps = 5;
  return result;
}
TEST(PathTrackingCore, UsesInjectedNearestSpeedAndNormalizedSteering) {
  control::PathTracking core(parameters());
  auto input = path(10, 2);
  input.points[1].velocity = 0;
  const auto command = core.process(input, ego(), status());
  EXPECT_EQ(command.longlCmdType, 1);
  EXPECT_DOUBLE_EQ(command.accel, 0);
  EXPECT_GT(command.brake, 0);
  EXPECT_GT(command.steering, 0);
  EXPECT_LE(command.steering, 1);
}
TEST(PathTrackingCore, RetainsPidStateAcrossCalls) {
  auto p = parameters();
  p.pid.kp = 0;
  p.pid.ki = 0.1;
  control::PathTracking core(p);
  auto input = path(6);
  const auto first = core.process(input, ego(), status());
  input.header.stamp = ros::Time(1, 50000000);
  const auto second = core.process(input, ego(), status());
  EXPECT_GT(second.accel, first.accel);
}
TEST(PathTrackingCore, RunsBothMpcSolversWithoutRosIo) {
  auto p = parameters();
  p.lateral_controller = "mpc";
  p.longitudinal_controller = "mpc";
  control::PathTracking core(p);
  const auto command = core.process(path(6), ego(), status());
  EXPECT_TRUE(std::isfinite(command.steering));
  EXPECT_LE(std::abs(command.steering), 1);
  EXPECT_GE(command.accel, 0);
  EXPECT_GE(command.brake, 0);
  EXPECT_FALSE(command.accel > 0 && command.brake > 0);
}
TEST(PathTrackingParameters, LoadsEveryConfiguredKey) {
  std::ifstream stream(std::string(NP_PACKAGE_DIR) + "/config/control/parameters/path_tracking.yaml");
  ASSERT_TRUE(stream.good());
  const std::string text{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  const auto loaded = control::loadPathTrackingParameters(node_pipeline::ParamReader::fromString(text));
  EXPECT_EQ("mpc", loaded.lateral_controller);
  EXPECT_NEAR(60.0 / 3.6, loaded.longitudinal_mpc.maximum_speed_mps, 1e-12);
  EXPECT_NEAR(40.0 * 0.017453292519943295, loaded.pure_pursuit.maximum_steering_angle_rad, 1e-12);
  EXPECT_THROW(control::loadPathTrackingParameters(node_pipeline::ParamReader::fromString("nominal_dt_sec: 0.05\n")),
               std::invalid_argument);
}
TEST(SafeStop, PublishesConfiguredBrakeWithoutInputs) {
  control::SafeStop core(control::loadSafeStopParameters(node_pipeline::ParamReader::fromString("brake: 0.5\n")));
  const auto command = core.process();
  EXPECT_EQ(command.longlCmdType, 1);
  EXPECT_DOUBLE_EQ(command.accel, 0);
  EXPECT_DOUBLE_EQ(command.steering, 0);
  EXPECT_DOUBLE_EQ(command.brake, 0.5);
  EXPECT_THROW(control::loadSafeStopParameters(node_pipeline::ParamReader::fromString("brake: 0\n")), std::invalid_argument);
}
TEST(TrackingInputs, RejectsDegeneratePathsAndOverspeed) {
  node_pipeline::HealthLimits limits;
  limits.frame = "base_link";
  limits.lower = 0;
  limits.upper = 60.0 / 3.6;
  const control::TrackingInputs validator;
  std::vector<node_pipeline::Kind> issues;
  validator(path(6), limits, issues);
  EXPECT_TRUE(issues.empty());
  validator(path(20), limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>(4, node_pipeline::Kind::OutRangePositive), issues);
  issues.clear();
  auto single = path(6);
  single.points.resize(1);
  validator(single, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>(2, node_pipeline::Kind::Missing), issues);
  issues.clear();
  auto odometry = ego();
  odometry.child_frame_id = "odom";
  validator(odometry, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::FrameMismatch}, issues);
}
}
int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
