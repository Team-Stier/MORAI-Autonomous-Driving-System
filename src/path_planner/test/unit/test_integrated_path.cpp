#include <gtest/gtest.h>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/integrated_path.hpp>
#include <path_planner/libraries/checkpoints.hpp>
#include <path_planner/parameters/integrated_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {
const std::string kPackage = std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/test/unit/"));

std::string contents(const std::string& path) {
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

// Section 3 (route station 186-237 m) with no objects.
struct Scene {
  path_planner::SharedMap map{path_planner::loadSharedMapParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/shared_map.yaml", kPackage))};
  path_planner::IntegratedPath core{path_planner::loadIntegratedPathParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/integrated_path.yaml", kPackage)), map};
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  sensor_msgs::Imu imu;
  tf2_msgs::TFMessage tf;
  path_planner::Route route;
  ros::Time stamp{100.0};

  // Puts the vehicle at (x, y) with yaw and speed; checkpoint rows 4-16 remain.
  void place(double x, double y, double yaw_rad, double speed_mps) {
    ego.header.stamp = stamp;
    ego.header.frame_id = "map";
    ego.pose.pose.position.x = x;
    ego.pose.pose.position.y = y;
    ego.pose.pose.orientation.z = std::sin(yaw_rad / 2);
    ego.pose.pose.orientation.w = std::cos(yaw_rad / 2);
    ego.twist.twist.linear.x = speed_mps;
    competition.velocity_x_mps = speed_mps;
    imu.orientation.w = 1;
    const auto all = path_planner::readCheckpoints(contents(kPackage + "/../map/checkpoints.csv"));
    const path_planner::Checkpoints remaining(all.begin() + 3, all.end());
    interfaces::Objects none;
    none.header = ego.header;
    path_planner::GlobalPath global(map);
    route = std::get<5>(global.process(ego, competition, none, tf, imu, remaining));
  }

  interfaces::Path plan(const std::vector<interfaces::Object>& list = {}) {
    interfaces::Objects objects;
    objects.header = ego.header;
    objects.objects = list;
    path_planner::PredictedObjects predictions;
    predictions.fill(objects);
    return core.process(ego, competition, objects, tf, imu, route, predictions, stamp);
  }
};

double startOffset(const interfaces::Path& path, const nav_msgs::Odometry& ego) {
  const auto& a = path.points.front().pose.pose.position;
  return std::hypot(a.x - ego.pose.pose.position.x, a.y - ego.pose.pose.position.y);
}

double maximumSpeed(const interfaces::Path& path) {
  double speed = 0;
  for (const auto& point : path.points) speed = std::max(speed, point.velocity);
  return speed;
}
}  // namespace

TEST(IntegratedPath, FirstCallWhileDriving) {
  Scene scene;
  scene.place(-85.993, -241.306, 1.0664, 10.0);
  ASSERT_TRUE(scene.route);
  const auto path = scene.plan();
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LT(startOffset(path, scene.ego), 1.0);
  EXPECT_GT(maximumSpeed(path), 5.0);
}

TEST(IntegratedPath, ReplansAfterCheckpointReturn) {
  Scene scene;
  scene.place(-85.993, -241.306, 1.0664, 10.0);
  ASSERT_GT(scene.plan().points.size(), 2u);
  // A checkpoint return moves the vehicle 30 m back along the road, standing still.
  scene.stamp += ros::Duration(1.0);
  scene.place(-100.597, -267.710, 1.0668, 0.0);
  const auto path = scene.plan();
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LT(startOffset(path, scene.ego), 1.0);
  EXPECT_GT(maximumSpeed(path), 1.0);
}

TEST(IntegratedPath, JumpReplansLikeAFreshCore) {
  Scene scene;
  scene.place(-100.597, -267.710, 1.0668, 10.0);
  ASSERT_GT(scene.plan().points.size(), 2u);
  // 0.1 s later the vehicle is 8 m further on: a jump, not driving.
  // After a jump the output equals that of a Core that never saw the earlier call.
  scene.stamp += ros::Duration(0.1);
  const double x = -100.597 + 8.0 * std::cos(1.0668), y = -267.710 + 8.0 * std::sin(1.0668);
  scene.place(x, y, 1.0668, 10.0);
  const auto after_jump = scene.plan();
  Scene fresh;
  fresh.stamp = scene.stamp;
  fresh.place(x, y, 1.0668, 10.0);
  const auto reference = fresh.plan();
  ASSERT_EQ(after_jump.points.size(), reference.points.size());
  for (std::size_t i = 0; i < reference.points.size(); ++i) {
    EXPECT_NEAR(after_jump.points[i].pose.pose.position.x, reference.points[i].pose.pose.position.x, 1e-6);
    EXPECT_NEAR(after_jump.points[i].pose.pose.position.y, reference.points[i].pose.pose.position.y, 1e-6);
    EXPECT_NEAR(after_jump.points[i].velocity, reference.points[i].velocity, 1e-6);
  }
}

namespace {
// A standing box at (x, y) fitted to one flat face: zero width, measured still.
interfaces::Object flatBox(double x, double y, double yaw_rad) {
  interfaces::Object o;
  o.id = 3;
  o.label = "unknown";
  o.position.x = x;
  o.position.y = y;
  o.bbox_pose.position = o.position;
  o.bbox_pose.orientation.z = std::sin(yaw_rad / 2);
  o.bbox_pose.orientation.w = std::cos(yaw_rad / 2);
  o.bbox_size.x = 1.13;
  o.bbox_size.y = 0.0;
  o.bbox_size.z = 1.0;
  o.velocity_available = true;
  return o;
}
}  // namespace

// A zero-width box 15 m ahead beside the lane is planned around, not a calculation-domain hold.
TEST(IntegratedPath, ZeroWidthBoxKeepsPlanning) {
  ros::Time::init();  // a failed plan logs through ROS_WARN_THROTTLE
  Scene scene;
  scene.place(-85.993, -241.306, 1.0664, 10.0);
  const double x = -85.993 + 15 * std::cos(1.0664) - 2.5 * std::sin(1.0664);
  const double y = -241.306 + 15 * std::sin(1.0664) + 2.5 * std::cos(1.0664);
  const auto path = scene.plan({flatBox(x, y, 1.0664)});
  EXPECT_STRNE(scene.core.planningOutcome(), "calculation-domain");
  EXPECT_GT(maximumSpeed(path), 5.0);
}

// Standing still with the brake released and a negative IMU reading, the plan still starts.
TEST(IntegratedPath, StandstillNegativeImuStarts) {
  ros::Time::init();  // a failed plan logs through ROS_WARN_THROTTLE
  Scene scene;
  scene.place(-100.597, -267.710, 1.0668, 0.02);
  scene.competition.brake = 0;
  scene.imu.linear_acceleration.x = -0.6;
  const auto path = scene.plan();
  EXPECT_GT(maximumSpeed(path), 1.0);
}
