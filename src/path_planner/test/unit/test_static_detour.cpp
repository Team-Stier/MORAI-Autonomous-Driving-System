#include <gtest/gtest.h>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/integrated_path.hpp>
#include <path_planner/cores/static_detour.hpp>
#include <path_planner/libraries/checkpoints.hpp>
#include <path_planner/parameters/integrated_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/static_detour.hpp>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace {
const std::string kPackage = std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/test/unit/"));

std::string contents(const std::string& path) {
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

// Section 4: the route lane runs north past stop line 6254 (y = -161.3) into
// lanelet 130870 (x -61.7..-58.1, centre -59.82 at the sample box); the
// southbound lane 130856 lies to its left. The approach pose is on the lane centre.
constexpr double kLaneCentreX = -59.82;
constexpr double kApproachX = -61.36, kApproachY = -175.0, kApproachYaw = 1.4425;

// Throttled warnings read the ROS clock; this test has no node.
struct RosClock {
  RosClock() { ros::Time::init(); }
};

struct Scene {
  RosClock clock;
  path_planner::SharedMap map{path_planner::loadSharedMapParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/shared_map.yaml", kPackage))};
  path_planner::StaticDetour core{path_planner::loadStaticDetourParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/static_detour.yaml", kPackage)), map};
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  sensor_msgs::Imu imu;
  tf2_msgs::TFMessage tf;
  path_planner::Route route;
  ros::Time stamp{100.0};
  interfaces::Objects objects;
  // Velocities of moving bodies; their forecasts advance 0.5 s per slot.
  path_planner::PredictedObjects predictions;

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
    imu.linear_acceleration.z = 9.81;
    const auto all = path_planner::readCheckpoints(contents(kPackage + "/../map/checkpoints.csv"));
    const path_planner::Checkpoints remaining(all.begin() + 4, all.end());
    interfaces::Objects none;
    none.header = ego.header;
    path_planner::GlobalPath global(map);
    route = std::get<5>(global.process(ego, competition, none, tf, imu, remaining));
  }

  void add(double x, double y, double yaw_rad, double length_m, double width_m,
           double vx_mps = 0, double vy_mps = 0, bool moving = false) {
    interfaces::Object object;
    object.id = static_cast<std::uint32_t>(objects.objects.size() + 1);
    object.label = "unknown";
    object.position.x = x;
    object.position.y = y;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.z = std::sin(yaw_rad / 2);
    object.bbox_pose.orientation.w = std::cos(yaw_rad / 2);
    object.bbox_size.x = length_m;
    object.bbox_size.y = width_m;
    object.bbox_size.z = 1.5;
    object.velocity_available = moving;
    object.velocity.x = vx_mps;
    object.velocity.y = vy_mps;
    objects.objects.push_back(object);
  }

  // One planning call on a new LiDAR scan 0.3 s after the previous one.
  interfaces::Path plan() {
    stamp += ros::Duration(0.3);
    ego.header.stamp = stamp;
    objects.header = ego.header;
    for (std::size_t i = 0; i < predictions.size(); ++i) {
      predictions[i] = objects;
      for (auto& object : predictions[i].objects) {
        if (!object.velocity_available) continue;
        object.position.x += object.velocity.x * 0.5 * (i + 1);
        object.position.y += object.velocity.y * 0.5 * (i + 1);
        object.bbox_pose.position = object.position;
      }
    }
    return core.process(ego, competition, objects, tf, imu, route, predictions, stamp);
  }
};

// x of the lateral reference where it crosses y (NaN when it does not reach y).
double crossingX(const interfaces::Path& path, double y) {
  const auto& points = path.lateral_reference;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto& a = points[i - 1].position;
    const auto& b = points[i].position;
    if ((a.y - y) * (b.y - y) > 0 || a.y == b.y) continue;
    return a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x);
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double maximumSpeed(const interfaces::Path& path) {
  double speed = 0;
  for (const auto& point : path.points) speed = std::max(speed, point.velocity);
  return speed;
}

double maximumLeftShift(const interfaces::Path& path, double y_min, double y_max) {
  double shift = -std::numeric_limits<double>::infinity();
  for (const auto& pose : path.lateral_reference)
    if (pose.position.y >= y_min && pose.position.y <= y_max) shift = std::max(shift, kLaneCentreX - pose.position.x);
  return shift;
}
}  // namespace

// The official sample box stands 0.85 m right of the lane centre past the stop line.
TEST(StaticDetour, BypassesInLaneBoxOnTheLeftAndReturnsBeforeCheckpoint4) {
  Scene scene;
  scene.place(kApproachX, kApproachY, kApproachYaw, 8.0);
  ASSERT_TRUE(scene.route);
  scene.add(-59.05, -143.32, kApproachYaw, 0.56, 0.56);
  interfaces::Path path;
  for (int scan = 0; scan < 3; ++scan) path = scene.plan();
  ASSERT_FALSE(path.lateral_reference.empty());
  // Body edge 1.08 m left of the rear-axle line clears the box edge (x = -59.33) by the 0.5 m gap.
  const double beside = crossingX(path, -143.32);
  ASSERT_TRUE(std::isfinite(beside));
  EXPECT_LT(beside, -59.33 - 0.5 - 1.08 + 0.05);
  // Back inside the route lane before the checkpoint-4 radius.
  const double after = crossingX(path, -122.0);
  ASSERT_TRUE(std::isfinite(after));
  EXPECT_NEAR(after, kLaneCentreX, 0.5);
  // The bypass keeps moving instead of stopping in front of the box.
  EXPECT_GT(maximumSpeed(path), 2.0);
  EXPECT_LE(maximumSpeed(path), 30.0 / 3.6 + 1e-6);
}

// A vehicle prop seen from behind is a wide, shallow face; the bypass holds
// its offset over the assumed 5 m body before returning.
TEST(StaticDetour, HoldsOffsetAlongUnseenVehicleBody) {
  Scene scene;
  scene.place(kApproachX, kApproachY, kApproachYaw, 8.0);
  scene.add(-59.3, -148.0, kApproachYaw, 0.4, 1.9);
  interfaces::Path path;
  for (int scan = 0; scan < 3; ++scan) path = scene.plan();
  ASSERT_FALSE(path.lateral_reference.empty());
  // Face right edge-to-left edge: x -60.25..-58.35; the body extends to y = -143.2.
  EXPECT_GT(maximumLeftShift(path, -148.0, -143.5), 0.0);
  for (const double y : {-148.0, -146.0, -144.0}) {
    const double x = crossingX(path, y);
    ASSERT_TRUE(std::isfinite(x));
    EXPECT_LT(x, -60.25 - 0.5 - 1.08 + 0.05) << "y=" << y;
  }
}

// Without obstacles the Core plans exactly like integrated_path.
TEST(StaticDetour, MatchesIntegratedPathWithoutObstacles) {
  Scene scene;
  scene.place(kApproachX, kApproachY, kApproachYaw, 8.0);
  path_planner::IntegratedPath base{path_planner::loadIntegratedPathParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/integrated_path.yaml", kPackage)), scene.map};
  for (int scan = 0; scan < 2; ++scan) {
    const auto detour = scene.plan();
    const auto reference = base.process(scene.ego, scene.competition, scene.objects, scene.tf, scene.imu,
                                        scene.route, scene.predictions, scene.stamp);
    ASSERT_EQ(detour.points.size(), reference.points.size());
    ASSERT_EQ(detour.lateral_reference.size(), reference.lateral_reference.size());
    for (std::size_t i = 0; i < reference.points.size(); ++i) {
      EXPECT_DOUBLE_EQ(detour.points[i].pose.pose.position.x, reference.points[i].pose.pose.position.x);
      EXPECT_DOUBLE_EQ(detour.points[i].pose.pose.position.y, reference.points[i].pose.pose.position.y);
      EXPECT_DOUBLE_EQ(detour.points[i].velocity, reference.points[i].velocity);
    }
  }
}

// An obstacle before stop line 6254, in the single lane between solid lines,
// is outside the bypass zone: the vehicle stays in its lane.
TEST(StaticDetour, KeepsLaneForObstacleBeforeStopLine) {
  Scene scene;
  scene.place(kApproachX, kApproachY, kApproachYaw, 6.0);
  scene.add(-60.3, -163.5, kApproachYaw, 4.6, 1.9);
  interfaces::Path path;
  for (int scan = 0; scan < 3; ++scan) path = scene.plan();
  ASSERT_GT(path.points.size(), 1u);
  for (const auto& pose : path.lateral_reference)
    if (pose.position.y < -161.3) EXPECT_LT(std::abs(pose.position.x - (-60.3)), 1.0);
  EXPECT_LT(path.points.back().velocity, 0.5);
}

// A southbound car about to pass through the bypass corridor defers the bypass.
TEST(StaticDetour, WaitsForOncomingTrafficBeforeBypassing) {
  Scene scene;
  scene.place(kApproachX, kApproachY, kApproachYaw, 6.0);
  scene.add(-59.05, -143.32, kApproachYaw, 0.56, 0.56);
  scene.add(-63.6, -128.0, -kApproachYaw, 4.6, 1.9, 0.0, -8.0, true);
  interfaces::Path path;
  for (int scan = 0; scan < 3; ++scan) path = scene.plan();
  ASSERT_GT(path.points.size(), 1u);
  for (const auto& pose : path.lateral_reference)
    if (pose.position.y > -158.0 && pose.position.y < -135.0) EXPECT_GT(pose.position.x, -61.7);
}

// A checkpoint return 6 m back forgets the remembered box: with nothing seen
// yet from there, the plan keeps the lane past the box position.
TEST(StaticDetour, CheckpointReturnClearsBypass) {
  Scene scene;
  scene.place(kApproachX, kApproachY, kApproachYaw, 8.0);
  scene.add(-59.05, -143.32, kApproachYaw, 0.56, 0.56);
  interfaces::Path path;
  for (int scan = 0; scan < 3; ++scan) path = scene.plan();
  ASSERT_LT(crossingX(path, -143.32), kLaneCentreX - 1.0);
  scene.objects.objects.clear();
  scene.place(-62.229, -181.0, 1.3973, 0.0);
  path = scene.plan();
  ASSERT_GT(path.points.size(), 1u);
  const double beside = crossingX(path, -143.32);
  ASSERT_TRUE(std::isfinite(beside));
  EXPECT_NEAR(beside, kLaneCentreX, 0.5);
  EXPECT_GT(maximumSpeed(path), 1.0);
}
