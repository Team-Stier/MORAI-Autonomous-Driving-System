#include <gtest/gtest.h>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/integrated_dynamic.hpp>
#include <path_planner/libraries/checkpoints.hpp>
#include <path_planner/parameters/integrated_dynamic.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <cmath>
#include <fstream>
#include <memory>
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

// Section 6 inside the dynamic zone by default: route station 480 m, heading north at 25 km/h.
struct Scene {
  path_planner::SharedMap map{path_planner::loadSharedMapParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/shared_map.yaml", kPackage))};
  path_planner::IntegratedDynamic core{path_planner::loadIntegratedDynamicParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/integrated_dynamic.yaml", kPackage)), map};
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  sensor_msgs::Imu imu;
  tf2_msgs::TFMessage tf;
  path_planner::Route route;
  ros::Time stamp{100.0};
  double yaw_rad = 0;
  std::size_t next_checkpoint = 6;

  Scene() { place(-60.41, 10.0, std::atan2(29.88, -0.165), 6.9, 6); }

  // Puts the vehicle at (x, y) with yaw and speed; checkpoints from `next` (0-based row) remain.
  void place(double x, double y, double yaw, double speed_mps, std::size_t next) {
    yaw_rad = yaw;
    next_checkpoint = next;
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
    const path_planner::Checkpoints remaining(all.begin() + next, all.end());
    interfaces::Objects none;
    none.header = ego.header;
    path_planner::GlobalPath global(map);
    route = std::get<5>(global.process(ego, competition, none, tf, imu, remaining));
  }

  void advance(double seconds) {
    stamp += ros::Duration(seconds);
    ego.header.stamp = stamp;
  }

  // Object at `ahead_m` along and `left_m` across the ego heading, moving at (vx, vy) in map.
  interfaces::Object object(double ahead_m, double left_m, double vx, double vy, double size_m) const {
    interfaces::Object o;
    o.id = 7;
    o.label = "unknown";
    o.position.x = ego.pose.pose.position.x + ahead_m * std::cos(yaw_rad) - left_m * std::sin(yaw_rad);
    o.position.y = ego.pose.pose.position.y + ahead_m * std::sin(yaw_rad) + left_m * std::cos(yaw_rad);
    o.bbox_pose.position = o.position;
    o.bbox_pose.orientation = ego.pose.pose.orientation;
    o.bbox_size.x = o.bbox_size.y = size_m;
    o.bbox_size.z = 1.7;
    o.velocity.x = vx;
    o.velocity.y = vy;
    o.velocity_available = true;
    return o;
  }

  interfaces::Path plan(const std::vector<interfaces::Object>& list) {
    interfaces::Objects objects;
    objects.header = ego.header;
    objects.objects = list;
    path_planner::PredictedObjects predictions;
    for (std::size_t i = 0; i < predictions.size(); ++i) {
      predictions[i] = objects;
      for (auto& o : predictions[i].objects) {
        o.position.x += o.velocity.x * 0.5 * (i + 1);
        o.position.y += o.velocity.y * 0.5 * (i + 1);
        o.bbox_pose.position = o.position;
      }
    }
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

TEST(IntegratedDynamic, RouteExistsInDynamicZone) {
  Scene scene;
  ASSERT_TRUE(scene.route);
  EXPECT_FALSE(scene.route->empty());
}

TEST(IntegratedDynamic, DrivesThroughEmptyZone) {
  Scene scene;
  const auto path = scene.plan({});
  EXPECT_STRNE(scene.core.planningOutcome(), "dynamic-zone-stop");
  EXPECT_GT(maximumSpeed(path), 1.0);
}

TEST(IntegratedDynamic, HoldsForMovingBodyInCorridor) {
  Scene scene;
  // A walker on our centreline 7 m ahead, crossing left at 1.2 m/s.
  const double vx = -1.2 * std::sin(scene.yaw_rad), vy = 1.2 * std::cos(scene.yaw_rad);
  const auto path = scene.plan({scene.object(7.0, 0.0, vx, vy, 0.6)});
  EXPECT_STREQ(scene.core.planningOutcome(), "dynamic-zone-stop");
  ASSERT_EQ(path.points.size(), 2u);
  EXPECT_EQ(maximumSpeed(path), 0.0);
}

TEST(IntegratedDynamic, DoesNotHoldForWalkerLeavingCorridor) {
  Scene scene;
  // Clear of our footprint (1.5 m left) and walking further left: already passed.
  const double vx = -1.2 * std::sin(scene.yaw_rad), vy = 1.2 * std::cos(scene.yaw_rad);
  const auto path = scene.plan({scene.object(10.0, 1.5, vx, vy, 0.6)});
  EXPECT_STRNE(scene.core.planningOutcome(), "dynamic-zone-stop");
  EXPECT_GT(maximumSpeed(path), 1.0);
}

TEST(IntegratedDynamic, DoesNotHoldForFutureConflictOutsideStoppingEnvelope) {
  Scene scene;
  // Reaches our lane only after we have passed its station; the solver times it.
  const double vx = -1.2 * std::sin(scene.yaw_rad), vy = 1.2 * std::cos(scene.yaw_rad);
  const auto path = scene.plan({scene.object(16.0, -4.5, vx, vy, 0.6)});
  EXPECT_STRNE(scene.core.planningOutcome(), "dynamic-zone-stop");
  EXPECT_GT(maximumSpeed(path), 1.0);
}

TEST(IntegratedDynamic, StationaryBodyIsLeftToSolver) {
  Scene scene;
  const auto path = scene.plan({scene.object(9.0, 0.0, 0.0, 0.0, 0.8)});
  EXPECT_STRNE(scene.core.planningOutcome(), "dynamic-zone-stop");
  EXPECT_FALSE(path.points.empty());
}

TEST(IntegratedDynamic, FirstCallWhileDrivingSection14) {
  Scene scene;
  // Section 13 -> 14 takeover: a fresh Core whose first call is at 58 km/h mid-section.
  scene.place(69.497, -424.011, -1.7285, 16.0, 14);
  ASSERT_TRUE(scene.route);
  const auto path = scene.plan({});
  EXPECT_STRNE(scene.core.planningOutcome(), "dynamic-zone-stop");
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LT(startOffset(path, scene.ego), 1.0);
  EXPECT_GT(maximumSpeed(path), 10.0);
}

TEST(IntegratedDynamic, FirstCallWhileDrivingSection4) {
  Scene scene;
  // Section 3 -> 4 takeover at 31 km/h, 9 m before the 13 m mouth narrows to the 3 m painted lane.
  scene.place(-84.2, -238.2, 1.0647, 8.7, 4);
  ASSERT_TRUE(scene.route);
  const auto path = scene.plan({});
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LT(startOffset(path, scene.ego), 1.0);
  // Still moving 15 m ahead, inside the 3 m lane.
  const double c = std::cos(scene.yaw_rad), s = std::sin(scene.yaw_rad);
  bool passed = false;
  for (const auto& point : path.points) {
    const double ahead = (point.pose.pose.position.x - scene.ego.pose.pose.position.x) * c +
                         (point.pose.pose.position.y - scene.ego.pose.pose.position.y) * s;
    if (ahead < 15.0) continue;
    passed = true;
    EXPECT_GT(point.velocity, 1.0);
    break;
  }
  EXPECT_TRUE(passed);
}

TEST(IntegratedDynamic, ReplansAfterCheckpointReturn) {
  Scene scene;
  ASSERT_GT(scene.plan({}).points.size(), 2u);
  // A checkpoint return moves the vehicle 40 m back along the road, standing still.
  scene.advance(1.0);
  scene.place(-60.173, -31.510, 1.5764, 0.0, 6);
  const auto path = scene.plan({});
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LT(startOffset(path, scene.ego), 1.0);
  EXPECT_GT(maximumSpeed(path), 1.0);
}

TEST(IntegratedDynamic, JumpReplansLikeAFreshCore) {
  Scene scene;
  ASSERT_GT(scene.plan({}).points.size(), 2u);
  // 0.1 s later the vehicle is 8 m further on: a jump, not driving.
  // After a jump the output equals that of a Core that never saw the earlier call.
  scene.advance(0.1);
  const double yaw = scene.yaw_rad;
  scene.place(-60.41 + 8.0 * std::cos(yaw), 10.0 + 8.0 * std::sin(yaw), yaw, 6.9, 6);
  const auto after_jump = scene.plan({});
  Scene fresh;
  fresh.stamp = scene.stamp;
  fresh.place(scene.ego.pose.pose.position.x, scene.ego.pose.pose.position.y, yaw, 6.9, 6);
  const auto reference = fresh.plan({});
  ASSERT_EQ(after_jump.points.size(), reference.points.size());
  for (std::size_t i = 0; i < reference.points.size(); ++i) {
    EXPECT_NEAR(after_jump.points[i].pose.pose.position.x, reference.points[i].pose.pose.position.x, 1e-6);
    EXPECT_NEAR(after_jump.points[i].pose.pose.position.y, reference.points[i].pose.pose.position.y, 1e-6);
    EXPECT_NEAR(after_jump.points[i].velocity, reference.points[i].velocity, 1e-6);
  }
}

namespace {
// Largest lateral distance of the timed path from the straight section 6 lane (x = -60.4 m).
double laneOffset(const interfaces::Path& path) {
  double offset = 0;
  for (const auto& point : path.points) offset = std::max(offset, std::abs(point.pose.pose.position.x + 60.4));
  return offset;
}
}  // namespace

// A walker first seen standing on the lane 18 m ahead at 29 km/h, before its motion is
// measured: the plan slows in the lane instead of steering around it.
TEST(IntegratedDynamic, UnmeasuredWalkerInZoneIsNotSteeredAround) {
  Scene scene;
  scene.place(-60.39, -3.07, 1.573, 7.94, 6);
  auto walker = scene.object(18.1, 0.0, 0.0, 0.0, 0.4);
  walker.velocity_available = false;
  const auto path = scene.plan({walker});
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_LT(laneOffset(path), 0.5);
  EXPECT_LT(path.points.back().velocity, 7.94);
}

// The same body measured standing still is still avoided by the solver's geometry.
TEST(IntegratedDynamic, MeasuredStandingBodyMayBeSteeredAround) {
  Scene scene;
  scene.place(-60.39, -3.07, 1.573, 7.94, 6);
  const auto path = scene.plan({scene.object(18.1, 0.0, 0.0, 0.0, 0.4)});
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_GT(laneOffset(path), 1.0);
}

// Stopped at the section 2 checkpoint (start of the radius-13 m left curve) with the body
// turned 0.25 rad toward the outer edge, as after a curb strike or a return: the plan drives on.
TEST(IntegratedDynamic, StoppedOffHeadingInCurveDrivesOn) {
  Scene scene;
  scene.place(-96.34, -364.89, 1.118 - 0.25, 0.0, 2);
  const auto path = scene.plan({});
  EXPECT_GT(maximumSpeed(path), 1.0);
}
