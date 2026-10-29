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

// Section 6 inside the dynamic zone: route station 480 m, heading north at 25 km/h.
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
  const ros::Time stamp{100.0};
  const double yaw_rad = std::atan2(29.88, -0.165);

  Scene() {
    ego.header.stamp = stamp;
    ego.header.frame_id = "map";
    ego.pose.pose.position.x = -60.41;
    ego.pose.pose.position.y = 10.0;
    ego.pose.pose.orientation.z = std::sin(yaw_rad / 2);
    ego.pose.pose.orientation.w = std::cos(yaw_rad / 2);
    ego.twist.twist.linear.x = 6.9;
    competition.velocity_x_mps = 6.9;
    imu.orientation.w = 1;
    const auto all = path_planner::readCheckpoints(contents(kPackage + "/../map/checkpoints.csv"));
    const path_planner::Checkpoints remaining(all.begin() + 6, all.end());
    interfaces::Objects none;
    none.header = ego.header;
    path_planner::GlobalPath global(map);
    route = std::get<5>(global.process(ego, competition, none, tf, imu, remaining));
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
