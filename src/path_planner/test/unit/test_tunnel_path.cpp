#include <gtest/gtest.h>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/tunnel_path.hpp>
#include <path_planner/libraries/checkpoints.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/tunnel_path.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
const std::string kPackage = std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/test/unit/"));

// Tunnel axis (localization tunnel_wall geometry): station along the bore, lateral to its left.
constexpr double kOriginX = -23.5, kOriginY = -519.2000122070312, kAxisYaw = -2.6434409392630127;

std::string contents(const std::string& path) {
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

// Section 15 inside the tunnel, in our lane (lateral -1.7 m), heading along the bore toward the exit.
struct Scene {
  path_planner::SharedMap map{path_planner::loadSharedMapParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/shared_map.yaml", kPackage))};
  path_planner::TunnelPath core{path_planner::loadTunnelPathParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/tunnel_path.yaml", kPackage)), map};
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  sensor_msgs::Imu imu;
  tf2_msgs::TFMessage tf;
  path_planner::Route route;
  ros::Time stamp{100.0};
  double yaw_rad = kAxisYaw;

  explicit Scene(double speed_mps, double station_m = -40.0) {
    ros::Time::init();
    ego.header.stamp = stamp;
    ego.header.frame_id = "map";
    ego.pose.pose.position.x = kOriginX + station_m * std::cos(kAxisYaw) + 1.7 * std::sin(kAxisYaw);
    ego.pose.pose.position.y = kOriginY + station_m * std::sin(kAxisYaw) - 1.7 * std::cos(kAxisYaw);
    ego.pose.pose.orientation.z = std::sin(yaw_rad / 2);
    ego.pose.pose.orientation.w = std::cos(yaw_rad / 2);
    ego.twist.twist.linear.x = speed_mps;
    competition.velocity_x_mps = speed_mps;
    imu.orientation.w = 1;
    // Section 15: the tunnel exit (checkpoint row 15) remains.
    const auto all = path_planner::readCheckpoints(contents(kPackage + "/../map/checkpoints.csv"));
    const path_planner::Checkpoints remaining(all.begin() + 15, all.end());
    interfaces::Objects none;
    none.header = ego.header;
    path_planner::GlobalPath global(map);
    route = std::get<5>(global.process(ego, competition, none, tf, imu, remaining));
  }

  // Stationary body at `ahead_m` along and `left_m` across our heading.
  interfaces::Object body(double ahead_m, double left_m, double length_m, double width_m, double height_m) const {
    interfaces::Object o;
    o.id = 7;
    o.label = "unknown";
    o.position.x = ego.pose.pose.position.x + ahead_m * std::cos(yaw_rad) - left_m * std::sin(yaw_rad);
    o.position.y = ego.pose.pose.position.y + ahead_m * std::sin(yaw_rad) + left_m * std::cos(yaw_rad);
    o.bbox_pose.position = o.position;
    o.bbox_pose.orientation = ego.pose.pose.orientation;
    o.bbox_size.x = length_m;
    o.bbox_size.y = width_m;
    o.bbox_size.z = height_m;
    o.velocity_available = true;
    return o;
  }

  interfaces::Path plan(const std::vector<interfaces::Object>& list) {
    interfaces::Objects objects;
    objects.header = ego.header;
    objects.objects = list;
    path_planner::PredictedObjects predictions;
    for (auto& slice : predictions) slice = objects;
    return core.process(ego, competition, objects, tf, imu, route, predictions, stamp);
  }
};

double maximumSpeed(const interfaces::Path& path) {
  double speed = 0;
  for (const auto& point : path.points) speed = std::max(speed, point.velocity);
  return speed;
}

// Smallest distance from a point to the vehicle footprint (rear axle origin) over the planned poses.
double footprintClearance(const interfaces::Path& path, double x, double y) {
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto& point : path.points) {
    const auto& q = point.pose.pose.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const double dx = x - point.pose.pose.position.x, dy = y - point.pose.pose.position.y;
    const double along = std::cos(yaw) * dx + std::sin(yaw) * dy, across = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double outside_along = std::max({0.0, -0.79 - along, along - 3.845008});
    const double outside_across = std::max(0.0, std::abs(across) - 1.080418);
    clearance = std::min(clearance, std::hypot(outside_along, outside_across));
  }
  return clearance;
}
}  // namespace

TEST(TunnelPath, RouteExistsInTunnel) {
  Scene scene(12.0);
  ASSERT_TRUE(scene.route);
  EXPECT_FALSE(scene.route->empty());
}

TEST(TunnelPath, CapsSpeedThroughTunnel) {
  Scene scene(10.0);
  const auto path = scene.plan({});
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_GT(maximumSpeed(path), 10.5);
  EXPECT_LE(maximumSpeed(path), 40.0 / 3.6 + 1e-6);
}

TEST(TunnelPath, ApproachesWalkwayPedestrianAtStoppableSpeed) {
  // A standing pedestrian 22 m ahead on the right walkway, 2.3 m right of our lane centre.
  // Stop at 2.0 m/s^2 short of its near face (22 - 0.25 - 3.845 m) from at most sqrt(4.5^2 + 4 * 17.905) m/s.
  Scene scene(9.0);
  const double ceiling = std::sqrt(4.5 * 4.5 + 2 * 2.0 * (22.0 - 0.25 - 3.845008));
  const auto path = scene.plan({scene.body(22.0, -2.3, 0.5, 0.5, 1.7)});
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LE(maximumSpeed(path), ceiling + 1e-6);
  Scene empty(9.0);
  EXPECT_GT(maximumSpeed(empty.plan({})), ceiling + 1.0);
}

TEST(TunnelPath, StopsForPedestrianWalkingIntoCorridor) {
  // 9 m ahead, 0.8 m right of our centre, walking left at 1.5 m/s.
  Scene scene(9.0);
  auto walker = scene.body(9.0, -0.8, 0.5, 0.5, 1.7);
  walker.velocity.x = -1.5 * std::sin(scene.yaw_rad);
  walker.velocity.y = 1.5 * std::cos(scene.yaw_rad);
  const auto path = scene.plan({walker});
  EXPECT_STREQ(scene.core.planningOutcome(), "dynamic-zone-stop");
  ASSERT_EQ(path.points.size(), 2u);
  EXPECT_EQ(maximumSpeed(path), 0.0);
}

TEST(TunnelPath, DoesNotHoldForBodyMovingAlongRoad) {
  // A box fragment in our lane 9 m ahead whose box drifts toward us at 0.9 m/s as more of it comes into view.
  Scene scene(9.0);
  auto fragment = scene.body(9.0, -0.3, 0.5, 0.2, 2.4);
  fragment.velocity.x = -0.9 * std::cos(scene.yaw_rad);
  fragment.velocity.y = -0.9 * std::sin(scene.yaw_rad);
  scene.plan({fragment});
  EXPECT_STRNE(scene.core.planningOutcome(), "dynamic-zone-stop");
}

TEST(TunnelPath, PlansSlowFragmentsAsStandingStill) {
  // A box fragment in our lane 15 m ahead whose box drifts 0.6 m/s along the road plans exactly like a standing one
  // (not as a slow car to follow).
  Scene drifting(8.0), standing(8.0);
  auto fragment = drifting.body(15.0, -0.6, 2.0, 0.4, 2.4);
  fragment.velocity.x = 0.6 * std::cos(drifting.yaw_rad);
  fragment.velocity.y = 0.6 * std::sin(drifting.yaw_rad);
  const auto moved = drifting.plan({fragment});
  const auto still = standing.plan({standing.body(15.0, -0.6, 2.0, 0.4, 2.4)});
  ASSERT_GT(still.points.size(), 2u);
  ASSERT_EQ(moved.points.size(), still.points.size());
  for (std::size_t i = 0; i < still.points.size(); ++i) {
    EXPECT_NEAR(moved.points[i].pose.pose.position.x, still.points[i].pose.pose.position.x, 1e-6);
    EXPECT_NEAR(moved.points[i].pose.pose.position.y, still.points[i].pose.pose.position.y, 1e-6);
    EXPECT_NEAR(moved.points[i].velocity, still.points[i].velocity, 1e-6);
  }
}

TEST(TunnelPath, DropsSidewaysMotionOfVehicleSizedBody) {
  // A box 5 x 2 m 15 m ahead half in our lane whose outline drifts 1.2 m/s across the road plans like a standing one.
  Scene drifting(8.0), standing(8.0);
  auto box = drifting.body(15.0, -1.8, 5.0, 2.0, 2.4);
  box.velocity.x = -1.2 * std::sin(drifting.yaw_rad);
  box.velocity.y = 1.2 * std::cos(drifting.yaw_rad);
  const auto moved = drifting.plan({box});
  const auto still = standing.plan({standing.body(15.0, -1.8, 5.0, 2.0, 2.4)});
  ASSERT_GT(still.points.size(), 2u);
  ASSERT_EQ(moved.points.size(), still.points.size());
  for (std::size_t i = 0; i < still.points.size(); ++i) {
    EXPECT_NEAR(moved.points[i].pose.pose.position.x, still.points[i].pose.pose.position.x, 1e-6);
    EXPECT_NEAR(moved.points[i].pose.pose.position.y, still.points[i].pose.pose.position.y, 1e-6);
    EXPECT_NEAR(moved.points[i].velocity, still.points[i].velocity, 1e-6);
  }
}

TEST(TunnelPath, PlansAroundLongZeroWidthFace) {
  // One flat face 2 m long and 0 m deep at our right lane edge 12 m ahead: the solver still gets a positive footprint.
  Scene scene(6.0);
  const auto path = scene.plan({scene.body(12.0, -1.6, 2.0, 0.0, 1.2)});
  EXPECT_STRNE(scene.core.planningOutcome(), "calculation-domain");
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_GT(maximumSpeed(path), 1.0);
}

TEST(TunnelPath, StartsFromRestWithNegativeAccelerationReading) {
  // Standing still with no brake applied, a small negative IMU reading is not a deceleration into reverse.
  Scene scene(0.0);
  scene.competition.brake = 0.0;
  scene.imu.linear_acceleration.x = -0.4;
  const auto path = scene.plan({});
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_GT(maximumSpeed(path), 1.0);
}

TEST(TunnelPath, PassesPedestrianNoFasterThanPassSpeedUntilBumperIsBy) {
  Scene beside(4.0);
  const auto passing = beside.plan({beside.body(4.5, -2.3, 0.5, 0.5, 1.7)});
  ASSERT_GT(passing.points.size(), 2u);
  EXPECT_LE(maximumSpeed(passing), std::sqrt(4.5 * 4.5 + 2 * 2.0 * (4.5 - 0.25 - 3.845008)) + 1e-6);
  // Once our bumper is past its near face the body no longer limits the speed.
  Scene passed(4.0);
  const auto after = passed.plan({passed.body(3.0, -2.3, 0.5, 0.5, 1.7)});
  ASSERT_GT(after.points.size(), 2u);
  EXPECT_GT(maximumSpeed(after), 6.5);
}

TEST(TunnelPath, KeepsLimitingForPedestrianBrieflyLostFromView) {
  // Seen once 22 m ahead on the right walkway, then missing from the scans: the ceiling holds for 2 s, then lapses.
  Scene scene(9.0);
  const double ceiling = std::sqrt(4.5 * 4.5 + 2 * 2.0 * (22.0 - 0.25 - 3.845008));
  ASSERT_GT(scene.plan({scene.body(22.0, -2.3, 0.5, 0.5, 1.7)}).points.size(), 2u);
  scene.stamp += ros::Duration(0.5);
  scene.ego.header.stamp = scene.stamp;
  const auto lost = scene.plan({});
  ASSERT_GT(lost.points.size(), 2u);
  EXPECT_LE(maximumSpeed(lost), ceiling + 1e-6);
  scene.stamp += ros::Duration(2.0);
  scene.ego.header.stamp = scene.stamp;
  EXPECT_GT(maximumSpeed(scene.plan({})), ceiling + 1.0);
}

TEST(TunnelPath, CountsWalkerSmearedAlongItsPathAsPedestrian) {
  // A walker's scan seen 1.8 m long and 0.35 m wide, 22 m ahead on the right walkway.
  Scene scene(9.0);
  const double ceiling = std::sqrt(4.5 * 4.5 + 2 * 2.0 * (22.0 - 0.9 - 3.845008));
  const auto path = scene.plan({scene.body(22.0, -2.3, 1.8, 0.35, 1.7)});
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LE(maximumSpeed(path), ceiling + 1e-6);
}

TEST(TunnelPath, IgnoresLowBodiesAndBodiesBeyondReach) {
  Scene low(9.0);
  EXPECT_GT(maximumSpeed(low.plan({low.body(22.0, -2.3, 0.5, 0.5, 0.2)})), 10.5);
  // Across the oncoming lane on the far walkway: 6 m left of our centre, beyond 1.08 + 3 m.
  Scene far(9.0);
  EXPECT_GT(maximumSpeed(far.plan({far.body(22.0, 6.0, 0.5, 0.5, 1.7)})), 10.5);
  // A car-sized body is not a pedestrian.
  Scene car(9.0);
  EXPECT_GT(maximumSpeed(car.plan({car.body(22.0, -4.0, 4.5, 1.9, 1.5)})), 10.5);
}

TEST(TunnelPath, PlansAroundSmallBodyWithMinimumFootprint) {
  // One LiDAR face of a 0.37 m box: 0.2 x 0.1 m seen 1.48 m right of our centre, 15 m ahead.
  // Its seen face clears our side by 0.35 m; the 0.6 m minimum footprint needs 0.3 m plus the 0.25 m margin.
  Scene scene(10.0);
  const auto box = scene.body(15.0, -1.48, 0.2, 0.1, 0.85);
  const auto path = scene.plan({box});
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_GE(footprintClearance(path, box.position.x, box.position.y), 0.3 + 0.25 - 0.02);
}
