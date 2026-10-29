#include <gtest/gtest.h>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/integrated_static.hpp>
#include <path_planner/libraries/checkpoints.hpp>
#include <path_planner/parameters/integrated_static.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
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

// Section 4: a 3.0 m lane between solid lines from CP3 (-80.0, -230.4) to CP4 (-59.9, -124.6).
struct Scene {
  path_planner::SharedMap map{path_planner::loadSharedMapParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/shared_map.yaml", kPackage))};
  path_planner::IntegratedStatic core{path_planner::loadIntegratedStaticParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/integrated_static.yaml", kPackage)), map};
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  sensor_msgs::Imu imu;
  tf2_msgs::TFMessage tf;
  interfaces::Objects objects;
  path_planner::PredictedObjects predictions;
  path_planner::Route route;
  ros::Time stamp{100.0};

  // Puts the vehicle at (x, y) with yaw and speed; checkpoints from row 4 (CP4) remain.
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
    objects.header = ego.header;
    for (auto& future : predictions) future.header = ego.header;
    const auto all = path_planner::readCheckpoints(contents(kPackage + "/../map/checkpoints.csv"));
    const path_planner::Checkpoints remaining(all.begin() + 4, all.end());
    interfaces::Objects none;
    none.header = ego.header;
    path_planner::GlobalPath global(map);
    route = std::get<5>(global.process(ego, competition, none, tf, imu, remaining));
  }
  // A static box measured by the detector, repeated in every prediction slice.
  void addBox(double x, double y, double size_m, double north_mps = 0) {
    interfaces::Object object;
    object.velocity_available = north_mps > 0;
    object.velocity.y = north_mps;
    object.id = 7;
    object.label = "unknown";
    object.position.x = x;
    object.position.y = y;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.w = 1;
    object.bbox_size.x = object.bbox_size.y = size_m;
    object.bbox_size.z = 0.9;
    objects.objects.push_back(object);
    for (auto& future : predictions) future.objects = objects.objects;
  }
  path_planner::Message plan() {
    return core.process(ego, competition, objects, tf, imu, route, predictions, stamp);
  }
  // Distance from the route lane centerline, which runs midway between the two painted lines.
  double offset(double x, double y) const {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& lane : *route)
      best = std::min(best, lanelet::geometry::distance2d(lane.centerline2d(), lanelet::BasicPoint2d(x, y)));
    return best;
  }
};

double speedAhead(const path_planner::Message& path, const nav_msgs::Odometry& ego, double distance_m) {
  const auto& start = ego.pose.pose.position;
  for (const auto& point : path.points)
    if (std::hypot(point.pose.pose.position.x - start.x, point.pose.pose.position.y - start.y) >= distance_m)
      return point.velocity;
  return -1;
}

TEST(IntegratedStatic, FirstCallWhileEnteringSection4) {
  Scene scene;
  // The section 3 Core drove up to here; this Core's first call happens at speed.
  scene.place(-84.2, -238.2, 61.0 * M_PI / 180, 8.7);
  const auto path = scene.plan();
  ASSERT_GT(path.points.size(), 2u);
  const auto& first = path.points.front().pose.pose.position;
  EXPECT_LT(std::hypot(first.x + 84.2, first.y + 238.2), 1.0);
  EXPECT_GT(speedAhead(path, scene.ego, 15.0), 1.0);  // inside the 3.0 m lane
}

TEST(IntegratedStatic, CheckpointReturnRestartsPlanning) {
  Scene scene;
  scene.place(-60.0, -150.0, M_PI / 2, 5.0);
  ASSERT_GT(scene.plan().points.size(), 2u);
  // Returned to CP3 at rest, 85 m behind the last call.
  scene.stamp = ros::Time(110.0);
  scene.place(-79.9685, -230.3943, 61.0977 * M_PI / 180, 0.0);
  const auto path = scene.plan();
  ASSERT_GT(path.points.size(), 2u);
  const auto& first = path.points.front().pose.pose.position;
  EXPECT_LT(std::hypot(first.x + 79.9685, first.y + 230.3943), 1.0);
  double fastest = 0;
  for (const auto& point : path.points) fastest = std::max(fastest, static_cast<double>(point.velocity));
  EXPECT_GT(fastest, 1.0);
}

TEST(IntegratedStatic, PassesInLaneBoxAndReturnsWithinShortCrossing) {
  Scene scene;
  // Scenario box 40100004 (0.56 m) right of the lane centre, 1.5-17 m past stop line 6254.
  scene.place(-59.9, -158.0, M_PI / 2, 5.0);
  scene.addBox(-59.05, -143.32, 0.56);
  const auto path = scene.plan();
  ASSERT_GT(path.points.size(), 2u);
  ASSERT_FALSE(path.lateral_reference.empty());
  double farthest = -1e9;
  for (const auto& point : path.points) farthest = std::max(farthest, point.pose.pose.position.y);
  EXPECT_GT(farthest, -143.32);  // the timed path reaches beside the box
  // The body (half width 1.08 m) is over a painted line while its centre is
  // more than 0.42 m off the 3.0 m lane centre; keep that below 3 s at 5.56 m/s.
  double crossing = 0, returned_y = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.lateral_reference.size(); ++i) {
    const auto& a = path.lateral_reference[i - 1].position;
    const auto& b = path.lateral_reference[i].position;
    if (scene.offset(b.x, b.y) > 0.42) crossing += std::hypot(b.x - a.x, b.y - a.y);
    if (b.y > -143.32 && scene.offset(b.x, b.y) < 0.3) returned_y = std::min(returned_y, b.y);
  }
  EXPECT_GT(crossing, 0.0);
  EXPECT_LT(crossing, 16.0);
  EXPECT_LT(returned_y, -143.32 + 20.0);  // back in the lane within 20 m past the box
}

TEST(IntegratedStatic, MovingBodyInLaneReleasesNoPaint) {
  Scene scene;
  // A slower leading body in the 3.0 m lane: no line crossing to overtake it.
  scene.place(-59.9, -158.0, M_PI / 2, 5.0);
  scene.addBox(-59.05, -143.32, 0.56, 3.0);
  const auto path = scene.plan();
  ASSERT_GT(path.points.size(), 1u);
  for (const auto& pose : path.lateral_reference) EXPECT_LT(scene.offset(pose.position.x, pose.position.y), 0.42);
  for (const auto& point : path.points)
    EXPECT_LT(scene.offset(point.pose.pose.position.x, point.pose.pose.position.y), 0.42);
}
}  // namespace
