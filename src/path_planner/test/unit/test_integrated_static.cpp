#include <gtest/gtest.h>
#include <path_planner/parameters/integrated_static.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace {
using path_planner::IntegratedStatic;

// Section parameters with the listed keys replaced; the S4/S5 paint asset is dropped
// because the synthetic road below is far from it.
node_pipeline::ParamReader parameters(const std::map<std::string, std::string>& overrides) {
  std::ifstream file(std::string(NP_PACKAGE_DIR) + "/config/path_planner/parameters/integrated_static.yaml");
  std::ostringstream yaml;
  std::string line;
  while (std::getline(file, line)) {
    const auto colon = line.find(':');
    const std::string key = colon == std::string::npos ? "" : line.substr(0, colon);
    if (key == "physical_boundaries_file" || overrides.count(key)) continue;
    yaml << line << "\n";
  }
  for (const auto& entry : overrides) yaml << entry.first << ": " << entry.second << "\n";
  return node_pipeline::ParamReader::fromString(yaml.str(), NP_PACKAGE_DIR);
}

lanelet::LineString3d line(lanelet::Id id, double y, const char* subtype) {
  lanelet::LineString3d result(id, {lanelet::Point3d(id * 10 + 1, -50, y, 0), lanelet::Point3d(id * 10 + 2, 250, y, 0)});
  result.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  result.attributes()[lanelet::AttributeName::Subtype] = subtype;
  return result;
}

// Two same-direction 3.5 m lanes along +x: the route lane y [-1.75, 1.75] and a free lane on its left.
struct Road {
  Road() {
    const auto right = line(1, -1.75, lanelet::AttributeValueString::Solid);
    const auto middle = line(2, 1.75, lanelet::AttributeValueString::Dashed);
    const auto left = line(3, 5.25, lanelet::AttributeValueString::Solid);
    lanelet::Lanelet route_lane(100, middle, right), free_lane(101, left, middle);
    for (auto* lane : {&route_lane, &free_lane}) {
      lane->attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
      lane->attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
    }
    map = lanelet::utils::createMap({route_lane, free_lane});
    route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{map->laneletLayer.get(100)});
  }
  std::shared_ptr<lanelet::LaneletMap> map;
  path_planner::Route route;
};

struct Inputs {
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus status;
  interfaces::Objects objects;
  tf2_msgs::TFMessage tf;
  sensor_msgs::Imu imu;
  path_planner::PredictedObjects predictions;
  ros::Time stamp{100.0};
  Inputs(double x, double speed) {
    ego.header.stamp = stamp;
    ego.header.frame_id = "map";
    ego.pose.pose.position.x = x;
    ego.pose.pose.orientation.w = 1;
    ego.twist.twist.linear.x = speed;
    status.velocity_x_mps = speed;
    imu.orientation.w = 1;
    imu.linear_acceleration.z = 9.81;
    objects.header = ego.header;
  }
  void add(double x, double y, double length, double width) {
    interfaces::Object object;
    object.id = static_cast<std::uint32_t>(objects.objects.size() + 1);
    object.label = "unknown";
    object.position.x = x;
    object.position.y = y;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.w = 1;
    object.bbox_size.x = length;
    object.bbox_size.y = width;
    object.bbox_size.z = 1.5;
    objects.objects.push_back(object);
    for (auto& future : predictions) {
      future.header = objects.header;
      future.objects = objects.objects;
    }
  }
};

path_planner::Message plan(const std::map<std::string, std::string>& overrides, Inputs& inputs) {
  Road road;
  path_planner::SharedMap shared({road.map});
  IntegratedStatic core(path_planner::loadIntegratedStaticParameters(parameters(overrides)), shared);
  return core.process(inputs.ego, inputs.status, inputs.objects, inputs.tf, inputs.imu, road.route,
                      inputs.predictions, inputs.stamp);
}

double maximumSpeed(const path_planner::Message& path) {
  double result = 0;
  for (const auto& point : path.points) result = std::max(result, static_cast<double>(point.velocity));
  return result;
}

// Rear-axle poses; Ioniq5 front 3.845 m, rear 0.79 m, half width 1.08 m.
bool bodyOverlaps(const path_planner::Message& path, double min_x, double max_x, double min_y, double max_y) {
  for (const auto& point : path.points) {
    const auto& pose = point.pose.pose;
    const double yaw = 2 * std::atan2(pose.orientation.z, pose.orientation.w);
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (double u = -0.79; u <= 3.845; u += 0.1)
      for (double v = -1.08; v <= 1.08; v += 0.1) {
        const double x = pose.position.x + c * u - s * v, y = pose.position.y + s * u + c * v;
        if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) return true;
      }
  }
  return false;
}

TEST(IntegratedStatic, ObjectBehindRearAxleHoldsCapOnlyWithoutAheadFilter) {
  const std::map<std::string, std::string> common{{"avoidance_speed_mps", "3.0"}};
  auto capped = common;
  capped["integrated_passing_ahead_only"] = "false";
  auto released = common;
  released["integrated_passing_ahead_only"] = "true";
  Inputs before(0, 5.0), after(0, 5.0);
  // Already passed: 6 m behind the rear axle, beside the route lane.
  before.add(-6.0, 3.5, 0.8, 0.8);
  after.add(-6.0, 3.5, 0.8, 0.8);
  const auto capped_path = plan(capped, before);
  const auto released_path = plan(released, after);
  ASSERT_GT(capped_path.points.size(), 2u);
  ASSERT_GT(released_path.points.size(), 2u);
  EXPECT_LE(capped_path.points.back().velocity, 3.0 + 1e-6);
  EXPECT_GT(maximumSpeed(released_path), 5.0);
}

TEST(IntegratedStatic, ObjectBesideVehicleKeepsCapWithAheadFilter) {
  Inputs inputs(0, 3.0);
  inputs.add(1.5, 3.5, 0.8, 0.8);
  const auto path = plan({{"avoidance_speed_mps", "3.0"}, {"integrated_passing_ahead_only", "true"}}, inputs);
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_LE(maximumSpeed(path), 3.0 + 1e-6);
}

TEST(IntegratedStatic, VehicleFacePriorClearsHiddenBody) {
  Inputs inputs(0, 5.5);
  // Rear face of a 5 m car whose body spans x [15, 20] in the route lane, right of centre.
  inputs.add(15.1, -0.5, 0.2, 1.8);
  const auto path = plan({{"integrated_vehicle_length_prior_m", "5.0"}}, inputs);
  ASSERT_GT(path.points.size(), 2u);
  ASSERT_GT(path.points.back().pose.pose.position.x, 21.0);
  EXPECT_FALSE(bodyOverlaps(path, 15.0, 20.0, -1.4, 0.4));
}
}  // namespace
