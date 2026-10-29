#include <gtest/gtest.h>
#include <path_planner/cores/roundabout_approach.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <cmath>
#include <memory>

namespace {
using path_planner::RoundaboutApproach;
using roundabout_approach_detail::path_planner::RoundaboutApproachParameters;

// Straight one-lane road along +x: lanelets 1 (0..40 m), 2 (40..80 m), 3 (80..160 m),
// roundabout yield line across the lane at x = 50 m.
struct Fixture {
  Fixture() {
    std::vector<lanelet::Point3d> left, right;
    for (int i = 0; i <= 32; ++i) {
      left.emplace_back(1000 + i, 5.0 * i, 1.75, 0);
      right.emplace_back(2000 + i, 5.0 * i, -1.75, 0);
    }
    const auto lane = [&](lanelet::Id id, int begin, int end) {
      lanelet::Lanelet value(id,
          lanelet::LineString3d(100 * id + 1, lanelet::Points3d(left.begin() + begin, left.begin() + end + 1)),
          lanelet::LineString3d(100 * id + 2, lanelet::Points3d(right.begin() + begin, right.begin() + end + 1)));
      value.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
      value.setAttribute(lanelet::AttributeName::Location, lanelet::AttributeValueString::Urban);
      value.setAttribute(lanelet::AttributeName::OneWay, true);
      return value;
    };
    lanes = {lane(1, 0, 8), lane(2, 8, 16), lane(3, 16, 32)};
    map.map = std::shared_ptr<lanelet::LaneletMap>(lanelet::utils::createMap(lanes).release());
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    const auto graph = lanelet::routing::RoutingGraph::build(*map.map, *rules);
    route = graph->shortestPath(lanes[0], lanes[2]);
    geometry_msgs::Point a, b;
    a.x = b.x = 50;
    a.y = -1.75;
    b.y = 1.75;
    parameters.yield_line = {a, b};
    parameters.local_path_length_m = 55;
    parameters.avoidance_speed_mps = 6;
  }
  lanelet::Lanelets lanes;
  path_planner::SharedMap map{path_planner::SharedMapParameters{}};
  path_planner::Route route;
  RoundaboutApproachParameters parameters;
};

nav_msgs::Odometry ego(double time_sec) {
  nav_msgs::Odometry odometry;
  odometry.header.stamp = ros::Time(time_sec);
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 10;
  odometry.pose.pose.orientation.w = 1;
  odometry.twist.twist.linear.x = 10;
  return odometry;
}

interfaces::CompetitionVehicleStatus moving() {
  interfaces::CompetitionVehicleStatus status;
  status.velocity_x_mps = 10;
  return status;
}

// A car crossing the road toward +y at 8 m/s, at y = 0 after 1.25 s.
interfaces::Objects crossing(double x_m, double time_sec) {
  interfaces::Objects objects;
  objects.header.stamp = ros::Time(time_sec);
  objects.header.frame_id = "map";
  interfaces::Object object;
  object.id = 3;
  object.position.x = x_m;
  object.position.y = -10 + 8 * (time_sec - 100);
  object.velocity.y = 8;
  object.velocity_available = true;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.z = std::sin(M_PI / 4);
  object.bbox_pose.orientation.w = std::cos(M_PI / 4);
  object.bbox_size.x = 4.5;
  object.bbox_size.y = 1.9;
  object.bbox_size.z = 1.5;
  objects.objects.push_back(object);
  return objects;
}

// Drive 0.5 s of 30 Hz ticks so the crossing car is confirmed as moving.
interfaces::Path drive(RoundaboutApproach& core, const path_planner::Route& route, double x_m) {
  interfaces::Path path;
  for (int tick = 0; tick <= 15; ++tick) {
    const double time = 100 + tick / 30.0;
    path = core.process(ego(time), moving(), crossing(x_m, time), route);
  }
  return path;
}
}  // namespace

TEST(RoundaboutApproach, IgnoresTrafficBeyondTheYieldLine) {
  Fixture fixture;
  ASSERT_TRUE(fixture.route);
  RoundaboutApproach core(fixture.parameters, fixture.map);
  const auto path = drive(core, fixture.route, 60);
  ASSERT_GE(path.points.size(), 2u);
  for (const auto& point : path.points) EXPECT_TRUE(std::isnan(point.velocity));
}

TEST(RoundaboutApproach, StopsForTrafficBeforeTheYieldLine) {
  Fixture fixture;
  ASSERT_TRUE(fixture.route);
  RoundaboutApproach core(fixture.parameters, fixture.map);
  const auto path = drive(core, fixture.route, 30);
  ASSERT_GE(path.points.size(), 2u);
  for (const auto& point : path.points) EXPECT_DOUBLE_EQ(point.velocity, 0.0);
}
