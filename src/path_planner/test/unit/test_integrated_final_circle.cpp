#include <gtest/gtest.h>
#include <path_planner/cores/integrated_final_circle.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <cmath>
#include <memory>

namespace {
using path_planner::IntegratedFinalCircle;
using integrated_final_circle_detail::path_planner::IntegratedFinalCircleParameters;

// Straight one-lane road along +x: lanelets 1 (0..40 m), 2 (40..80 m), 3 (80..160 m).
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
    parameters.local_path_length_m = 55;
    parameters.avoidance_speed_mps = 6;
  }
  lanelet::Lanelets lanes;
  path_planner::SharedMap map{path_planner::SharedMapParameters{}};
  path_planner::Route route;
  IntegratedFinalCircleParameters parameters;
};
}  // namespace

// The section handover calls a fresh Core while the vehicle is already moving mid-route.
TEST(IntegratedFinalCircle, FirstCallWhileMovingFollowsTheRoute) {
  Fixture fixture;
  ASSERT_TRUE(fixture.route);
  IntegratedFinalCircle core(fixture.parameters, fixture.map);
  nav_msgs::Odometry ego;
  ego.header.stamp = ros::Time(100);
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = 52;
  ego.pose.pose.position.y = 0.4;
  ego.pose.pose.orientation.w = 1;
  ego.twist.twist.linear.x = 12;
  interfaces::CompetitionVehicleStatus status;
  status.velocity_x_mps = 12;
  interfaces::Objects objects;
  objects.header = ego.header;
  const auto path = core.process(ego, status, objects, fixture.route);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 52, 0.5);
  EXPECT_NEAR(path.points.front().pose.pose.position.y, 0, 0.1);
  EXPECT_GT(path.points.back().pose.pose.position.x, 52 + 40);
  for (const auto& point : path.points) EXPECT_TRUE(std::isnan(point.velocity));
}
