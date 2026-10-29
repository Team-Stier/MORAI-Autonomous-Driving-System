#include <gtest/gtest.h>
#include <path_planner/cores/integrated_highway_approach.hpp>
#include <lanelet2_core/utility/Utilities.h>
#include <cmath>

namespace {
namespace approach = integrated_highway_approach_detail::path_planner;

// One straight southbound lane from y=+50 to y=-250, sampled every metre.
lanelet::LaneletMapPtr southboundLane() {
  lanelet::LineString3d left(lanelet::utils::getId()), right(lanelet::utils::getId());
  for (int y = 50; y >= -250; --y) {
    left.push_back(lanelet::Point3d(lanelet::utils::getId(), 1.75, y, 0));
    right.push_back(lanelet::Point3d(lanelet::utils::getId(), -1.75, y, 0));
  }
  lanelet::Lanelet lane(lanelet::utils::getId(), left, right);
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return lanelet::utils::createMap({lane});
}

approach::PathPlannerMissionParameters parameters() {
  approach::PathPlannerMissionParameters result;
  result.reference_smoothing_enabled = false;
  return result;
}

path_planner::Message step(approach::PathPlannerMission& mission, const path_planner::Route& route, double y,
                           double speed_mps, double stamp_sec, const std::vector<interfaces::Object>& objects = {}) {
  nav_msgs::Odometry ego;
  ego.header.stamp = ros::Time(stamp_sec);
  ego.pose.pose.position.y = y;
  ego.pose.pose.orientation.z = std::sin(-M_PI / 4);
  ego.pose.pose.orientation.w = std::cos(-M_PI / 4);
  ego.twist.twist.linear.x = speed_mps;
  interfaces::CompetitionVehicleStatus competition;
  competition.velocity_x_mps = speed_mps;
  interfaces::Objects observed;
  observed.header.stamp = ego.header.stamp;
  observed.objects = objects;
  return mission.process(ego, competition, observed, route);
}

bool stops(const path_planner::Message& path) {
  for (const auto& point : path.points)
    if (std::isfinite(point.velocity) && point.velocity <= 0) return true;
  return false;
}
}  // namespace

TEST(IntegratedHighwayApproach, FirstCallWhileDrivingPlansFromEgo) {
  auto map = southboundLane();
  approach::PathPlannerMission mission(parameters(), *map);
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{*map->laneletLayer.begin()});

  // Taking over from S9 mid-lane at speed: the lane ahead from the ego, speeds left to velocity_planner.
  const auto path = step(mission, route, -20, 12, 10);
  ASSERT_GT(path.points.size(), 30u);
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 0, 1e-6);
  EXPECT_NEAR(path.points.front().pose.pose.position.y, -20, 1e-6);
  EXPECT_LT(path.points.back().pose.pose.position.y, -60);
  for (const auto& point : path.points) EXPECT_TRUE(std::isnan(point.velocity));
}

TEST(IntegratedHighwayApproach, ClockRewindForgetsEarlierObjects) {
  auto map = southboundLane();
  approach::PathPlannerMission mission(parameters(), *map);
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{*map->laneletLayer.begin()});
  interfaces::Object parked;
  parked.id = 7;
  parked.position.y = parked.bbox_pose.position.y = -35;
  parked.bbox_pose.orientation.w = 1;
  parked.bbox_size.x = 2;
  parked.bbox_size.y = 4;
  parked.bbox_size.z = 1.5;
  parked.velocity_available = true;

  step(mission, route, -20, 0, 100, {parked});
  ASSERT_TRUE(stops(step(mission, route, -20, 0, 100.6, {parked})));  // confirmed stationary on the lane

  // A new lap restarts the clock without that object: the lane is free again.
  const auto path = step(mission, route, -20, 0, 5);
  EXPECT_FALSE(stops(path));
  for (const auto& point : path.points) EXPECT_TRUE(std::isnan(point.velocity));
}
