#include <gtest/gtest.h>
#include <path_planner/cores/integrated_highway.hpp>
#include <lanelet2_core/utility/Utilities.h>
#include <cmath>

namespace {
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

path_planner::HighwayMissionParameters parameters() {
  path_planner::HighwayMissionParameters result;
  result.reference_smoothing_enabled = false;
  result.mission_end_y_m = 1000;  // follow the given route without mission lane changes
  result.mission_change_count = 1;
  result.entry_speed_mps = 60.0 / 3.6;
  result.exit_speed_mps = 15.0;
  result.highway_end_y_m = -100;
  result.stopping_deceleration_mps2 = 3;
  return result;
}
}  // namespace

TEST(IntegratedHighway, PathPointsPreviewExitBrakingProfile) {
  ros::Time::init();  // the Core's ROS_*_THROTTLE logging reads ros::Time::now() without a node
  auto map = southboundLane();
  path_planner::HighwayMission mission(parameters(), *map);
  nav_msgs::Odometry ego;
  ego.header.stamp = ros::Time(10);
  ego.pose.pose.position.y = -80;
  ego.pose.pose.orientation.z = std::sin(-M_PI / 4);
  ego.pose.pose.orientation.w = std::cos(-M_PI / 4);
  interfaces::CompetitionVehicleStatus competition;
  competition.velocity_x_mps = 16.0;
  interfaces::Objects objects;
  objects.header.stamp = ego.header.stamp;
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{*map->laneletLayer.begin()});

  const auto path = mission.process(ego, competition, objects, route);
  ASSERT_GT(path.points.size(), 30u);
  bool saw_exit_speed = false;
  for (const auto& point : path.points) {
    const double y = point.pose.pose.position.y;
    const double profile = std::sqrt(15.0 * 15.0 + 2 * 3 * std::max(0.0, y + 100));
    EXPECT_NEAR(point.velocity, std::min(16.0, profile), 1e-6) << "y=" << y;
    if (y <= -100) {
      EXPECT_NEAR(point.velocity, 15.0, 1e-6);
      saw_exit_speed = true;
    }
  }
  EXPECT_TRUE(saw_exit_speed);
  EXPECT_NEAR(path.points.front().velocity, 16.0, 1e-6);  // ego far enough before the exit
}
