#include <path_planner/cores/global_path.hpp>
#include <gtest/gtest.h>

namespace pp = path_planner;
namespace {
pp::SharedMap connectedMap() {
  std::vector<lanelet::Point3d> left, right;
  for (int i = 0; i <= 3; ++i) {
    left.emplace_back(100 + 2*i, 5.0*i, 1, 0);
    right.emplace_back(101 + 2*i, 5.0*i, -1, 0);
  }
  lanelet::Lanelets lanes;
  for (int i = 0; i < 3; ++i) {
    lanelet::Lanelet lane(300 + i,
        lanelet::LineString3d(200 + 2*i, {left[i], left[i + 1]}),
        lanelet::LineString3d(201 + 2*i, {right[i], right[i + 1]}));
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["one_way"] = "yes";
    lanes.push_back(lane);
  }
  return pp::SharedMap({lanelet::utils::createMap(lanes)});
}
}

TEST(GlobalPath, PrependsPredecessorAndKeepsCurrentLaneWhenEgoAdvances) {
  auto shared = connectedMap();
  pp::GlobalPath core(shared);
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 5.1;
  ego.pose.pose.orientation.w = 1;
  pp::Checkpoints checkpoints(1);
  checkpoints.front().lanelet_id = 302;
  for (int repeat = 0; repeat < 2; ++repeat) {
    const auto route = std::get<5>(core.process(ego, interfaces::CompetitionVehicleStatus{},
        interfaces::Objects{}, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, checkpoints));
    ASSERT_TRUE(static_cast<bool>(route));
    ASSERT_EQ(route->size(), 3u);
    EXPECT_EQ((*route)[0].id(), 300);
    EXPECT_EQ((*route)[1].id(), 301);
    EXPECT_EQ((*route)[2].id(), 302);
  }
  ego.pose.pose.position.x = 10.1;
  const auto route = std::get<5>(core.process(ego, interfaces::CompetitionVehicleStatus{},
      interfaces::Objects{}, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  ASSERT_EQ(route->size(), 2u);
  EXPECT_EQ(route->front().id(), 301);
  EXPECT_EQ(route->back().id(), 302);
}

TEST(GlobalPath, PrependsPredecessorWithoutRemainingCheckpoints) {
  auto shared = connectedMap();
  pp::GlobalPath core(shared);
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 5.1;
  ego.pose.pose.orientation.w = 1;
  const auto route = std::get<5>(core.process(ego, interfaces::CompetitionVehicleStatus{},
      interfaces::Objects{}, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Checkpoints{}));
  ASSERT_TRUE(static_cast<bool>(route));
  ASSERT_EQ(route->size(), 2u);
  EXPECT_EQ(route->front().id(), 300);
  EXPECT_EQ(route->back().id(), 301);
}

TEST(GlobalPath, KeepsCurrentLaneWhenItHasNoPredecessor) {
  auto shared = connectedMap();
  pp::GlobalPath core(shared);
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 0.1;
  ego.pose.pose.orientation.w = 1;
  const auto route = std::get<5>(core.process(ego, interfaces::CompetitionVehicleStatus{},
      interfaces::Objects{}, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Checkpoints{}));
  ASSERT_TRUE(static_cast<bool>(route));
  ASSERT_EQ(route->size(), 1u);
  EXPECT_EQ(route->front().id(), 300);
}
