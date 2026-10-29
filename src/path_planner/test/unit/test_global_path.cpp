#include <path_planner/cores/global_path.hpp>
#include <gtest/gtest.h>
#include <path_planner/libraries/map_parameters.hpp>
#include <fstream>
#include <iterator>

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
pp::SharedMap recordedForkMap() {
  std::ifstream file(std::string(NP_PACKAGE_DIR)+"/../map/kcity_lanelet2_smooth_circle.osm");
  const std::string xml((std::istreambuf_iterator<char>(file)),{});
  return pp::SharedMap({pp::readLaneletMap(xml)});
}

nav_msgs::Odometry recordedForkPose(double x,double y,double yaw) {
  nav_msgs::Odometry ego;ego.pose.pose.position.x=x;ego.pose.pose.position.y=y;
  ego.pose.pose.orientation.z=std::sin(yaw/2);ego.pose.pose.orientation.w=std::cos(yaw/2);
  return ego;
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

TEST(GlobalPath, RecordedExitKeepsRouteThroughOverlappingBranchPolygons) {
  auto map=recordedForkMap();pp::GlobalPath core(map);
  pp::Checkpoints checkpoints(1);checkpoints.front().lanelet_id=96938;
  for(const auto& ego:{recordedForkPose(-67.3352896483,341.9526171342,.0530717703),
                      recordedForkPose(-65.2169905704,342.0943695143,.0772362309),
                      recordedForkPose(-64.3574216017,342.1588332515,.0834243479)}) {
    const auto route=std::get<5>(core.process(ego,{}, {}, {}, {},checkpoints));
    ASSERT_TRUE(static_cast<bool>(route));
    bool exit=false;
    for(const auto& lane:*route) {
      EXPECT_NE(lane.id(),140377);EXPECT_NE(lane.id(),105233);
      exit=exit || lane.id()==110022;
    }
    EXPECT_TRUE(exit);
  }
}

TEST(GlobalPath, ColdStartAtRecordedForkUsesDirectionBeforeCentimetreDistance) {
  auto map=recordedForkMap();pp::GlobalPath core(map);
  const auto route=std::get<5>(core.process(
      recordedForkPose(-65.2169905704,342.0943695143,.0772362309),{}, {}, {}, {}, {}));
  ASSERT_TRUE(static_cast<bool>(route));
  EXPECT_EQ(route->back().id(),110022);
}

TEST(GlobalPath, RouteContinuityStillAllowsAnActualLaneChange) {
  auto map=connectedMap();
  lanelet::Lanelet adjacent(400,
      lanelet::LineString3d(410,{lanelet::Point3d(411,0,5,0),lanelet::Point3d(412,15,5,0)}),
      lanelet::LineString3d(420,{lanelet::Point3d(421,0,3,0),lanelet::Point3d(422,15,3,0)}));
  adjacent.attributes()["subtype"]="road";adjacent.attributes()["location"]="urban";
  adjacent.attributes()["one_way"]="yes";map.map->add(adjacent);
  pp::GlobalPath core(map);
  auto route=std::get<5>(core.process(recordedForkPose(1,0,0),{}, {}, {}, {}, {}));
  ASSERT_TRUE(static_cast<bool>(route));EXPECT_EQ(route->back().id(),300);
  route=std::get<5>(core.process(recordedForkPose(1,4,0),{}, {}, {}, {}, {}));
  ASSERT_TRUE(static_cast<bool>(route));EXPECT_EQ(route->back().id(),400);
}
