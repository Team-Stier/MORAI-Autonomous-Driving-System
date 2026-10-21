#include <path_planner/cores/global_path.hpp>
#include <gtest/gtest.h>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/global_path.hpp>

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

TEST(GlobalPath, KeepsPlannedStraightBranchAcrossOverlappingNativeJunction) {
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/shared_map.yaml",package);
  pp::SharedMap shared(pp::loadSharedMapParameters(reader));
  pp::GlobalPath core(shared);
  const auto loaded=pp::readCheckpoints(reader.asset("checkpoints_file"));
  const pp::Checkpoints remaining(loaded.begin()+10,loaded.end());
  struct Fixture { double x,y,qx,qy,qz,qw; lanelet::Id current; };
  const Fixture poses[]{
    {-66.0269,342.475,0,0,std::sin(.0688079/2),std::cos(.0688079/2),110266},
    {-65.0844960066903,342.5325795830591,-0.001856259368865971,-0.004263927519195721,-0.03406366251498903,-0.9994088453277424,110022},
    {-62.59024905319982,342.66546365112845,-0.0037477902751589607,-0.003662528265570861,-0.03458687942510527,-0.999387956564612,110022}
  };
  for(const auto& p:poses) {
    SCOPED_TRACE(p.x);
    nav_msgs::Odometry ego;ego.pose.pose.position.x=p.x;ego.pose.pose.position.y=p.y;
    auto& q=ego.pose.pose.orientation;q.x=p.qx;q.y=p.qy;q.z=p.qz;q.w=p.qw;
    const auto route=std::get<5>(core.process(ego,interfaces::CompetitionVehicleStatus{},interfaces::Objects{},tf2_msgs::TFMessage{},sensor_msgs::Imu{},remaining));
    ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),3u);
    EXPECT_EQ((*route)[1].id(),p.current);
    EXPECT_EQ((*route)[2].id(),p.current==110266?110022:110031);
  }
}

TEST(GlobalPath, LeavesPlannedRouteWhenEgoEntersAnotherNativeBranch) {
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/shared_map.yaml",package);
  pp::SharedMap shared(pp::loadSharedMapParameters(reader));
  pp::GlobalPath core(shared);
  const auto loaded=pp::readCheckpoints(reader.asset("checkpoints_file"));
  const pp::Checkpoints remaining(loaded.begin()+10,loaded.end());
  nav_msgs::Odometry ego;ego.pose.pose.position.x=-66.0269;ego.pose.pose.position.y=342.475;
  ego.pose.pose.orientation.z=std::sin(.0688079/2);ego.pose.pose.orientation.w=std::cos(.0688079/2);
  core.process(ego,interfaces::CompetitionVehicleStatus{},interfaces::Objects{},tf2_msgs::TFMessage{},sensor_msgs::Imu{},remaining);
  const auto line=shared.map->laneletLayer.get(105233).centerline3d().basicLineString();
  ASSERT_GE(line.size(),2u);const auto i=std::min(line.size()-2,line.size()/2);
  const auto point=(line[i]+line[i+1])/2;const auto direction=line[i+1]-line[i];
  const double yaw=std::atan2(direction.y(),direction.x());
  ego.pose.pose.position.x=point.x();ego.pose.pose.position.y=point.y();
  ego.pose.pose.orientation.z=std::sin(yaw/2);ego.pose.pose.orientation.w=std::cos(yaw/2);
  const auto route=std::get<5>(core.process(ego,interfaces::CompetitionVehicleStatus{},interfaces::Objects{},tf2_msgs::TFMessage{},sensor_msgs::Imu{},remaining));
  ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),2u);
  EXPECT_EQ((*route)[1].id(),105233);
}

TEST(GlobalPathEarlyChange, ReachesAdjacentTargetWithoutWaitingForDistantLanelet) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto loaded=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath core(shared);nav_msgs::Odometry ego;
  ego.pose.pose.position.x=66.94;ego.pose.pose.position.y=80;
  ego.pose.pose.orientation.z=-std::sqrt(.5);ego.pose.pose.orientation.w=std::sqrt(.5);
  const path_planner::Checkpoints remaining(loaded.begin()+12,loaded.end());
  const auto route=std::get<5>(core.process(ego,{}, {}, {}, {},remaining));
  ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),3u);
  EXPECT_EQ((*route)[1].id(),97498);EXPECT_EQ((*route)[2].id(),97499);
  auto next=route->begin();
  for(const auto& checkpoint:remaining) {
    next=std::find_if(next,route->end(),[&](const lanelet::ConstLanelet& lane){return lane.id()==checkpoint.lanelet_id;});
    ASSERT_NE(next,route->end());++next;
  }
}
TEST(GlobalPathEarlyChange, KeepsMandatoryOuterLaneBeforeStartingEarlyChange) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto loaded=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath core(shared);nav_msgs::Odometry ego;
  ego.pose.pose.position.x=66.6;ego.pose.pose.position.y=130;
  ego.pose.pose.orientation.z=-std::sqrt(.5);ego.pose.pose.orientation.w=std::sqrt(.5);
  const auto route=std::get<5>(core.process(ego,{}, {}, {}, {},path_planner::Checkpoints(loaded.begin()+11,loaded.end())));
  ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),4u);
  EXPECT_EQ((*route)[1].id(),96952);EXPECT_EQ((*route)[2].id(),97498);EXPECT_EQ((*route)[3].id(),97499);
}
TEST(GlobalPathEarlyChange, DoesNotInventAdjacentRoadOnCurvedMergeApproach) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto loaded=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath core(shared);nav_msgs::Odometry ego;
  ego.pose.pose.position.x=59.0272445;ego.pose.pose.position.y=291.2387205;
  ego.pose.pose.orientation.z=.6340805644;ego.pose.pose.orientation.w=-.7732509866;
  const auto route=std::get<5>(core.process(ego,{}, {}, {}, {},path_planner::Checkpoints(loaded.begin()+10,loaded.end())));
  ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),4u);
  EXPECT_EQ((*route)[1].id(),110016);EXPECT_EQ((*route)[2].id(),96938);EXPECT_EQ((*route)[3].id(),96937);
}

TEST(GlobalPathEarlyChange, ApprovedAdjacentCheckpointAllowsChangeBeforeOriginalLanelet) {
 ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
 auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
 pp::SharedMap shared(pp::loadSharedMapParameters(reader));auto loaded=pp::readCheckpoints(reader.asset("checkpoints_file"));
 pp::GlobalPath core(pp::GlobalPathParameters{{97498},10},shared);nav_msgs::Odometry ego;
 ego.pose.pose.position.x=66.6;ego.pose.pose.position.y=130;
 ego.pose.pose.orientation.z=-std::sqrt(.5);ego.pose.pose.orientation.w=std::sqrt(.5);
 pp::Checkpoints remaining(loaded.begin()+11,loaded.end());
 auto route=std::get<5>(core.process(ego,{}, {}, {}, {},remaining));
 ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),3u);
 EXPECT_EQ((*route)[1].id(),96952);EXPECT_EQ((*route)[2].id(),96955);
 auto next=route->begin();for(size_t i=1;i<remaining.size();++i) {
  next=std::find_if(next,route->end(),[&](const lanelet::ConstLanelet& lane){return lane.id()==remaining[i].lanelet_id;});
  ASSERT_NE(next,route->end());++next;
 }
 const auto& checkpoint=remaining.front();double distance=1e9;
 for(const auto& lane:*route)if(lane.id()==97499)for(const auto& point:lane.centerline3d())
  distance=std::min(distance,std::hypot(point.x()-checkpoint.x_m,point.y()-checkpoint.y_m));
 EXPECT_LT(distance,10.0);
}

TEST(GlobalPathEarlyChange, AdjacentCheckpointStillRequiresItsOriginalPassageRadius) {
 ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
 auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
 pp::SharedMap shared(pp::loadSharedMapParameters(reader));auto loaded=pp::readCheckpoints(reader.asset("checkpoints_file"));
 pp::GlobalPath core(pp::GlobalPathParameters{{97498},1},shared);nav_msgs::Odometry ego;
 ego.pose.pose.position.x=66.6;ego.pose.pose.position.y=130;
 ego.pose.pose.orientation.z=-std::sqrt(.5);ego.pose.pose.orientation.w=std::sqrt(.5);
 auto route=std::get<5>(core.process(ego,{}, {}, {}, {},pp::Checkpoints(loaded.begin()+11,loaded.end())));
 ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),4u);
 EXPECT_EQ((*route)[1].id(),96952);EXPECT_EQ((*route)[2].id(),97498);
}
TEST(GlobalPathEarlyChange, ApprovedAdjacentCheckpointDoesNotReturnEgoToOriginalLane) {
 ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
 auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
 pp::SharedMap shared(pp::loadSharedMapParameters(reader));auto loaded=pp::readCheckpoints(reader.asset("checkpoints_file"));
 pp::GlobalPath core(pp::GlobalPathParameters{{97498},10},shared);nav_msgs::Odometry ego;
 ego.pose.pose.position.x=70.4;ego.pose.pose.position.y=120;
 ego.pose.pose.orientation.z=-std::sqrt(.5);ego.pose.pose.orientation.w=std::sqrt(.5);
 auto remaining=pp::Checkpoints(loaded.begin()+11,loaded.end());
 for(int repeat=0;repeat<2;++repeat) {
  auto route=std::get<5>(core.process(ego,{}, {}, {}, {},remaining));
  ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),3u);
  EXPECT_EQ((*route)[1].id(),96955);
  EXPECT_EQ(std::find_if(route->begin()+1,route->end(),[](const lanelet::ConstLanelet& lane){return lane.id()==97498;}),route->end());
 }
}
TEST(GlobalPathEarlyChange, AdjacentCheckpointSettingPreservesOtherMandatoryCheckpoints) {
 ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
 auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
 pp::SharedMap shared(pp::loadSharedMapParameters(reader));auto loaded=pp::readCheckpoints(reader.asset("checkpoints_file"));
 pp::GlobalPath core(pp::GlobalPathParameters{{97498},10},shared);nav_msgs::Odometry ego;
 ego.pose.pose.position.x=59.0272445;ego.pose.pose.position.y=291.2387205;
 ego.pose.pose.orientation.z=.6340805644;ego.pose.pose.orientation.w=-.7732509866;
 auto remaining=pp::Checkpoints(loaded.begin()+10,loaded.end());
 auto route=std::get<5>(core.process(ego,{}, {}, {}, {},remaining));
 ASSERT_TRUE(static_cast<bool>(route));ASSERT_GE(route->size(),4u);
 EXPECT_EQ((*route)[1].id(),110016);EXPECT_EQ((*route)[2].id(),96938);EXPECT_EQ((*route)[3].id(),96937);
}
TEST(GlobalPathEarlyChange, LoadsOnlyApprovedCheckpointAndSharedSectionRadius) {
 const std::string pkg=NP_PACKAGE_DIR;
 const auto parameters=pp::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(
     pkg+"/config/path_planner/parameters/global_path.yaml",pkg));
 ASSERT_EQ(parameters.adjacent_checkpoint_lanelet_ids.size(),1u);
 EXPECT_EQ(parameters.adjacent_checkpoint_lanelet_ids.front(),97498);
 EXPECT_DOUBLE_EQ(parameters.checkpoint_radius_m,10.0);
}
