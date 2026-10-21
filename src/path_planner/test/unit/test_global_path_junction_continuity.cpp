#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <gtest/gtest.h>
#include <algorithm>
namespace {
nav_msgs::Odometry pose(double x,double y,double yaw) {
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(100,0);
  ego.pose.pose.position.x=x;ego.pose.pose.position.y=y;
  ego.pose.pose.orientation.z=std::sin(yaw/2);ego.pose.pose.orientation.w=std::cos(yaw/2);return ego;
}
TEST(GlobalPathJunctionContinuity, KeepsThePlannedExitAcrossOverlappingBranchPolygons) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  const path_planner::Checkpoints remaining(checkpoints.begin()+9,checkpoints.end());
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/global_path.yaml",pkg)),shared);
  const auto before=std::get<5>(global.process(pose(-89.7916859442,331.1968127,.872977341953),{},{},{},{},remaining));
  ASSERT_TRUE(bool(before));ASSERT_GE(before->size(),3u);const auto planned_next=(*before)[2];
  const auto overlap=std::get<5>(global.process(pose(-88.1384246755,333.061207672,.879074360672),{},{},{},{},remaining));
  ASSERT_TRUE(bool(overlap));ASSERT_GE(overlap->size(),2u);
  EXPECT_EQ((*overlap)[1],planned_next);
  EXPECT_LE(overlap->size(),before->size());
  // 실제로 원래 출구 polygon을 떠나 다른 가지에 들어간 경우는 재매칭한다.
  const auto off_route=std::get<5>(global.process(pose(-83.7268049763,345.670225807,1.72742488557),{},{},{},{},remaining));
  ASSERT_TRUE(bool(off_route));ASSERT_GE(off_route->size(),2u);
  EXPECT_NE((*off_route)[1],planned_next);
  EXPECT_EQ((*off_route)[1].id(),110309);
  // 임무가 바뀌면 과거 route를 새 매칭의 우선순위로 쓰지 않는다.
  path_planner::GlobalPath fresh(shared);
  const path_planner::Checkpoints changed(remaining.begin()+1,remaining.end());
  const auto fresh_changed=std::get<5>(fresh.process(pose(-88.1384246755,333.061207672,.879074360672),{},{},{},{},changed));
  const auto old_changed=std::get<5>(global.process(pose(-88.1384246755,333.061207672,.879074360672),{},{},{},{},changed));
  ASSERT_TRUE(bool(fresh_changed));ASSERT_TRUE(bool(old_changed));
  ASSERT_GE(fresh_changed->size(),2u);ASSERT_GE(old_changed->size(),2u);
  EXPECT_EQ((*fresh_changed)[1],(*old_changed)[1]);
}
}
