#include <chrono>
#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/core_prediction.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/global_path.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <fstream>
namespace {
template<class T> T readMessage(std::ifstream& input) {
  uint32_t size=0;input.read(reinterpret_cast<char*>(&size),sizeof(size));
  EXPECT_GT(size,0u);EXPECT_LT(size,1024u*1024u);
  if(!input || size==0 || size>=1024u*1024u)return T{};
  std::vector<uint8_t> bytes(size);input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),bytes.size());T message;ros::serialization::deserialize(stream,message);return message;
}
void verifyMeasuredMerge(bool rear_approaching) {
  const int section=12;
  // #MEASURED_ACCELERATION_MERGE_CHOICE
  // R205에서 목표 차선의 빠른 뒤차 앞으로 이동하기 직전 입력이다.
  // 계획은 약 2m/s²를 요구하지만 실측 약 0.8m/s²가 유지되면 뒤차와
  // 겹치는 장면이다. 실제 초기 운동량·map 도로·원본 예측을 보존하고,
  // 제동 없는 현재 차선 경로를 선택하는지 확인한다. 빈 장면은 같은 입력의
  // 객체만 제거한 대조군이며, 합법적인 옆 차선 이동을 막지 않아야 한다.
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  std::ifstream file(package+"/test/data/core_path_measured_acceleration_merge_r205.bin",std::ios::binary);
  ASSERT_TRUE(file.good());uint64_t ns=0;file.read(reinterpret_cast<char*>(&ns),sizeof(ns));
  const auto ego=readMessage<nav_msgs::Odometry>(file);
  const auto status=readMessage<interfaces::CompetitionVehicleStatus>(file);
  auto objects=readMessage<interfaces::Objects>(file);
  ASSERT_FALSE(objects.objects.empty());
  if(!rear_approaching)objects.objects.clear();
  const auto tf=readMessage<tf2_msgs::TFMessage>(file);
  const auto imu=readMessage<sensor_msgs::Imu>(file);ASSERT_TRUE(file.good());
  const auto reader=node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/shared_map.yaml",package);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/global_path.yaml",package)),map);
  const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,
      path_planner::Checkpoints(checkpoints.begin()+section,checkpoints.end())));ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePrediction prediction;const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));
  const auto params=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  EXPECT_DOUBLE_EQ(params.road_margin_m,.05);
  path_planner::CorePathPlanner core(params,map);ros::Time stamp;stamp.fromNSec(ns);
  const auto before=std::chrono::steady_clock::now();
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count();
  testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(elapsed));
  EXPECT_LT(elapsed,.25);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  const auto two=std::min_element(path.points.begin(),path.points.end(),[](const interfaces::PathPoint& a,const interfaces::PathPoint& b) {
    return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);
  });
  const double lateral_shift=two->pose.pose.position.x-ego.pose.pose.position.x;
  if(rear_approaching)EXPECT_LT(lateral_shift,1.0);
  else EXPECT_GT(lateral_shift,2.0);
  for(const auto& point:path.points)if(point.time_from_start.toSec()<=.5)EXPECT_GE(point.acceleration,-1e-6);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
  EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-path.points.front().pose.pose.position.x,
                      path.points.back().pose.pose.position.y-path.points.front().pose.pose.position.y),1);
}
TEST(CorePathMeasuredAccelerationMerge, KeepsMovingInTheCurrentLaneBesideAnApproachingRearVehicle) {
  verifyMeasuredMerge(true);
}
TEST(CorePathMeasuredAccelerationMerge, ContinuesTheAdjacentLaneChangeWhenTheGapIsClear) {
  verifyMeasuredMerge(false);
}
}  // namespace
