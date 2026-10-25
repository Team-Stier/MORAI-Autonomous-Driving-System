#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/core_prediction.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/global_path.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <fstream>
#include <chrono>

namespace {
template<class T> T readMessage(std::ifstream& input) {
  uint32_t size=0;
  input.read(reinterpret_cast<char*>(&size),sizeof(size));
  EXPECT_GT(size,0u);
  EXPECT_LT(size,1024u*1024u);
  if(!input || size==0 || size>=1024u*1024u) return T{};
  std::vector<uint8_t> bytes(size);
  input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),bytes.size());
  T message;ros::serialization::deserialize(stream,message);return message;
}
void verifyHighway(const std::string& fixture,std::size_t section,double speed_fraction,bool deadline) {
  // #HIGHWAY_CONTINUITY
  // R173의 실제 조립 입력을 원본 시각·객체·IMU·차량 운동 그대로 재생한다.
  // 장애물 관측을 제거하거나 초기 속도를 낮춰 통과시키지 않는다. 관측된
  // 2초 내 불필요한 급감속을 재현하며, 5초 예측과 최초 속도 계약도 유지한다.
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  std::ifstream input(package+"/test/data/"+fixture,std::ios::binary);
  ASSERT_TRUE(input.good());
  uint64_t ns=0;input.read(reinterpret_cast<char*>(&ns),sizeof(ns));
  const auto ego=readMessage<nav_msgs::Odometry>(input);
  const auto status=readMessage<interfaces::CompetitionVehicleStatus>(input);
  const auto objects=readMessage<interfaces::Objects>(input);
  const auto tf=readMessage<tf2_msgs::TFMessage>(input);
  const auto imu=readMessage<sensor_msgs::Imu>(input);
  ASSERT_TRUE(input.good());
  ASSERT_FALSE(objects.objects.empty());
  const auto reader=node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/shared_map.yaml",package);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(
      node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/global_path.yaml",package)),map);
  const path_planner::Checkpoints remaining(checkpoints.begin()+section,checkpoints.end());
  const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePrediction prediction;
  const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));
  path_planner::CorePathPlanner core(path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/core_path_planner.yaml",package)),map);
  ros::Time stamp;stamp.fromNSec(ns);
  const auto started=std::chrono::steady_clock::now();
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(elapsed));
  if(deadline) EXPECT_LT(elapsed,.15) << "Planner exceeds existing Velocity Planner path stale_after_sec";
  ASSERT_GE(path.points.size(),2u);
  const auto two=std::min_element(path.points.begin(),path.points.end(),
      [](const interfaces::PathPoint& a,const interfaces::PathPoint& b) {
        return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);
      });
  EXPECT_GE(two->velocity,speed_fraction*status.velocity_x_mps);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GE(path.points.back().time_from_start.toSec(),4.8);
}
TEST(CorePathHighway, DoesNotLoseTheMergePathBesideAMovingRearVehicle) {
  verifyHighway("core_path_merge_r173.bin",11,.9,true);
}
TEST(CorePathHighway, FollowingPlanArrivesBeforeTheVelocityInputDeadline) {
  verifyHighway("core_path_following_r173.bin",12,0,true);
}
TEST(CorePathHighway, DenseMergeInputArrivesBeforeTheVelocityDeadline) {
  // R181의 정확한 조립 입력이다. 기존 약 1초 계산이 경로 입력 만료와
  // 최대 제동을 일으킨 장면이며, 원본 차량·객체·예측·도로를 모두 유지한다.
  verifyHighway("core_path_merge_deadline_r181.bin",11,.98,true);
}
}  // namespace
