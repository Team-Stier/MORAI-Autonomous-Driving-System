#include <chrono>
#include <fstream>
#include <gtest/gtest.h>
#include <interfaces/SectionAnnotation.h>
#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/core_prediction.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <path_planner/parameters/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <ros/serialization.h>

namespace {
template<class T> T readMessage(std::ifstream& input) {
  uint32_t size=0;input.read(reinterpret_cast<char*>(&size),sizeof(size));
  EXPECT_GT(size,0u);EXPECT_LT(size,1024u*1024u);
  if(!input || size==0 || size>=1024u*1024u)return T{};
  std::vector<uint8_t> bytes(size);input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),bytes.size());
  T message;ros::serialization::deserialize(stream,message);return message;
}

TEST(CorePathConnectedSpeedBudget, PreservesTheVerifiedCurveBeforeItsControlReferenceExpires) {
  // #CONNECTED_CURVATURE_SEARCH
  // R217의 정상 map 뒷차축·객체·IMU 원본을 그대로 호출한다. 같은 곡선에서
  // 최종 공간 곡률 검사를 뒤늦게 통과하지 못하는 후보를 반복 계산하면
  // 경로가 존재해도 제어 입력이 먼저 만료된다. 이미 검증된 이동 경로의
  // 형상·속도·가속도·예정 시각을 보존하면서 갱신 시간을 검사한다.
  // 객체/도로/물리 한도나 MPC 만료 시간을 늘리는 변경을 허용하지 않는다.
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  std::ifstream input(pkg+"/test/data/core_path_connected_speed_r217.bin",std::ios::binary);
  ASSERT_TRUE(input.good());uint32_t count=0;input.read(reinterpret_cast<char*>(&count),4);ASSERT_EQ(count,1u);
  double elapsed_recording=0;uint64_t ns=0;input.read(reinterpret_cast<char*>(&elapsed_recording),8);input.read(reinterpret_cast<char*>(&ns),8);
  const auto ego=readMessage<nav_msgs::Odometry>(input);
  const auto status=readMessage<interfaces::CompetitionVehicleStatus>(input);
  const auto objects=readMessage<interfaces::Objects>(input);
  const auto tf=readMessage<tf2_msgs::TFMessage>(input);
  const auto imu=readMessage<sensor_msgs::Imu>(input);
  const auto section=readMessage<interfaces::SectionAnnotation>(input);
  const auto recorded=readMessage<interfaces::Path>(input);ASSERT_TRUE(input.good());
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/global_path.yaml",pkg)),map);
  const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,path_planner::Checkpoints(checkpoints.begin()+section.state,checkpoints.end())));
  ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePrediction prediction;
  const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));
  path_planner::CorePathPlanner core(path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg)),map);
  ros::Time stamp;stamp.fromNSec(ns);const auto begin=std::chrono::steady_clock::now();
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  RecordProperty("planning_elapsed_sec",std::to_string(elapsed));EXPECT_LT(elapsed,.30);
  ASSERT_GE(path.points.size(),2u);ASSERT_EQ(path.points.size(),recorded.points.size());
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);EXPECT_GT(path.points.back().velocity,0);
  for(std::size_t i=0;i<path.points.size();++i) {
    EXPECT_NEAR(path.points[i].pose.pose.position.x,recorded.points[i].pose.pose.position.x,1e-9);
    EXPECT_NEAR(path.points[i].pose.pose.position.y,recorded.points[i].pose.pose.position.y,1e-9);
    EXPECT_NEAR(path.points[i].pose.pose.orientation.z,recorded.points[i].pose.pose.orientation.z,1e-9);
    EXPECT_NEAR(path.points[i].pose.pose.orientation.w,recorded.points[i].pose.pose.orientation.w,1e-9);
    EXPECT_NEAR(path.points[i].velocity,recorded.points[i].velocity,1e-9);
    EXPECT_NEAR(path.points[i].acceleration,recorded.points[i].acceleration,1e-9);
    EXPECT_EQ(path.points[i].time_from_start,recorded.points[i].time_from_start);
  }
}
}  // namespace
