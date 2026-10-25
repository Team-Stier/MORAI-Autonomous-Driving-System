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
void verifyLongAvoidanceWindow() {
  const int section=12;
  // #HIGH_SPEED_AVOIDANCE_WINDOW
  // R211 출구 접근의 원본 map 관측을 재생한다. 옆 차량의 예측이 중앙선
  // 형상을 막아도, 실측 속도의 정지 가능 공간보다 짧은 50m 창만 검사해
  // 빈 경로를 반환하면 안 된다. 원래 객체·전체 차체와 5초 예측을 유지하고,
  // 기존 긴 창의 단일 C2가 실제 도로·충돌·동역학 검사를 통과하는지 확인한다.
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  std::ifstream file(package+"/test/data/core_path_long_avoidance_r211.bin",std::ios::binary);
  ASSERT_TRUE(file.good());uint64_t ns=0;file.read(reinterpret_cast<char*>(&ns),sizeof(ns));
  const auto ego=readMessage<nav_msgs::Odometry>(file);
  const auto status=readMessage<interfaces::CompetitionVehicleStatus>(file);
  const auto objects=readMessage<interfaces::Objects>(file);
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
  path_planner::CorePathPlanner core(params,map);ros::Time stamp;stamp.fromNSec(ns);
  const auto begin=std::chrono::steady_clock::now();
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(elapsed));
  EXPECT_LT(elapsed,.30);
  ASSERT_GT(status.velocity_x_mps,60/3.6);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
  EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-path.points.front().pose.pose.position.x,
                      path.points.back().pose.pose.position.y-path.points.front().pose.pose.position.y),60);
  EXPECT_GT(path.points.back().velocity,0.0);
  for(std::size_t i=1;i<path.points.size();++i) {
    const auto& a=path.points[i-1];const auto& b=path.points[i];
    const double distance=std::hypot(b.pose.pose.position.x-a.pose.pose.position.x,b.pose.pose.position.y-a.pose.pose.position.y);
    if(distance<=1e-8)continue;
    const auto yaw=[](const geometry_msgs::Quaternion& q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
    const double angle=std::atan2(std::sin(yaw(b.pose.pose.orientation)-yaw(a.pose.pose.orientation)),std::cos(yaw(b.pose.pose.orientation)-yaw(a.pose.pose.orientation)));
    EXPECT_LE(std::max(a.velocity*a.velocity,b.velocity*b.velocity)*std::abs(angle)/distance,params.max_lateral_accel_mps2+1e-5);
  }
}
TEST(CorePathLongAvoidanceWindow, PreservesAValidTrajectoryWhenAdjacentTrafficCutsAcrossTheReference) {
  verifyLongAvoidanceWindow();
}
}  // namespace
