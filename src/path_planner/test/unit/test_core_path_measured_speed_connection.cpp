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
void verifyMeasuredSpeedConnection() {
  const int section=13;
  // #MEASURED_SPEED_CONNECTION
  // R203의 실제 1→2차선 변경 직전 입력이다. 고속 연결이 필요한
  // 실측 89km/h를 그대로 유지하고, 동일 객체·도로·5초 예측으로 계획한다.
  // 짧은 연결 실패 뒤 기존 긴 연결을 누락해 빈 경로가 되는 회귀를 검사한다.
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  std::ifstream file(package+"/test/data/core_path_measured_speed_connection_r203.bin",std::ios::binary);
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
  EXPECT_DOUBLE_EQ(params.road_margin_m,.05);
  path_planner::CorePathPlanner core(params,map);ros::Time stamp;stamp.fromNSec(ns);
  const auto begin=std::chrono::steady_clock::now();
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()));
  ASSERT_GT(status.velocity_x_mps,80/3.6);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
  EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-path.points.front().pose.pose.position.x,
                      path.points.back().pose.pose.position.y-path.points.front().pose.pose.position.y),1);
}
TEST(CorePathMeasuredSpeedConnection, DoesNotDiscardLongConnectionsWhenShortConnectionsAreInfeasible) {
  verifyMeasuredSpeedConnection();
}
}  // namespace
