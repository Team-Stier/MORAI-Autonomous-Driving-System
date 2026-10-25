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
void verifyNativeReentry(int section) {
  // #REENTRY_INTERVAL_GEOMETRY_REUSE
  // R202의 계획 여유 복귀 입력을 객체·운동·시각 그대로 재생한다.
  // 0.5초 Control 입력 수명의 절반 안에서 같은 payload를 계산하는지 검사한다.
  // 전체 경로의 시각 헤더는 Pipeline이 주입하므로 payload 비교에서만 정렬한다.
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  std::ifstream file(package+"/test/data/core_path_reentry_cost_r202.bin",std::ios::binary);
  ASSERT_TRUE(file.good());uint64_t ns=0;file.read(reinterpret_cast<char*>(&ns),sizeof(ns));
  const auto ego=readMessage<nav_msgs::Odometry>(file);
  const auto status=readMessage<interfaces::CompetitionVehicleStatus>(file);
  const auto objects=readMessage<interfaces::Objects>(file);
  const auto tf=readMessage<tf2_msgs::TFMessage>(file);
  const auto imu=readMessage<sensor_msgs::Imu>(file);
  const auto expected=readMessage<interfaces::Path>(file);ASSERT_TRUE(file.good());
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
  const auto started=std::chrono::steady_clock::now();
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(elapsed));
  EXPECT_LT(elapsed,.25);
  ASSERT_GE(path.points.size(),2u);
  ASSERT_EQ(path.points.size(),expected.points.size());
  auto actual=path;actual.header=expected.header;
  for(std::size_t i=0;i<actual.points.size();++i)actual.points[i].pose.header=expected.points[i].pose.header;
  const auto serialized=[](const interfaces::Path& message) {
    std::vector<uint8_t> bytes(ros::serialization::serializationLength(message));
    ros::serialization::OStream stream(bytes.data(),bytes.size());
    ros::serialization::serialize(stream,message);return bytes;
  };
  EXPECT_TRUE(serialized(actual)==serialized(expected));
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
  EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-path.points.front().pose.pose.position.x,
                      path.points.back().pose.pose.position.y-path.points.front().pose.pose.position.y),1);
}
TEST(CorePathReentryCost, PreservesTheNativeCurveBeforeTheControlInputExpires) {
  verifyNativeReentry(2);
}
}  // namespace
