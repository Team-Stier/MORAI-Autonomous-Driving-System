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
template<class T> T readFixtureMessage(std::ifstream& input) {
  uint32_t size=0;
  input.read(reinterpret_cast<char*>(&size),sizeof(size));
  EXPECT_GT(size,0u);
  EXPECT_LT(size,1024u*1024u);
  if(!input || size==0 || size>=1024u*1024u) return T{};
  std::vector<uint8_t> bytes(size);
  input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),bytes.size());
  T message;ros::serialization::deserialize(stream,message);
  return message;
}
TEST(CorePathExitContinuity, ComparesExistingCentralGeometryBeforeUnnecessaryEarlyBraking) {
  // #EXIT_PATH_CONTINUITY
  // R165 출구의 실제 조립 입력과 19개 map 객체를 그대로 재생한다.
  // 원래 첫 중앙 후보는 11.86m/s를 2초 뒤 5.55m/s로 줄였다.
  // 기존 계산 창 안의 다른 중앙 후보는 같은 차체·도로·동적 충돌·곡률·
  // 횡가속도·가감속·jerk 검사를 통과하며 이 불필요한 초기 제동을 피한다.
  // fixture 시각이나 초기 v/a를 보정하거나 실제 객체를 제외하지 않는다.
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  std::ifstream input(package+"/test/data/core_path_exit_r165.bin",std::ios::binary);
  ASSERT_TRUE(input.good());
  uint64_t nanoseconds=0;input.read(reinterpret_cast<char*>(&nanoseconds),sizeof(nanoseconds));
  const auto ego=readFixtureMessage<nav_msgs::Odometry>(input);
  const auto status=readFixtureMessage<interfaces::CompetitionVehicleStatus>(input);
  const auto objects=readFixtureMessage<interfaces::Objects>(input);
  const auto tf=readFixtureMessage<tf2_msgs::TFMessage>(input);
  const auto imu=readFixtureMessage<sensor_msgs::Imu>(input);
  ASSERT_TRUE(input.good());
  const auto reader=node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/shared_map.yaml",package);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/global_path.yaml",package)),map);
  const path_planner::Checkpoints remaining(checkpoints.begin()+12,checkpoints.end());
  const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePrediction prediction;
  const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  path_planner::CorePathPlanner core(parameters,map);
  ros::Time stamp;stamp.fromNSec(nanoseconds);
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  ASSERT_GE(path.points.size(),2u);
  const auto two=std::min_element(path.points.begin(),path.points.end(),[](const interfaces::PathPoint& a,const interfaces::PathPoint& b){return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);});
  EXPECT_GE(two->velocity,.9*status.velocity_x_mps);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GE(path.points.back().time_from_start.toSec(),4.8);
}
}  // namespace
