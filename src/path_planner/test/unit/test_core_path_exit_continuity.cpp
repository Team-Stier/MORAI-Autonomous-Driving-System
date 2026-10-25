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
void verifyExitSpeed(const std::string& fixture,std::size_t section,double speed_fraction=.9,bool deadline=false) {
  // #EXIT_PATH_CONTINUITY
  // 실측 조립 입력의 시각·초기 운동·map 객체를 보정 없이 재생한다.
  // 전체 차체·도로·동적 충돌·곡률·횡가속도·가감속·jerk 검사를 거친
  // 5초 궤적이 불필요하게 초기 속도를 잃지 않는지 확인한다.
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  std::ifstream input(package+"/test/data/"+fixture,std::ios::binary);
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
  const path_planner::Checkpoints remaining(checkpoints.begin()+section,checkpoints.end());
  const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePrediction prediction;
  const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  path_planner::CorePathPlanner core(parameters,map);
  ros::Time stamp;stamp.fromNSec(nanoseconds);
  const auto started=std::chrono::steady_clock::now();
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(elapsed));
  if(deadline) EXPECT_LT(elapsed,.15);
  ASSERT_GE(path.points.size(),2u);
  const auto two=std::min_element(path.points.begin(),path.points.end(),[](const interfaces::PathPoint& a,const interfaces::PathPoint& b){return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);});
  EXPECT_GE(two->velocity,speed_fraction*status.velocity_x_mps);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GE(path.points.back().time_from_start.toSec(),4.8);
}
TEST(CorePathExitContinuity, ComparesExistingCentralGeometryBeforeUnnecessaryEarlyBraking) {
  // R165: 11.86m/s를 2초 뒤 5.55m/s로 줄이던 기존 중앙 후보 선택 회귀다.
  verifyExitSpeed("core_path_exit_r165.bin",12);
}
TEST(CorePathExitContinuity, ShortSourceConnectorKeepsFeasibleSpeedBeforeTheExitLaneChange) {
  // R168의 실제 출구 접근 입력이다. 합법적으로 이어진 도로가 남아 있는데도
  // 25m에 횡이동을 압축하면 14.82m/s를 2초 뒤 7.41m/s까지 줄인다.
  // 기존 연결 범위 50m를 쓰되 객체·차체·곡률·횡가속도 검사는 보존한다.
  verifyExitSpeed("core_path_exit_r168.bin",13);
}

TEST(CorePathExitContinuity, HighwaySpeedDoesNotCompressTheExitIntoAnOrdinaryRoadWindow) {
  // R174의 정확한 발행 stamp에 대응하는 원본 5개 입력이다. 23.84m/s에서
  // 2초 뒤 18.28m/s로 줄어들던 회귀를 고정하며 한도/객체는 바꾸지 않는다.
  verifyExitSpeed("core_path_exit_highspeed_r174.bin",12,.98,true);
}
TEST(CorePathExitContinuity, ExitLaneChangeKeepsFeasibleSpeedAcrossTheLongConnection) {
  verifyExitSpeed("core_path_exit_lane_r174.bin",13,.98,true);
}
}  // namespace
