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
void verifySpatialCurvatureSearch() {
  const int section=13;
  // #SPATIAL_CURVATURE_SEARCH
  // R208의 1→2차선 출구 변경 입력을 그대로 재생한다. map 뒷차축 자세,
  // 실측 v/a, 원본 객체와 예측을 유지한다. 공간 재표본화의 곡률을 뒤늦게
  // 발견해 같은 속도 후보를 반복 거부하던 약 0.44초 계산을 재현한다.
  // 제어 입력 한도는 바꾸지 않고, 충분한 길이·시간·속도와 기존 횡가속도
  // 한도를 함께 검사한다. 시간 예산은 이 시험 PC의 회귀 검출 기준이다.
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  std::ifstream file(package+"/test/data/core_path_spatial_curvature_search_r208.bin",std::ios::binary);
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
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(elapsed));
  EXPECT_LT(elapsed,.25);
  ASSERT_GT(status.velocity_x_mps,60/3.6);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
  EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-path.points.front().pose.pose.position.x,
                      path.points.back().pose.pose.position.y-path.points.front().pose.pose.position.y),60);
  EXPECT_LT(path.points.back().pose.pose.position.x,ego.pose.pose.position.x-3);
  for(std::size_t i=1;i<path.points.size();++i) {
    const auto& a=path.points[i-1];const auto& b=path.points[i];
    const double distance=std::hypot(b.pose.pose.position.x-a.pose.pose.position.x,b.pose.pose.position.y-a.pose.pose.position.y);
    if(distance<=1e-8)continue;
    const auto yaw=[](const geometry_msgs::Quaternion& q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
    const double angle=std::atan2(std::sin(yaw(b.pose.pose.orientation)-yaw(a.pose.pose.orientation)),std::cos(yaw(b.pose.pose.orientation)-yaw(a.pose.pose.orientation)));
    EXPECT_LE(std::max(a.velocity*a.velocity,b.velocity*b.velocity)*std::abs(angle)/distance,params.max_lateral_accel_mps2+1e-5);
  }
}
TEST(CorePathSpatialCurvatureSearch, ProducesACompleteSafeExitTrajectoryBeforeTheControlDeadline) {
  verifySpatialCurvatureSearch();
}
}  // namespace
