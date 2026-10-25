#include <chrono>
#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/core_prediction.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/global_path.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <ros/serialization.h>
#include <interfaces/SectionAnnotation.h>
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

TEST(CorePathFutureCurveConnection, KeepsMovingThroughTheCurveBeforeAnUpcomingLaneChange) {
  // #FUTURE_LANE_CHANGE_CURVE_CONNECTION
  // R214 교차로 출구의 연속 원본 입력 두 개다. 차체는 실제 도로 안에 있고
  // MPC는 기존 경로를 추종하지만, 아직 시작하지 않은 차선 변경 예고 때문에
  // 현재 곡선의 연결을 제한해 경로가 비는 회귀를 검사한다. 실측 초기 운동과
  // 원본 객체·map·예측을 유지한 시간 경로, 실제 전진과 계산 시간을 검사한다.
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  std::ifstream input(pkg+"/test/data/core_path_future_curve_r214.bin",std::ios::binary);ASSERT_TRUE(input.good());
  uint32_t count=0;input.read(reinterpret_cast<char*>(&count),sizeof(count));ASSERT_EQ(count,2u);
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/global_path.yaml",pkg)),map);
  const auto params=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
  path_planner::CorePathPlanner core(params,map);path_planner::CorePrediction prediction;
  for(uint32_t frame=0;frame<count;++frame) {
    double time=0;uint64_t ns=0;input.read(reinterpret_cast<char*>(&time),sizeof(time));input.read(reinterpret_cast<char*>(&ns),sizeof(ns));
    const auto ego=readMessage<nav_msgs::Odometry>(input);const auto status=readMessage<interfaces::CompetitionVehicleStatus>(input);
    const auto objects=readMessage<interfaces::Objects>(input);const auto tf=readMessage<tf2_msgs::TFMessage>(input);const auto imu=readMessage<sensor_msgs::Imu>(input);
    const auto section=readMessage<interfaces::SectionAnnotation>(input);const auto recorded=readMessage<interfaces::Path>(input);(void)recorded;ASSERT_TRUE(input.good());
    const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,path_planner::Checkpoints(checkpoints.begin()+section.state,checkpoints.end())));ASSERT_TRUE(static_cast<bool>(route));
    const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));ros::Time stamp;stamp.fromNSec(ns);
    const auto begin=std::chrono::steady_clock::now();const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    SCOPED_TRACE(time);
    testing::Test::RecordProperty("planning_elapsed_sec_"+std::to_string(frame),std::to_string(elapsed));
    EXPECT_LT(elapsed,.30);
    ASSERT_GE(path.points.size(),2u);
    EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
    EXPECT_NEAR(path.points.front().acceleration,frame==0 ? .46425215405889153 : .196089701,1e-4);
    EXPECT_NEAR(path.points.front().pose.pose.position.x,frame==0 ? 39.431669384510933 : 40.289865362533632,1e-4);
    EXPECT_EQ(path.points.front().time_from_start,ros::Duration(0));
    EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
    const auto future=std::min_element(path.points.begin(),path.points.end(),[](const auto& a,const auto& b){return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);});
    EXPECT_GT(future->velocity,8.0);
    EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-path.points.front().pose.pose.position.x,
                        path.points.back().pose.pose.position.y-path.points.front().pose.pose.position.y),30.0);
    for(std::size_t i=1;i<path.points.size();++i) {
      EXPECT_GT(path.points[i].time_from_start,path.points[i-1].time_from_start);
      EXPECT_TRUE(std::isfinite(path.points[i].velocity));
      EXPECT_GE(path.points[i].velocity,0);
      EXPECT_LE(path.points[i].velocity,60/3.6+1e-6);
    }
  }
}
}  // namespace
