#include <chrono>
#include <fstream>
#include <gtest/gtest.h>
#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/core_prediction.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <path_planner/parameters/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <interfaces/SectionAnnotation.h>
#include <ros/serialization.h>
namespace {
const std::string fixture=std::string(NP_PACKAGE_DIR)+"/test/data/core_path_braking_event_r216.bin";
constexpr double recorded_acceleration=-7.0010172838224145;
template<class T> T readMessage(std::ifstream& input) {
  uint32_t size=0;input.read(reinterpret_cast<char*>(&size),4);EXPECT_GT(size,0u);EXPECT_LT(size,1024u*1024u);
  if(!input || size==0 || size>=1024u*1024u)return T{};
  std::vector<uint8_t> bytes(size);input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),bytes.size());T msg;ros::serialization::deserialize(stream,msg);return msg;
}
struct Result {interfaces::Path path;double speed=0,elapsed=0;};
Result evaluate(double acceleration,bool low_speed=false) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;std::ifstream input(fixture,std::ios::binary);EXPECT_TRUE(input.good());if(!input)return {};
  uint32_t count=0;input.read(reinterpret_cast<char*>(&count),4);EXPECT_EQ(count,1u);double t=0;uint64_t ns=0;input.read(reinterpret_cast<char*>(&t),8);input.read(reinterpret_cast<char*>(&ns),8);
  auto ego=readMessage<nav_msgs::Odometry>(input);auto status=readMessage<interfaces::CompetitionVehicleStatus>(input);
  const auto objects=readMessage<interfaces::Objects>(input);const auto tf=readMessage<tf2_msgs::TFMessage>(input);auto imu=readMessage<sensor_msgs::Imu>(input);
  const auto section=readMessage<interfaces::SectionAnnotation>(input);const auto recorded=readMessage<interfaces::Path>(input);(void)recorded;EXPECT_TRUE(input.good());
  imu.linear_acceleration.x+=acceleration-recorded_acceleration;
  if(low_speed) {const double scale=.5/status.velocity_x_mps;status.velocity_x_mps=.5;status.signed_velocity_mps=.5;ego.twist.twist.linear.x=.5;ego.twist.twist.angular.z*=scale;}
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/global_path.yaml",pkg)),map);
  const auto params=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
  path_planner::CorePathPlanner core(params,map);path_planner::CorePrediction prediction;
  const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,path_planner::Checkpoints(checkpoints.begin()+section.state,checkpoints.end())));EXPECT_TRUE(static_cast<bool>(route));if(!route)return {};
  const auto predicted=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));ros::Time stamp;stamp.fromNSec(ns);
  const auto begin=std::chrono::steady_clock::now();const auto path=core.process(ego,status,objects,tf,imu,route,predicted,stamp);
  return {path,status.velocity_x_mps,std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()};
}
TEST(CorePathBrakingEventClock, PreservesFeasibleMeasuredBrakeReleaseNearTheRegularTimeGrid) {
  // #BRAKING_EVENT_CLOCK
  // R216 실측 제동 해제 시각 0.5002543209556036초와 정규 0.5초 격자가
  // 인접한 원본 입력이다. jerk 4로 해제하면 최저 속도가 약 3.995m/s라
  // 전진 계획이 가능하다. 실측 초기 운동과 원본 객체·도로를 유지한다.
  // 같은 경계 양쪽의 작은 가속도 차이도 검사해 특정 실측 값만 고치지 않는다.
  for(double acceleration:{recorded_acceleration,-7.0002,-6.9998,-7.01}) {
    SCOPED_TRACE(acceleration);const auto result=evaluate(acceleration);ASSERT_GE(result.path.points.size(),2u);
    EXPECT_DOUBLE_EQ(result.path.points.front().velocity,result.speed);
    EXPECT_NEAR(result.path.points.front().acceleration,acceleration,1e-10);
    EXPECT_EQ(result.path.points.front().time_from_start,ros::Duration(0));
    EXPECT_GT(result.path.points.back().time_from_start.toSec(),4.8);EXPECT_LT(result.elapsed,.30);
    const double event=(-5-acceleration)/4;
    for(std::size_t i=0;i<result.path.points.size();++i) {
      const auto& point=result.path.points[i];const double time=point.time_from_start.toSec();
      EXPECT_TRUE(std::isfinite(point.velocity));EXPECT_GE(point.velocity,0);
      if(time<event)EXPECT_NEAR(point.acceleration,acceleration+4*time,1e-7);
      else EXPECT_GE(point.acceleration,-5-1e-7);
      if(i) {const auto& before=result.path.points[i-1];const double dt=time-before.time_from_start.toSec();EXPECT_GT(dt,0);EXPECT_LE(std::abs(point.acceleration-before.acceleration),4*dt+1e-7);}
    }
    EXPECT_GT(std::hypot(result.path.points.back().pose.pose.position.x-result.path.points.front().pose.pose.position.x,
                        result.path.points.back().pose.pose.position.y-result.path.points.front().pose.pose.position.y),5.0);
  }
}
TEST(CorePathBrakingEventClock, DoesNotInventForwardMotionWhenBrakeReleaseWouldRequireReverse) {
  // 0.5m/s에서 같은 실측 제동을 jerk 4로 해제하면 음의 속도가 불가피하다.
  // 시간 격자 보정이 초기 가속도를 자르거나 jerk/전진 조건을 풀어서는 안 된다.
  const auto result=evaluate(recorded_acceleration,true);EXPECT_TRUE(result.path.points.empty());
}
}  // namespace
