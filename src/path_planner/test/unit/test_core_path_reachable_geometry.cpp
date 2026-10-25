#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/core_prediction.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/global_path.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <interfaces/SectionAnnotation.h>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <fstream>

namespace {
template<class T> T readRecordedMessage(std::ifstream& input) {
  uint32_t size=0;
  input.read(reinterpret_cast<char*>(&size),sizeof(size));
  if(!input || size==0 || size>=1024u*1024u) {
    ADD_FAILURE()<<"Invalid recorded message length";
    return T{};
  }
  std::vector<uint8_t> bytes(size);
  input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),size);
  T message;
  ros::serialization::deserialize(stream,message);
  return message;
}

void verifyRecordedCurve(const std::string& fixture,uint32_t expected_count,bool braking_release=false) {
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  std::ifstream input(package+"/test/data/core_path_reachable_geometry_"+fixture+".bin",std::ios::binary);
  ASSERT_TRUE(input.good());
  uint32_t count=0;
  input.read(reinterpret_cast<char*>(&count),sizeof(count));
  ASSERT_EQ(count,expected_count);
  const auto reader=node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/shared_map.yaml",package);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/global_path.yaml",package)),map);
  const auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  path_planner::CorePathPlanner core(parameters,map);
  path_planner::CorePrediction prediction;
  bool inherited_excess=false;
  for(uint32_t frame=0;frame<count;++frame) {
    SCOPED_TRACE(frame);
    double relative_time=0;
    uint64_t ns=0;
    input.read(reinterpret_cast<char*>(&relative_time),sizeof(relative_time));
    input.read(reinterpret_cast<char*>(&ns),sizeof(ns));
    const auto ego=readRecordedMessage<nav_msgs::Odometry>(input);
    const auto status=readRecordedMessage<interfaces::CompetitionVehicleStatus>(input);
    const auto objects=readRecordedMessage<interfaces::Objects>(input);
    const auto tf=readRecordedMessage<tf2_msgs::TFMessage>(input);
    const auto imu=readRecordedMessage<sensor_msgs::Imu>(input);
    const auto section=readRecordedMessage<interfaces::SectionAnnotation>(input);
    readRecordedMessage<interfaces::Path>(input);
    ASSERT_TRUE(input.good());
    const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,
        path_planner::Checkpoints(checkpoints.begin()+section.state,checkpoints.end())));
    ASSERT_TRUE(static_cast<bool>(route));
    const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));
    ros::Time stamp;
    stamp.fromNSec(ns);
    const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
    ASSERT_GE(path.points.size(),2u);
    EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
    EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
    const auto& orientation=ego.pose.pose.orientation;
    const Eigen::Quaterniond rotation(orientation.w,orientation.x,orientation.y,orientation.z);
    const double measured_acceleration=imu.linear_acceleration.x-
        (rotation.conjugate()*Eigen::Vector3d(0,0,9.81)).x();
    if(braking_release) {
      EXPECT_NEAR(path.points.front().acceleration,measured_acceleration,1e-8);
      inherited_excess=inherited_excess || measured_acceleration<-parameters.max_deceleration_mps2;
    }
    for(const auto& point:path.points) {
      EXPECT_TRUE(std::isfinite(point.velocity));
      EXPECT_GE(point.velocity,0);
      EXPECT_LE(point.acceleration,parameters.max_acceleration_mps2+1e-6);
      if(braking_release) {
        const double floor=std::min(-parameters.max_deceleration_mps2,
            measured_acceleration+parameters.max_longitudinal_jerk_mps3*point.time_from_start.toSec());
        EXPECT_GE(point.acceleration,floor-1e-6);
      } else EXPECT_GE(point.acceleration,-parameters.max_deceleration_mps2-1e-6);
    }
  }
  if(braking_release)EXPECT_TRUE(inherited_excess);
}
TEST(CorePathReachableGeometry, KeepsAFeasibleCurveWithoutInstantaneousSpeedReduction) {
  // R182에서 직전 경로 대비 횡오차 1.2cm·방향 오차 0.18도인 첫 단절까지
  // 네 입력 묶음을 원본 시각·객체·운동 상태 그대로 재생한다. 동일한 전체
  // 경계·충돌·동역학 검사를 통과하는 별도 계산 경로가 확인된 사례다.
  verifyRecordedCurve("r182",4);
}
TEST(CorePathReachableGeometry, KeepsAFeasibleCurveBeforeTheFootprintMarginIsExhausted) {
  // R186의 마지막 정상 다섯 입력과 첫 단절을 원본 그대로 재생한다.
  // 기존 경로의 첫 1초 도로 여유가 5.4cm까지 줄어든 뒤 경로가 비었다.
  // 이 마지막 상태에서도 같은 최종 검사를 통과하는 C2 후보가 확인됐다.
  // 제어·시각·속도·5cm 경계를 바꾸지 않고 매 호출의 전방 경로를 요구한다.
  verifyRecordedCurve("r186",6);
}
TEST(CorePathReachableGeometry, PreservesNativeBrakingInputWhileImmediatelyReleasingTheMeasuredExcess) {
  // 정상 FPS 주행 R188의 마지막 다섯 정상 입력과 첫 빈 경로를 재생한다.
  // 실측 -6.40019m/s²를 보존하며 기존 jerk 4m/s³로 한도 -5까지 즉시
  // 해제해야 한다. 객체·지도·시각·차체·초기 속도는 원본 그대로 사용한다.
  verifyRecordedCurve("r188",6,true);
}
TEST(CorePathReachableGeometry, BrakingEnablesTheLaterCurvatureOfTheRoundaboutExit) {
  // R196 교차로 출구의 마지막 정상 여섯 입력과 첫 단절을 재생한다.
  // 원본 객체·시각·실측 가속도와 곡률을 유지한 채 감속으로 가능한
  // 후반 곡률을 제안하고, 5초 전체 경로와 기존 동역학 한도를 검사한다.
  verifyRecordedCurve("r196_section9",7);
}
}  // namespace
