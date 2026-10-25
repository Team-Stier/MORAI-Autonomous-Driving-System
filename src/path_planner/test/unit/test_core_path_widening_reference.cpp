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

TEST(CorePathWideningReference, UsesTheAvailableParallelRoadBeforeSlowingForTheLaneChange) {
  // #WIDENING_LANE_CHANGE_REFERENCE
  // R217에서 실제 횡가속도가 한도 안에 있고 가속 중이던 원본 입력이다.
  // 80m 이상 연결된 출발/목표 도로를 50m 참조 전환에 압축해서,
  // 접근 중 속도부터 줄이는 회귀를 검출한다. 원본 객체와 5초 예측,
  // map 뒷차축 초기 운동 및 최종 전체 도로/충돌/물리 검사는 유지한다.
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  std::ifstream input(pkg+"/test/data/core_path_widening_reference_r217.bin",std::ios::binary);
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
  ASSERT_GE(path.points.size(),2u);(void)recorded;
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);EXPECT_GT(path.points.back().velocity,0);
  const auto future=std::min_element(path.points.begin(),path.points.end(),[](const auto& a,const auto& b){
    return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);
  });
  RecordProperty("initial_speed_mps",std::to_string(status.velocity_x_mps));
  RecordProperty("two_second_speed_mps",std::to_string(future->velocity));
  EXPECT_GE(future->velocity,.98*status.velocity_x_mps);

  // 빈 확장 차로를 가정해 관측 객체를 무시하는 변경은 허용하지 않는다.
  // 같은 이동 이력 뒤 차체와 겹치는 새 bbox가 생기면 기존 충돌 검사로 거부한다.
  auto blocked_objects=objects;interfaces::Object block;
  block.id=999999;block.label="unknown";block.position=ego.pose.pose.position;
  block.bbox_pose=ego.pose.pose;block.bbox_size.x=2;block.bbox_size.y=2;block.bbox_size.z=2;
  block.velocity_available=true;block.acceleration_available=true;
  blocked_objects.objects.push_back(block);
  const auto blocked_predictions=std::get<6>(prediction.process(ego,status,blocked_objects,tf,imu,route));
  EXPECT_TRUE(core.process(ego,status,blocked_objects,tf,imu,route,blocked_predictions,stamp).points.empty());

}
}  // namespace
