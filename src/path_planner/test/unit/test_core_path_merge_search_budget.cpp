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

TEST(CorePathMergeSearchBudget, KeepsTheMovingTrajectoryAvailableBeforeTheControlInputExpires) {
  // #HIGHWAY_MERGE_INPUT_DEADLINE
  // R211 합류의 연속 원본 입력을 보존한다. 정상 감속 경로가 늦게 발행되어
  // 입력 만료→강제동→빈 경로로 이어지는 회귀를 검출한다. 원본 객체와
  // map 뒷차축 운동, 예측 및 안전 한도를 유지한 이동 참조와 계산 시간을 검사한다.
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  std::ifstream input(pkg+"/test/data/core_path_merge_search_r211.bin",std::ios::binary);ASSERT_TRUE(input.good());
  uint32_t count=0;input.read(reinterpret_cast<char*>(&count),sizeof(count));ASSERT_GT(count,0u);ASSERT_LT(count,30u);
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/global_path.yaml",pkg)),map);
  const auto params=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
  path_planner::CorePathPlanner core(params,map);path_planner::CorePrediction prediction;
  bool checked=false;double previous_elapsed=0;
  for(uint32_t frame=0;frame<count;++frame) {
    double time=0;uint64_t ns=0;input.read(reinterpret_cast<char*>(&time),sizeof(time));input.read(reinterpret_cast<char*>(&ns),sizeof(ns));
    const auto ego=readMessage<nav_msgs::Odometry>(input);const auto status=readMessage<interfaces::CompetitionVehicleStatus>(input);
    const auto objects=readMessage<interfaces::Objects>(input);const auto tf=readMessage<tf2_msgs::TFMessage>(input);const auto imu=readMessage<sensor_msgs::Imu>(input);
    const auto section=readMessage<interfaces::SectionAnnotation>(input);const auto recorded=readMessage<interfaces::Path>(input);(void)recorded;ASSERT_TRUE(input.good());
    const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,path_planner::Checkpoints(checkpoints.begin()+section.state,checkpoints.end())));ASSERT_TRUE(static_cast<bool>(route));
    const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));ros::Time stamp;stamp.fromNSec(ns);
    const auto begin=std::chrono::steady_clock::now();const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    if(std::abs(time-144.7591248)<1e-5) {
      checked=true;testing::Test::RecordProperty("planning_elapsed_sec",std::to_string(elapsed));testing::Test::RecordProperty("consecutive_elapsed_sec",std::to_string(elapsed+previous_elapsed));
      EXPECT_LT(elapsed,.25);EXPECT_LT(elapsed+previous_elapsed,.40);ASSERT_GE(path.points.size(),2u);
      const auto future=std::min_element(path.points.begin(),path.points.end(),[](const auto& a,const auto& b){return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);});
      EXPECT_GE(future->velocity,40/3.6);EXPECT_GT(path.points.back().time_from_start.toSec(),4.8);
      EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
      ASSERT_EQ(path.points.size(),recorded.points.size());
      for(std::size_t i=0;i<path.points.size();++i) {
        EXPECT_NEAR(path.points[i].pose.pose.position.x,recorded.points[i].pose.pose.position.x,1e-6);
        EXPECT_NEAR(path.points[i].pose.pose.position.y,recorded.points[i].pose.pose.position.y,1e-6);
        EXPECT_NEAR(path.points[i].velocity,recorded.points[i].velocity,1e-6);
        EXPECT_NEAR(path.points[i].acceleration,recorded.points[i].acceleration,1e-6);
        EXPECT_EQ(path.points[i].time_from_start,recorded.points[i].time_from_start);
      }
    }
    previous_elapsed=elapsed;
  }
  EXPECT_TRUE(checked);
}
}  // namespace
