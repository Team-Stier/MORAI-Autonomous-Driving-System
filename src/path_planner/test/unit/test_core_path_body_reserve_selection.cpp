#include <boost/geometry.hpp>
#include <boost/geometry/io/wkt/read.hpp>
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
const std::string fixtures=std::string(NP_PACKAGE_DIR)+"/test/data/";
template<class T> T readMessage(std::ifstream& input) {
  uint32_t size=0;input.read(reinterpret_cast<char*>(&size),4);EXPECT_GT(size,0u);EXPECT_LT(size,1024u*1024u);
  if(!input || size==0 || size>=1024u*1024u)return T{};
  std::vector<uint8_t> bytes(size);input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),bytes.size());T msg;ros::serialization::deserialize(stream,msg);return msg;
}
struct Result { interfaces::Path path;double elapsed=0,initial_speed=0; };
Result scenario(bool block_last_frame) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  std::ifstream input(fixtures+"core_path_body_reserve_r215.bin",std::ios::binary);EXPECT_TRUE(input.good());if(!input)return {};
  uint32_t count=0;input.read(reinterpret_cast<char*>(&count),4);EXPECT_EQ(count,6u);if(count!=6)return {};
  const auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/global_path.yaml",pkg)),map);
  const auto params=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
  path_planner::CorePathPlanner core(params,map);path_planner::CorePrediction prediction;Result result;
  for(uint32_t i=0;i<count;++i) {
    double elapsed=0;uint64_t ns=0;input.read(reinterpret_cast<char*>(&elapsed),8);input.read(reinterpret_cast<char*>(&ns),8);
    const auto ego=readMessage<nav_msgs::Odometry>(input);const auto status=readMessage<interfaces::CompetitionVehicleStatus>(input);
    auto objects=readMessage<interfaces::Objects>(input);const auto tf=readMessage<tf2_msgs::TFMessage>(input);const auto imu=readMessage<sensor_msgs::Imu>(input);
    const auto section=readMessage<interfaces::SectionAnnotation>(input);const auto recorded=readMessage<interfaces::Path>(input);(void)recorded;EXPECT_TRUE(input.good());
    if(block_last_frame && i+1==count) {
      interfaces::Object block;block.id=999999;block.label="unknown";block.position=ego.pose.pose.position;
      block.bbox_pose=ego.pose.pose;block.bbox_size.x=2;block.bbox_size.y=2;block.bbox_size.z=2;
      block.velocity_available=true;block.acceleration_available=true;objects.objects.push_back(block);
    }
    const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,path_planner::Checkpoints(checkpoints.begin()+section.state,checkpoints.end())));
    EXPECT_TRUE(static_cast<bool>(route));if(!route)return {};
    const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));ros::Time stamp;stamp.fromNSec(ns);
    const auto begin=std::chrono::steady_clock::now();result.path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
    result.elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();result.initial_speed=status.velocity_x_mps;
  }
  return result;
}
TEST(CorePathBodyReserveSelection, KeepsExtraFootprintRoomBeforeTheCurveBecomesUnrecoverable) {
  // #CURVE_FOOTPRINT_RESERVE
  // R215 원본 입력과 해당 map 도로의 고정 WKT를 사용한다. 실제 크기와
  // 뒷차축 기준은 유지하며 출력 bbox를 각 차체 축에서 10cm 늘려 별도의
  // Boost polygon 연산으로 포함 여부를 검사한다. 실제 도로 밖에 경로를
  // 허용하거나 Core의 여유 검사 함수를 그대로 기대값으로 쓰지 않는다.
  const auto result=scenario(false);ASSERT_GE(result.path.points.size(),2u);
  EXPECT_DOUBLE_EQ(result.path.points.front().velocity,result.initial_speed);
  EXPECT_GT(result.path.points.back().time_from_start.toSec(),4.8);
  EXPECT_GT(result.path.points.back().velocity,0);
  EXPECT_LT(result.elapsed,.30);RecordProperty("planning_elapsed_sec",std::to_string(result.elapsed));
  using Point=boost::geometry::model::d2::point_xy<double>;using Polygon=boost::geometry::model::polygon<Point>;
  boost::geometry::model::multi_polygon<Polygon> road;
  std::ifstream file(fixtures+"core_path_body_reserve_r215_road.wkt");ASSERT_TRUE(file.good());
  const std::string wkt((std::istreambuf_iterator<char>(file)),{});boost::geometry::read_wkt(wkt,road);ASSERT_FALSE(road.empty());
  std::size_t checked=0;
  for(const auto& p:result.path.points) {
    if(p.time_from_start.toSec()>1.0)break;
    const double yaw=2*std::atan2(p.pose.pose.orientation.z,p.pose.pose.orientation.w),c=std::cos(yaw),s=std::sin(yaw);
    Polygon body;
    for(const auto& xy:std::vector<std::pair<double,double>>{{-.890,-1.1804175},{3.945008,-1.1804175},{3.945008,1.1804175},{-.890,1.1804175},{-.890,-1.1804175}})
      body.outer().emplace_back(p.pose.pose.position.x+c*xy.first-s*xy.second,p.pose.pose.position.y+s*xy.first+c*xy.second);
    boost::geometry::correct(body);std::vector<Polygon> outside;boost::geometry::difference(body,road,outside);
    double area=0;for(const auto& polygon:outside)area+=std::abs(boost::geometry::area(polygon));
    EXPECT_LE(area,1e-6)<<"planned time "<<p.time_from_start.toSec();++checked;
  }
  EXPECT_GT(checked,5u);
}
TEST(CorePathBodyReserveSelection, RechecksNewCollisionInsteadOfPublishingTheEarlierMovingPath) {
  // 같은 이동 이력 뒤 현재 차체와 겹치는 새 bbox를 관측하면 경로가 없다.
  // 차체 여유 선호가 원본 객체·현재 상태의 충돌 검사를 우회하지 않아야 한다.
  const auto result=scenario(true);EXPECT_TRUE(result.path.points.empty());
}
}  // namespace
