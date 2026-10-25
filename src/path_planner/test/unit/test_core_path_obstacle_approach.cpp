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
  uint32_t size=0;
  input.read(reinterpret_cast<char*>(&size),sizeof(size));
  EXPECT_GT(size,0u);
  EXPECT_LT(size,1024u*1024u);
  if(!input || size==0 || size>=1024u*1024u) return T{};
  std::vector<uint8_t> bytes(size);
  input.read(reinterpret_cast<char*>(bytes.data()),size);
  ros::serialization::IStream stream(bytes.data(),bytes.size());
  T message;ros::serialization::deserialize(stream,message);return message;
}
void verifyApproach(const std::string& fixture,std::size_t section=4,double speed_fraction=.9) {
  // #OBSTACLE_APPROACH
  // R171의 실제 조립 입력을 원본 시각·객체·IMU·차량 운동 그대로 재생한다.
  // 장애물 관측을 제거하거나 초기 속도를 낮춰 통과시키지 않는다. 관측된
  // 2초 내 불필요한 급감속을 재현하며, 5초 예측과 최초 속도 계약도 유지한다.
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  std::ifstream input(package+"/test/data/"+fixture,std::ios::binary);
  ASSERT_TRUE(input.good());
  uint64_t ns=0;input.read(reinterpret_cast<char*>(&ns),sizeof(ns));
  const auto ego=readMessage<nav_msgs::Odometry>(input);
  const auto status=readMessage<interfaces::CompetitionVehicleStatus>(input);
  const auto objects=readMessage<interfaces::Objects>(input);
  const auto tf=readMessage<tf2_msgs::TFMessage>(input);
  const auto imu=readMessage<sensor_msgs::Imu>(input);
  ASSERT_TRUE(input.good());
  ASSERT_FALSE(objects.objects.empty());
  const auto reader=node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/shared_map.yaml",package);
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(path_planner::loadGlobalPathParameters(
      node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/global_path.yaml",package)),map);
  const path_planner::Checkpoints remaining(checkpoints.begin()+section,checkpoints.end());
  const auto route=std::get<5>(global.process(ego,status,objects,tf,imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePrediction prediction;
  const auto predictions=std::get<6>(prediction.process(ego,status,objects,tf,imu,route));
  path_planner::CorePathPlanner core(path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/core_path_planner.yaml",package)),map);
  ros::Time stamp;stamp.fromNSec(ns);
  const auto path=core.process(ego,status,objects,tf,imu,route,predictions,stamp);
  ASSERT_GE(path.points.size(),2u);
  const auto two=std::min_element(path.points.begin(),path.points.end(),
      [](const interfaces::PathPoint& a,const interfaces::PathPoint& b) {
        return std::abs(a.time_from_start.toSec()-2)<std::abs(b.time_from_start.toSec()-2);
      });
  EXPECT_GE(two->velocity,speed_fraction*status.velocity_x_mps);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GE(path.points.back().time_from_start.toSec(),4.8);
}
TEST(CorePathObstacleApproach, DistantStaticBlockerDoesNotDiscardTheSafeTimedApproach) {
  verifyApproach("core_path_distant_static_r171.bin");
}
TEST(CorePathObstacleApproach, ReducedSearchDoesNotAcceptAnUnnecessaryNearStop) {
  verifyApproach("core_path_static_beam_r171.bin");
}
TEST(CorePathObstacleApproach, RemoteObjectsCannotPreemptTheNormalForwardCurve) {
  // R195의 원본 객체·자세·속도·시각을 유지한다. 가장 가까운 객체도 250m
  // 이상 떨어진 상태에서 장애물 전용 후보가 일반 곡선 연결을 선점해
  // 2초 뒤 속도를 8.62→3.88m/s로 낮췄다. 실제 충돌 검사를 유지한 공통
  // 탐색은 같은 도로에서 약 7.93m/s의 전진 경로를 계산할 수 있다.
  verifyApproach("core_path_remote_objects_r195.bin",1,.8);
}
TEST(CorePathObstacleApproach, MovingTrafficDoesNotPullTheClearExitToANearStop) {
  // R171 t=378.552: 출구에서 차량들이 뒤에 있고 자차는 가속 중인 실제 입력이다.
  // 합류 직전 가까운 순환 차량을 양보하던 t=371.652와 구분한다. 모든 차량의
  // 원본 위치·속도·예측을 유지해 출구의 일시적인 불필요 감속을 재현한다.
  verifyApproach("core_path_roundabout_exit_r171.bin",9);
}

stier::Predictions followingScene(const stier::PathParameters& parameters,double gap,double speed) {
  stier::Predictions scene;scene.reference_stamp_sec=10;
  stier::PredictedObject lead;lead.id=1;lead.kind=stier::ObjectKind::Vehicle;
  lead.pose.position={parameters.vehicle.front_m+2+parameters.obstacle_margin_m+gap,0};
  lead.length_m=4;lead.width_m=2;lead.velocity_mps={speed,0};lead.source_stamp_sec=10;
  scene.current.push_back(lead);
  for(std::size_t i=0;i<scene.future.size();++i) {
    auto future=lead;future.pose.position.x+=speed*.5*(i+1);
    scene.future[i].push_back(future);
  }
  return scene;
}

void verifyRelativeFollowing(double gap,double lead_speed,bool separating) {
  // map 차체의 실제 범퍼 간격과 절대 속도로 예측한다. 여유가 벌어지는
  // 앞차를 불필요하게 따라 제동하는 경우와, 느린 앞차에 늦게 접근하는
  // 경우를 분리한다. 실측 v/a와 전체 충돌·도로·동역학 검사는 유지한다.
  stier::PathParameters parameters;parameters.max_speed_mps=20;
  parameters.max_lateral_accel_mps2=3;parameters.trajectory_horizon_m=150;
  path_planner::CorePathPlanner core(parameters);
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=9;
  const auto scene=followingScene(parameters,gap,lead_speed);
  stier::PlanningConstraints limits;limits.maximum_speed_mps=20;
  const auto path=core.plan(ego,road,scene,limits);
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
  EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
  double minimum=ego.speed_mps;
  for(const auto& point:path)if(point.time_sec<=1)minimum=std::min(minimum,point.speed_mps);
  if(separating)EXPECT_GE(minimum,ego.speed_mps-1e-8);
  else EXPECT_LT(minimum,8.75);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
}
TEST(CorePathObstacleApproach, SeparatingLeadPreservesTheFeasibleDepartureSpeed) {
  verifyRelativeFollowing(12,12,true);
}
TEST(CorePathObstacleApproach, ClosingLeadStartsBrakingWithAdditionalResponseDistance) {
  verifyRelativeFollowing(20,5,false);
}

TEST(CorePathObstacleApproach, LowSpeedEqualLeadDoesNotForceAnUnnecessarySpeedDip) {
  // 저속에서 앞차와 동일한 3m/s로 6m 간격을 유지하는 장면이다.
  // 1.5초의 고정 추종 목표가 이 간격을 6.5m로 벌리기 위해 감속하던
  // 현상을 재현한다. 출발 상태와 전체 충돌·도로·동역학 검사는 유지한다.
  const std::string package=NP_PACKAGE_DIR;
  auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  path_planner::CorePathPlanner core(parameters);
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=3;
  const auto scene=followingScene(parameters,6,3);
  stier::PlanningConstraints limits;limits.maximum_speed_mps=10;
  const auto path=core.plan(ego,road,scene,limits);
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,3);
  for(const auto& point:path)if(point.time_sec<=1)EXPECT_GE(point.speed_mps,3-1e-8);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
}

TEST(CorePathObstacleApproach, StaticStopRetainsTheExistingFollowingStandstillMargin) {
  // 동일한 실제 bbox 앞에서 움직이는 앞차의 정지 여유(2m)를 정적 객체에도
  // 확보한다. 넓은 감속 상한 대신 좁은 도로의 최종 정지 위치를 검사한다.
  stier::PathParameters parameters;parameters.trajectory_horizon_m=150;
  path_planner::CorePathPlanner core(parameters);
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=9;
  auto scene=followingScene(parameters,20,0);
  scene.current[0].kind=stier::ObjectKind::Static;
  for(auto& future:scene.future)future[0].kind=stier::ObjectKind::Static;
  const stier::PlanningConstraints limits;
  const auto path=core.plan(ego,road,scene,limits);
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.back().speed_mps,0);
  const double gap=scene.current[0].pose.position.x-2-path.back().pose.position.x-parameters.vehicle.front_m;
  EXPECT_GE(gap,2.25-1e-4);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
}

TEST(CorePathObstacleApproach, AdditionalStaticMarginDoesNotDiscardACloseFeasibleStop) {
  stier::PathParameters parameters;parameters.trajectory_horizon_m=150;
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
  for(const auto& initial:std::vector<std::pair<double,double>>{{9,14.5},{0,1}}) {
    path_planner::CorePathPlanner core(parameters);
    stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=initial.first;
    auto scene=followingScene(parameters,initial.second,0);
    scene.current[0].kind=stier::ObjectKind::Static;
    for(auto& future:scene.future)future[0].kind=stier::ObjectKind::Static;
    const stier::PlanningConstraints limits;
    const auto path=core.plan(ego,road,scene,limits);
    ASSERT_GE(path.size(),2u);
    EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
    EXPECT_DOUBLE_EQ(path.back().speed_mps,0);
    EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
  }
}
TEST(CorePathObstacleApproach, LowSpeedClosingLeadStillProducesAValidatedBrakePlan) {
  const std::string package=NP_PACKAGE_DIR;
  const auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  path_planner::CorePathPlanner core(parameters);
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
  for(double lead_speed:{0.,1.}) {
    stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=3;
    const auto scene=followingScene(parameters,6,lead_speed);
    stier::PlanningConstraints limits;limits.maximum_speed_mps=10;
    const auto path=core.plan(ego,road,scene,limits);
    ASSERT_GE(path.size(),2u);
    double minimum=3;
    for(const auto& point:path)if(point.time_sec<=1)minimum=std::min(minimum,point.speed_mps);
    EXPECT_LT(minimum,2.8);
    EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
    if(lead_speed==0)EXPECT_DOUBLE_EQ(path.back().speed_mps,0);
  }
}

TEST(CorePathObstacleApproach, AboveTheLowSpeedRangePreservesTheLegacyFollowingProfile) {
  const std::string package=NP_PACKAGE_DIR;
  auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=9;
  const auto scene=followingScene(parameters,16,9);
  stier::PlanningConstraints limits;limits.maximum_speed_mps=20;
  path_planner::CorePathPlanner adjusted(parameters);
  const auto path=adjusted.plan(ego,road,scene,limits);
  parameters.following_low_speed_headway_sec=parameters.following_time_headway_sec;
  path_planner::CorePathPlanner legacy(parameters);
  const auto reference=legacy.plan(ego,road,scene,limits);
  ASSERT_GE(path.size(),2u);ASSERT_EQ(path.size(),reference.size());
  for(std::size_t i=0;i<path.size();++i) {
    EXPECT_DOUBLE_EQ(path[i].speed_mps,reference[i].speed_mps);
    EXPECT_DOUBLE_EQ(path[i].acceleration_mps2,reference[i].acceleration_mps2);
  }
  EXPECT_TRUE(adjusted.checkPlannedPath(ego,road,scene,limits,path));
}


TEST(CorePathObstacleApproach, CloseButSeparatingLeadDoesNotRequestYieldBraking) {
  verifyRelativeFollowing(3,12,true);
}
TEST(CorePathObstacleApproach, CloseSlowerLeadStillRequiresBraking) {
  stier::PathParameters parameters;parameters.max_lateral_accel_mps2=3;
  path_planner::CorePathPlanner core(parameters);
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=3;
  const auto scene=followingScene(parameters,5,2);
  const auto path=core.plan(ego,road,scene,stier::PlanningConstraints{});
  ASSERT_GE(path.size(),2u);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,stier::PlanningConstraints{},path));
  EXPECT_LT(path[std::min<std::size_t>(10,path.size()-1)].speed_mps,ego.speed_mps);
}
}  // namespace
