#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <gtest/gtest.h>

namespace {
stier::PathParameters configuredParameters() {
  const std::string package=NP_PACKAGE_DIR;
  return path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
}
stier::Predictions pedestrian(double x,double y,double width=.69) {
  stier::Predictions scene;scene.reference_stamp_sec=10;
  stier::PredictedObject object;object.id=37;object.kind=stier::ObjectKind::Pedestrian;
  object.pose.position={x,y};object.length_m=.3825;object.width_m=width;object.source_stamp_sec=10;
  scene.current.push_back(object);
  for(auto& sample:scene.future)sample.push_back(object);
  return scene;
}
const std::vector<stier::ReferencePoint> road{{{-20,0},-1.88919,1.88919},{{200,0},-1.88919,1.88919}};
stier::EgoState initial() {
  stier::EgoState state;state.stamp_sec=10;state.speed_mps=14;state.acceleration_mps2=.5;return state;
}
double speedAt(const std::vector<stier::PathPoint>& path,double time) {
  return std::min_element(path.begin(),path.end(),[&](const stier::PathPoint& a,const stier::PathPoint& b) {
    return std::abs(a.time_sec-time)<std::abs(b.time_sec-time);
  })->speed_mps;
}
TEST(CorePathPedestrianApproach, StartsRoadsideCautionBeforeCrossingRatherThanWaitingForBoxAnimation) {
  // 실측 R178과 같은 도로 옆 거리에서 보행자가 아직 정지해 있는 경우다.
  // bbox 크기는 충돌 검사에 그대로 사용하고 실제 초기 운동량도 보존한다.
  const auto parameters=configuredParameters();path_planner::CorePathPlanner core(parameters);
  const auto ego=initial();const auto objects=pedestrian(33.42,-3.62);
  stier::PlanningConstraints limits;limits.maximum_speed_mps=60./3.6;
  const auto path=core.plan(ego,road,objects,limits);
  ASSERT_GE(path.size(),2u);EXPECT_LT(speedAt(path,2),8);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,14);EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,.5);
  EXPECT_LE(path.back().speed_mps,10./3.6+1e-6);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,objects,limits,path));
}
TEST(CorePathPedestrianApproach, InitialCautionIsStableAcrossObservedBoxWidths) {
  const auto parameters=configuredParameters();const auto ego=initial();
  stier::PlanningConstraints limits;limits.maximum_speed_mps=60./3.6;
  double previous=-1;
  for(double width:{.69,.75}) {
    path_planner::CorePathPlanner core(parameters);const auto scene=pedestrian(33.42,-3.62,width);
    const auto path=core.plan(ego,road,scene,limits);ASSERT_GE(path.size(),2u);
    const double speed=speedAt(path,2);EXPECT_LT(speed,8);
    if(previous>=0)EXPECT_NEAR(speed,previous,1e-8);previous=speed;
    EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
  }
}
TEST(CorePathPedestrianApproach, RemoteSidewalkOrPassedPedestrianDoesNotSlowTheClearRoad) {
  const auto parameters=configuredParameters();const auto ego=initial();
  stier::PlanningConstraints limits;limits.maximum_speed_mps=60./3.6;
  for(const auto& position:std::vector<stier::Vec2>{{33.42,-12},{-10,-3.62}}) {
    path_planner::CorePathPlanner core(parameters);const auto scene=pedestrian(position.x,position.y);
    const auto path=core.plan(ego,road,scene,limits);ASSERT_GE(path.size(),2u);
    EXPECT_GE(speedAt(path,2),ego.speed_mps);EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
  }
}
TEST(CorePathPedestrianApproach, StationaryPedestrianOnTheRoadStillRequiresAValidatedStop) {
  const auto parameters=configuredParameters();path_planner::CorePathPlanner core(parameters);
  auto ego=initial();ego.speed_mps=4;ego.acceleration_mps2=0;
  const auto scene=pedestrian(14,0);stier::PlanningConstraints limits;limits.maximum_speed_mps=60./3.6;
  const auto path=core.plan(ego,road,scene,limits);ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.back().speed_mps,0);
  EXPECT_LT(path.back().pose.position.x+parameters.vehicle.front_m+parameters.obstacle_margin_m,13.8);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
}
}  // namespace
