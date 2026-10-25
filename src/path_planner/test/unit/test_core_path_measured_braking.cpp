#include <path_planner/cores/core_path_planner.hpp>
#include <gtest/gtest.h>

namespace {
const std::vector<stier::ReferencePoint> road{{{-20,0},-3,3},{{200,0},-3,3}};
stier::EgoState brakingState() {
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=12.219940185546875;
  ego.acceleration_mps2=-6.4001909135281876;return ego;
}
stier::Predictions clearScene() {stier::Predictions scene;scene.reference_stamp_sec=10;return scene;}

TEST(CorePathMeasuredBraking, ReleasesInheritedExcessAtMaximumJerkWithoutClippingTheInitialState) {
  const stier::PathParameters parameters;const auto ego=brakingState();
  path_planner::CorePathPlanner core(parameters);
  for(double cap:{60./3.6,10./3.6}) {
    stier::PlanningConstraints limits;limits.maximum_speed_mps=cap;
    const auto path=core.plan(ego,road,clearScene(),limits);
    ASSERT_GE(path.size(),2u);
    EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
    EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
    EXPECT_TRUE(core.checkPlannedPath(ego,road,clearScene(),limits,path));
    EXPECT_NEAR(path.back().time_sec,5,1e-8);
    const double release=(-parameters.max_deceleration_mps2-ego.acceleration_mps2)/parameters.max_longitudinal_jerk_mps3;
    for(std::size_t i=0;i<path.size();++i) {
      const auto& q=path[i];
      EXPECT_GE(q.speed_mps,0);
      if(q.time_sec<release)EXPECT_NEAR(q.acceleration_mps2,ego.acceleration_mps2+parameters.max_longitudinal_jerk_mps3*q.time_sec,1e-6);
      else EXPECT_GE(q.acceleration_mps2,-parameters.max_deceleration_mps2-1e-6);
      EXPECT_LE(q.acceleration_mps2,parameters.max_acceleration_mps2+1e-6);
      if(i)EXPECT_LE(std::abs(q.acceleration_mps2-path[i-1].acceleration_mps2)/(q.time_sec-path[i-1].time_sec),parameters.max_longitudinal_jerk_mps3+1e-6);
    }
  }
}
TEST(CorePathMeasuredBraking, RejectsAStateThatCannotReleaseBrakingBeforeReversing) {
  auto ego=brakingState();ego.speed_mps=4;
  EXPECT_TRUE(path_planner::CorePathPlanner{}.plan(ego,road,clearScene(),stier::PlanningConstraints{}).empty());
}
TEST(CorePathMeasuredBraking, RejectsMaintainingInheritedExcessEvenWithoutAnObstacle) {
  const auto ego=brakingState();std::vector<stier::PathPoint> path;
  for(double t:{0.,.1,.2,.3,.4,.5}) {
    stier::PathPoint point;
    point.time_sec=t;point.progress_m=ego.speed_mps*t+ego.acceleration_mps2*t*t/2;
    point.pose.position={point.progress_m,0};point.speed_mps=ego.speed_mps+ego.acceleration_mps2*t;
    point.acceleration_mps2=ego.acceleration_mps2;path.push_back(point);
  }
  EXPECT_FALSE(path_planner::CorePathPlanner{}.checkPlannedPath(
      ego,road,clearScene(),stier::PlanningConstraints{},path));
}
TEST(CorePathMeasuredBraking, DoesNotReleaseThroughABlockingObstacle) {
  const auto ego=brakingState();auto scene=clearScene();
  stier::PredictedObject object;object.id=9;object.kind=stier::ObjectKind::Static;
  object.pose.position={12,0};object.length_m=1;object.width_m=6;object.source_stamp_sec=10;
  scene.current.push_back(object);for(auto& sample:scene.future)sample.push_back(object);
  EXPECT_TRUE(path_planner::CorePathPlanner{}.plan(ego,road,scene,stier::PlanningConstraints{}).empty());
}
}  // namespace
