#include <sstream>
#include <sensor_msgs/Imu.h>
#include <path_planner/libraries/types.hpp>
#include <Eigen/Geometry>
// Standalone core regression: no ROS node, message conversion, or simulator.
// Compile with C++14, Eigen3, GoogleTest, and src/path_planner/include.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
// Inspect existing numerical helpers without adding a testing API to the core.
#define private public
#include <path_planner/cores/core_path_planner.hpp>
#undef private
#include <gtest/gtest.h>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/core_path_planner.hpp>

namespace {
using namespace stier;
using PathGenerationCore = path_planner::CorePathPlanner;
using geometry = path_planner::CorePathPlanner::Geometry;

TEST(CorePathMeasuredRecovery, KeepsMeasuredExcessAndBrakesAtTheExistingJerkBound) {
  PathGenerationCore core;
  const EgoState ego{{{0,0},0},0,10.2,0,.02};
  PlanningConstraints limits;limits.maximum_speed_mps=10;
  const auto recovery=core.ceilingRecovery(ego,limits);
  ASSERT_EQ(recovery.count,2u);
  const auto& phase=recovery.phase[0];
  const double v=phase.speed(phase.h),a=phase.a+phase.j*phase.h;
  const PathGenerationCore::SpeedEdge edge{0,10.2,0,v,a,0,phase.h};
  // 초기 실측 2.0808m/s²를 보존하되 기존 jerk=4의 가장 빠른 감속
  // 포락선 안에서 한도로 복귀하는 후보는 선두 초과만으로 버리지 않는다.
  EXPECT_TRUE(core.speedAdmissible(edge,ego,limits,.02,recovery));
}
TEST(CorePathMeasuredRecovery, DoesNotAcceptHoldingAnExcessInsteadOfRecovering) {
  PathGenerationCore core;const EgoState ego{{{0,0},0},0,10.2,0,.02};
  PlanningConstraints limits;limits.maximum_speed_mps=10;
  const PathGenerationCore::SpeedEdge hold{0,10.2,0,10.2,0,0,.5};
  EXPECT_FALSE(core.speedAdmissible(hold,ego,limits,.02,core.ceilingRecovery(ego,limits)));
}
TEST(CorePathMeasuredRecovery, DoesNotIncreaseCurvatureDuringExistingLateralExcess) {
  PathGenerationCore core;const EgoState ego{{{0,0},0},0,10.2,0,.02};
  PlanningConstraints limits;limits.maximum_speed_mps=10;
  const auto recovery=core.ceilingRecovery(ego,limits);const auto& q=recovery.phase[0];
  const PathGenerationCore::SpeedEdge edge{0,10.2,0,q.speed(q.h),q.a+q.j*q.h,0,q.h};
  EXPECT_FALSE(core.speedAdmissible(edge,ego,limits,.021,recovery));
}
TEST(CorePathMeasuredRecovery, NominalInitialStateCannotCreateANewLateralExcess) {
  PathGenerationCore core;const EgoState ego{{{0,0},0},0,10.2,0,.019};
  PlanningConstraints limits;limits.maximum_speed_mps=10;
  const auto recovery=core.ceilingRecovery(ego,limits);const auto& q=recovery.phase[0];
  const PathGenerationCore::SpeedEdge edge{0,10.2,0,q.speed(q.h),q.a+q.j*q.h,0,q.h};
  EXPECT_FALSE(core.speedAdmissible(edge,ego,limits,.02,recovery));
}
TEST(CorePathMeasuredRecovery, CurvedRoadProducesANewFullyCheckedBrakingPlan) {
  PathGenerationCore core;
  std::vector<ReferencePoint> road;
  for(int i=-40;i<=300;++i){double a=i*.5/50;road.push_back({{50*std::sin(a),50*(1-std::cos(a))},-4,4});}
  const EgoState ego{{{0,0},0},0,10.2,0,.02};
  Predictions scene;PlanningConstraints limits;
  auto path=core.plan(ego,road,scene,limits);
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,10.2);
  EXPECT_DOUBLE_EQ(path.front().curvature_inv_m,.02);
  EXPECT_LE(path.back().speed_mps,10+1e-6);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
  double maximum=0;for(const auto& q:path)maximum=std::max(maximum,q.speed_mps*q.speed_mps*std::abs(q.curvature_inv_m));
  EXPECT_LE(maximum,2.0808+1e-5);
}
TEST(CorePathMeasuredRecovery, CannotDeferRecoveryPastPredictionHorizon) {
  PathParameters parameters;parameters.horizon_m=37.5;parameters.trajectory_horizon_m=150;
  PathGenerationCore core(parameters);std::vector<ReferencePoint> road;
  for(int i=-40;i<=300;++i){const double a=i*.5/50;road.push_back({{50*std::sin(a),50*(1-std::cos(a))},-4,4});}
  const EgoState ego{{{0,0},0},0,30,0,.02};
  // 기존 최대 jerk/감속으로 10m/s까지 복귀하는 데 5초보다 오래 걸린다.
  // 한도 초과가 남은 궤적을 정상적인 5초 계획으로 발행하면 안 된다.
  const auto path=core.plan(ego,road,Predictions{},PlanningConstraints{});
  EXPECT_TRUE(path.empty());
}

TEST(CorePathMeasuredRecovery, RejectsRecoveryPrefixBeforeReturningToLimit) {
  PathGenerationCore core;std::vector<ReferencePoint> road;
  for(int i=-40;i<=300;++i){double a=i*.5/50;road.push_back({{50*std::sin(a),50*(1-std::cos(a))},-4,4});}
  const EgoState ego{{{0,0},0},0,10.2,0,.02};Predictions scene;PlanningConstraints limits;
  auto path=core.plan(ego,road,scene,limits);ASSERT_GE(path.size(),3u);
  path.resize(2);ASSERT_GT(path.back().speed_mps,10.0);
  EXPECT_FALSE(core.checkPlannedPath(ego,road,scene,limits,path));
}

}
