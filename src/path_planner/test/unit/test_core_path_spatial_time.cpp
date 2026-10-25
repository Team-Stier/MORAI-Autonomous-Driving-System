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
#include <fstream>

namespace {
std::vector<stier::PathPoint> recordedSpeedCandidate() {
  std::ifstream input(std::string(NP_PACKAGE_DIR)+"/test/data/core_path_spatial_time_r189.csv");
  std::string line;std::getline(input,line);std::vector<stier::PathPoint> path;
  while(std::getline(input,line)) {
    std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);stier::PathPoint q;
    row>>q.pose.position.x>>q.pose.position.y>>q.pose.yaw_rad>>q.curvature_inv_m>>q.time_sec>>q.speed_mps>>q.acceleration_mps2;
    if(!row){ADD_FAILURE()<<"Invalid native candidate fixture";return {};}
    q.progress_m=path.empty()?0:path.back().progress_m+std::hypot(q.pose.position.x-path.back().pose.position.x,q.pose.position.y-path.back().pose.position.y);
    path.push_back(q);
  }
  return path;
}
TEST(CorePathSpatialTime, ChecksVelocityWithinTheSameSpatialCurvatureInterval) {
  // R189의 최종 후보는 32.5~34m의 균일 공간 창에서만 거부됐다.
  // 창 밖의 이전 속도를 제외하면 해당 창의 연속 속도 다항식 상한으로
  // 계산한 횡가속도는 2.994m/s² 이하이며 원래 3.0 한도를 만족한다.
  const auto path=recordedSpeedCandidate();ASSERT_EQ(path.size(),401u);
  stier::PathParameters parameters;parameters.max_lateral_accel_mps2=3.0;
  EXPECT_TRUE(path_planner::CorePathPlanner{parameters}.sampledGeometryClear(path,true));
}
TEST(CorePathSpatialTime, StillRejectsActualLateralExcessWithinTheSpatialInterval) {
  auto path=recordedSpeedCandidate();ASSERT_EQ(path.size(),401u);
  for(auto& q:path){q.time_sec/=1.02;q.speed_mps*=1.02;q.acceleration_mps2*=1.02*1.02;}
  stier::PathParameters parameters;parameters.max_lateral_accel_mps2=3.0;
  EXPECT_FALSE(path_planner::CorePathPlanner{parameters}.sampledGeometryClear(path,true));
}
}  // namespace
