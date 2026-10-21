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
using Core=path_planner::CorePathPlanner;
using Geometry=Core::Geometry;
void verifyForwardPlan(EgoState ego,int section) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  const auto map_reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_reader));
  const auto checkpoints=path_planner::readCheckpoints(map_reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(shared);
  const auto params=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
  Core core(params,shared);
  nav_msgs::Odometry odom;odom.pose.pose.position.x=ego.pose.position.x;odom.pose.pose.position.y=ego.pose.position.y;
  odom.pose.pose.orientation.z=std::sin(ego.pose.yaw_rad/2);odom.pose.pose.orientation.w=std::cos(ego.pose.yaw_rad/2);
  const path_planner::Checkpoints remain(checkpoints.begin()+section,checkpoints.end());
  const auto route=std::get<5>(global.process(odom,{},{},{},{},remain));ASSERT_TRUE(bool(route));
  const auto body=core.rearFootprintRoute(route);const auto region=core.nativeRoadRegion(body);
  Core::NativeRegionScope scope(core.active_native_region_,region);
  const auto reference=core.nativeReference(body)->points;
  ASSERT_TRUE(region->containsBox(Geometry::sweptEgoEnvelope(ego.pose,ego.pose,params.vehicle,params.road_margin_m)));
  Predictions scene;scene.reference_stamp_sec=10;
  PlanningConstraints limits;limits.maximum_speed_mps=params.normal_speed_limit_mps;
  const auto path=core.plan(ego,reference,scene,limits);
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
  EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
  EXPECT_DOUBLE_EQ(path.front().curvature_inv_m,ego.curvature_inv_m);
  EXPECT_GT(path.back().progress_m,1);
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,path));
  for(const auto& q:path) EXPECT_TRUE(region->containsBox(Geometry::egoBox(q.pose,params.vehicle)));
  PredictedObject obstacle;obstacle.id=1;obstacle.pose=ego.pose;obstacle.length_m=10;obstacle.width_m=10;
  scene.current.push_back(obstacle);for(auto& slice:scene.future)slice.push_back(obstacle);
  EXPECT_TRUE(core.plan(ego,reference,scene,limits).empty());
}
TEST(CorePathC2InitialSlope, ReplansTheMeasuredStraighteningTransitionWithoutChangingInitialCurvature) {
  EgoState ego;ego.pose={{62.173702923785015,258.30437548323022},-1.5260649523237027};
  ego.speed_mps=7.4123706817626953;ego.acceleration_mps2=.1103830228442271;
  ego.curvature_inv_m=.0012680941046749319;ego.stamp_sec=10.053358;
  verifyForwardPlan(ego,11);
}
}
