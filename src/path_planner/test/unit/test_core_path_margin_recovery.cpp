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
using Core=path_planner::CorePathPlanner;
using namespace stier;
using Geometry=Core::Geometry;
struct MarginRecoveryFixture : ::testing::Test {
  Core core;
  std::vector<ReferencePoint> road{{{-20,0},-2,2},{{150,0},-2,2}};
  Core::Reference reference{road};
  Predictions scene;
  std::vector<ProgressTime> schedule;
  std::vector<Vec2> boundary;
  std::shared_ptr<Core::NativeRoadRegion> original;
  MarginRecoveryFixture() {
    boundary=reference.boundary(.25);original=std::make_shared<Core::NativeRoadRegion>();
    Core::RoadPolygon polygon;
    for(const auto& q:boundary)polygon.outer().emplace_back(q.x,q.y);
    boost::geometry::correct(polygon);original->polygons.push_back(polygon);original->prepare();
    core.active_native_region_=original;
  }
  bool check(std::vector<PathPoint> points,bool require_return) {
    EgoState initial;initial.pose=points.front().pose;
    auto region=core.roadReentryRegion(initial,core.roadGeometry(road,reference));
    EXPECT_TRUE(static_cast<bool>(region->recovery_original));
    const Core::Context c{reference,scene,schedule,boundary,0,0,false,nullptr,region.get()};
    return core.reentryProgressClear(c,points,require_return);
  }
  std::vector<PathPoint> inwardPath() {
    std::vector<PathPoint> out(3);
    out[0].pose={{0,.88},.005};
    out[1].pose={{.5,.8825},.005};out[1].progress_m=.5;
    out[2].pose={{5,0},0};out[2].progress_m=5;
    return out;
  }
};
TEST_F(MarginRecoveryFixture, AllowsSmallMarginConsumptionWhilePhysicalBodyStaysInside) {
  const auto path=inwardPath();
  for(const auto& q:path)ASSERT_TRUE(original->containsBox(Geometry::egoBox(q.pose,PathParameters{}.vehicle)));
  ASSERT_FALSE(original->containsBox(Geometry::sweptEgoEnvelope(path.front().pose,path.front().pose,PathParameters{}.vehicle,.05)));
  EXPECT_TRUE(check(path,true));
}
TEST_F(MarginRecoveryFixture, RejectsAnyNewPhysicalDeparture) {
  auto path=inwardPath();path[1].pose.position.y=1.02;
  ASSERT_FALSE(original->containsBox(Geometry::egoBox(path[1].pose,PathParameters{}.vehicle)));
  EXPECT_FALSE(check(path,true));
}
TEST_F(MarginRecoveryFixture, StillRequiresRestoringTheFullConfiguredMargin) {
  auto path=inwardPath();path.resize(2);
  EXPECT_FALSE(check(path,true));
  EXPECT_TRUE(check(path,false));
}
TEST_F(MarginRecoveryFixture, ChecksContinuousPhysicalSweepBetweenContainedPoses) {
  auto path=inwardPath();path.resize(2);path[0].pose.yaw_rad=0;
  path[1].pose={{1,.88},Geometry::kPi};path[1].progress_m=1;
  for(const auto& q:path)ASSERT_TRUE(original->containsBox(Geometry::egoBox(q.pose,PathParameters{}.vehicle)));
  EXPECT_FALSE(check(path,false));
}
TEST(CorePathMarginRecovery, ReplansTheRecordedRoundaboutEntryWithoutObjects) {
  ros::Time::init();const std::string pkg=NP_PACKAGE_DIR;
  auto reader=node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto loaded=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(shared);
  auto params=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
  Core core(params,shared);
  EgoState ego;ego.pose={{-98.076996016631185,320.15057299073487},1.1720896739372502};
  ego.stamp_sec=10.080;
  ego.speed_mps=2.1717860698699951;ego.acceleration_mps2=-2.7090376742363103;
  ego.curvature_inv_m=-.027553317791784968;
  nav_msgs::Odometry odom;odom.pose.pose.position.x=ego.pose.position.x;odom.pose.pose.position.y=ego.pose.position.y;
  odom.pose.pose.orientation.z=std::sin(ego.pose.yaw_rad/2);odom.pose.pose.orientation.w=std::cos(ego.pose.yaw_rad/2);
  const path_planner::Checkpoints remaining(loaded.begin()+9,loaded.end());
  const auto route=std::get<5>(global.process(odom,{},{},{},{},remaining));ASSERT_TRUE(bool(route));
  const auto body=core.rearFootprintRoute(route);const auto original=core.nativeRoadRegion(body);
  Core::NativeRegionScope scope(core.active_native_region_,original);
  const auto reference=core.nativeReference(body)->points;
  ASSERT_TRUE(original->containsBox(Geometry::egoBox(ego.pose,params.vehicle)));
  ASSERT_FALSE(original->containsBox(Geometry::sweptEgoEnvelope(ego.pose,ego.pose,params.vehicle,params.road_margin_m)));
  Predictions scene;scene.reference_stamp_sec=10;PlanningConstraints limits;
  limits.maximum_speed_mps=params.normal_speed_limit_mps;
  const auto path=core.plan(ego,reference,scene,limits);
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
  EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
  EXPECT_DOUBLE_EQ(path.front().curvature_inv_m,ego.curvature_inv_m);
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,path));
  for(const auto& q:path)EXPECT_TRUE(original->containsBox(Geometry::egoBox(q.pose,params.vehicle)));
  EXPECT_TRUE(original->containsBox(Geometry::sweptEgoEnvelope(path.back().pose,path.back().pose,params.vehicle,params.road_margin_m)));
  PredictedObject obstacle;obstacle.id=1;obstacle.pose=ego.pose;obstacle.length_m=10;obstacle.width_m=10;
  scene.current.push_back(obstacle);for(auto& slice:scene.future)slice.push_back(obstacle);
  EXPECT_TRUE(core.plan(ego,reference,scene,limits).empty());
}
}
