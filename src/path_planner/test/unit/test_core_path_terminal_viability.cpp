// 먼 정적 장애물 접근과 가까운 정지 동작의 회귀 검사.
#include <chrono>
#include <iostream>
#include <string>
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
#include <path_planner/parameters/core_path_planner.hpp>
namespace {
using Core=path_planner::CorePathPlanner;
void verifyTerminal(double distance,bool far,bool stop_requested=false) {
 const std::string pkg=NP_PACKAGE_DIR;
 auto p=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
 Core core(p);
 std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{200,0},-1.75,1.75}};
 const Core::Reference reference(road);auto geometry=core.roadGeometry(road,reference);
 stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=far?13.711:9;ego.acceleration_mps2=far?.871915:0;
 stier::Predictions scene;scene.reference_stamp_sec=10;
 stier::PredictedObject wall;wall.id=1;wall.kind=stier::ObjectKind::Static;wall.pose={{distance,0},0};wall.length_m=4;wall.width_m=2;wall.source_stamp_sec=10;
 scene.current.push_back(wall);for(auto& future:scene.future)future.push_back(wall);
 const std::vector<stier::ProgressTime> unused;
 Core::Context context{reference,scene,unused,geometry->boundary,0,0,false,&geometry->index};
 std::vector<stier::PathPoint> shape;for(int i=0;i<=400;++i)shape.push_back({{{i*.25,0},0},i*.25,i*.25,0});
 const Core::ArcPath arc(shape);stier::PlanningConstraints limits;limits.maximum_speed_mps=stop_requested?0:60./3.6;
 bool truncated=false;const auto path=core.planSpeed(context,arc,ego,limits,5,INFINITY,road,p.speed_beam_width,truncated);
 ASSERT_GE(path.size(),2u);EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
 auto two=std::min_element(path.begin(),path.end(),[](const stier::PathPoint& a,const stier::PathPoint& b){return std::abs(a.time_sec-2)<std::abs(b.time_sec-2);});
 if(far&&!stop_requested) {EXPECT_GE(two->speed_mps,13.5);EXPECT_GT(path.back().speed_mps,0);}
 else EXPECT_DOUBLE_EQ(path.back().speed_mps,0);
 for(const auto& point:path)EXPECT_LT(point.pose.position.x+p.vehicle.front_m,distance-2-p.obstacle_margin_m);
}
TEST(CorePathTerminalStop, DistantWallKeepsTheViableFiveSecondApproach){verifyTerminal(100,true);}
TEST(CorePathTerminalStop, CloseWallStillProducesCompleteStop){verifyTerminal(29,false);}
TEST(CorePathTerminalStop, ExplicitZeroSpeedRequestStillStops){verifyTerminal(100,false,true);}
}
