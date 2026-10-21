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

struct ReentryPrefixFixture : ::testing::Test {
  PathGenerationCore core;
  std::vector<ReferencePoint> road{{{-20,0},-2,2},{{150,0},-2,2}};
  PathGenerationCore::Reference reference{road};
  Predictions predictions;
  std::vector<ProgressTime> unused;
  std::vector<Vec2> boundary;
  ReentryPrefixFixture() {
    boundary=reference.boundary(.25);
    auto region=std::make_shared<PathGenerationCore::NativeRoadRegion>();
    PathGenerationCore::RoadPolygon polygon;
    for(const auto& point:boundary)polygon.outer().emplace_back(point.x,point.y);
    boost::geometry::correct(polygon);region->polygons.push_back(polygon);region->prepare();
    core.active_native_region_=region;
  }
  bool edge(double yaw,double speed) {
    const EgoState ego{{{0,1.2},yaw},speed};
    auto geometry=core.roadGeometry(road,reference);
    auto region=core.roadReentryRegion(ego,geometry);
    EXPECT_TRUE(static_cast<bool>(region->recovery_original));
    const PathGenerationCore::Context context{reference,predictions,unused,boundary,0,0,false,nullptr,region.get()};
    std::vector<PathPoint> points(2);points[0].pose=ego.pose;points[0].speed_mps=speed;
    points[1]=points[0];points[1].pose.position.x=std::cos(yaw);points[1].pose.position.y+=std::sin(yaw);points[1].progress_m=1;points[1].time_sec=1;
    const PathGenerationCore::ArcPath arc(points);
    const PathGenerationCore::SpeedEdge edge{0,speed,0,speed,0,0,.5};
    return core.edgeClear(context,arc,edge);
  }
};
TEST_F(ReentryPrefixFixture, RejectsGrowingOutsideAreaWithoutWaitingForTheTerminalBeam) {
  // y=1.2m에서 +0.1rad로 전진하면 이미 걸친 차체의 바깥 면적이 증가한다.
  // 현재 객체가 없어도 기존 복귀 조건을 위반한 edge는 속도 beam에 넣지 않는다.
  EXPECT_FALSE(edge(.1,1));
}
TEST_F(ReentryPrefixFixture, KeepsAnInwardPrefixBeforeItHasFullyReturned) {
  // 최종 복귀 전이라도 바깥 면적이 감소하는 초기 edge는 계속 탐색한다.
  EXPECT_TRUE(edge(-.1,1));
}
TEST_F(ReentryPrefixFixture, KeepsStationaryWaitDuringPrefixValidation) {
  EXPECT_TRUE(edge(.1,0));
}
}
