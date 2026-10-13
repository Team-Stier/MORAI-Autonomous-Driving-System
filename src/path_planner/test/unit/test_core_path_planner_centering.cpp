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

namespace {
using namespace stier;
std::vector<ReferencePoint> circle(double radius,double turn,double angle_step=.025) {
  std::vector<ReferencePoint> points;
  for(int i=-20;i<=140;++i) {
    const double angle=i*angle_step;
    points.push_back({{radius*std::sin(angle),turn*radius*(1-std::cos(angle))},-1.75,1.75});
  }
  return points;
}
Predictions emptyScene() { Predictions predictions; predictions.reference_stamp_sec=10; return predictions; }
double inwardOffset(const Pose2& pose,double radius,double turn) {
  return radius-std::hypot(pose.position.x,pose.position.y-turn*radius);
}
void expectInsideCircle(const std::vector<PathPoint>& path,double radius,double turn,double half_road_width=1.75) {
  const VehicleShape vehicle;
  const double margin=PathParameters().road_margin_m;
  const double rear=-vehicle.rear_m-margin,front=vehicle.front_m+margin;
  const double half_width=vehicle.width_m/2+margin;
  // Independent analytic annulus oracle: maximum radius is at a corner;
  // minimum radius can lie INSIDE an edge, not at any corner. Check the
  // whole padded rectangle at 17 poses on every output interpolation edge.
  for(std::size_t i=1;i<path.size();++i) {
    const Pose2 first=path[i-1].pose,last=path[i].pose;
    const double angle=std::remainder(last.yaw_rad-first.yaw_rad,2*std::acos(-1.0));
    for(int k=0;k<=16;++k) {
      const double u=k/16.0;
      const Pose2 pose{{first.position.x+u*(last.position.x-first.position.x),
                        first.position.y+u*(last.position.y-first.position.y)},first.yaw_rad+u*angle};
      const double cs=std::cos(pose.yaw_rad),sn=std::sin(pose.yaw_rad);
      const double dx=-pose.position.x,dy=turn*radius-pose.position.y;
      const double center_x=cs*dx+sn*dy,center_y=-sn*dx+cs*dy;
      const double closest_x=std::max(rear,std::min(front,center_x));
      const double closest_y=std::max(-half_width,std::min(half_width,center_y));
      EXPECT_GT(std::hypot(center_x-closest_x,center_y-closest_y),radius-half_road_width);
      for(double x:{rear,front}) for(double y:{-half_width,half_width}) {
        const double world_x=pose.position.x+x*cs-y*sn;
        const double world_y=pose.position.y+x*sn+y*cs;
        const double radial=std::hypot(world_x,world_y-turn*radius);
        EXPECT_LT(radial,radius+half_road_width);
      }
    }
  }
}
}

TEST(CorePathCentering, ClearCurveDoesNotInheritSmallPreviousInwardBias) {
  for(double turn:{-1.0,1.0}) {
    SCOPED_TRACE(turn);
    const auto reference=circle(20,turn);
    EgoState ego{{{0,0},0},10}; ego.curvature_inv_m=turn/20;
    std::vector<PathPoint> previous;
    for(int i=0;i<=100;++i) {
      const double s=i*.25,a=s/20;
      previous.push_back({{{19.88*std::sin(a),turn*(20-19.88*std::cos(a))},turn*a},s,s/3,turn/19.88});
    }
    const PathGenerationCore core;
    const auto path=core.process(ego,reference,emptyScene(),{{0,0},{25,25.0/3}},previous);
    ASSERT_FALSE(path.empty());
    double maximum_inward=0;
    for(const auto& point:path)
      maximum_inward=std::max(maximum_inward,inwardOffset(point.pose,20,turn));
    EXPECT_LT(maximum_inward,.003);
    EXPECT_DOUBLE_EQ(path.front().pose.position.x,0);
    EXPECT_DOUBLE_EQ(path.front().pose.position.y,0);
    EXPECT_TRUE(core.checkTimedPath(ego,reference,emptyScene(),path));
    expectInsideCircle(path,20,turn);
  }
}

TEST(CorePathCentering, TightCurveUsesNecessaryFootprintCompensationNotHalfMeterGridOffset) {
  for(double turn:{-1.0,1.0}) {
    SCOPED_TRACE(turn);
    const auto reference=circle(8,turn);
    EgoState ego{{{0,turn*.23},0},10}; ego.curvature_inv_m=turn/7.77;
    const PathGenerationCore core;
    const auto path=core.process(ego,reference,emptyScene(),{{0,0},{25,12.5}});
    ASSERT_FALSE(path.empty());
    EXPECT_DOUBLE_EQ(path.front().pose.position.y,turn*.23);
    double maximum_inward=0;
    for(const auto& point:path)
      maximum_inward=std::max(maximum_inward,inwardOffset(point.pose,8,turn));
    EXPECT_LT(maximum_inward,.30);
    EXPECT_TRUE(core.checkTimedPath(ego,reference,emptyScene(),path));
    expectInsideCircle(path,8,turn);
  }
}

TEST(CorePathCentering, SparseCircularReferenceDoesNotShortcutTheMapCurve) {
  for(double turn:{-1.0,1.0}) {
    std::vector<ReferencePoint> points;
    for(int i=-3;i<=5;++i) {
      const double a=i*.5;
      points.push_back({{20*std::sin(a),turn*20*(1-std::cos(a))},-3.5,3.5});
    }
    const PathGenerationCore::Reference reference(points);
    const double chord=40*std::sin(.25);
    const auto sample=reference.at(3.5*chord);
    const double error=20-std::hypot(sample.position.x,sample.position.y-turn*20);
    EXPECT_LT(std::abs(error),.005);
  }
}

TEST(CorePathCentering, StraightOutputKeepsRearAxleContractWithoutLongitudinalShift) {
  std::vector<ReferencePoint> reference;
  for(int x=-10;x<=70;++x) reference.push_back({{static_cast<double>(x),0},-1.75,1.75});
  const EgoState ego{{{0,0},0},10};
  const auto path=PathGenerationCore().process(ego,reference,emptyScene(),{{0,0},{25,12.5}});
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.front().pose.position.x,0);
  EXPECT_DOUBLE_EQ(path.front().pose.position.y,0);
  EXPECT_NEAR(path.back().pose.position.x,10,1e-7);
  for(const auto& point:path) EXPECT_NEAR(point.pose.position.y,0,1e-9);
}

TEST(CorePathCentering, CurvedEndpointHeadingDoesNotLagByHalfASample) {
  for(double turn:{-1.0,1.0}) {
    const auto reference=circle(20,turn);
    const EgoState ego{{{0,0},0},10};
    const auto path=PathGenerationCore().process(ego,reference,emptyScene(),{{0,0},{25,25.0/3}});
    ASSERT_FALSE(path.empty());
    const auto& pose=path.back().pose;
    const double tangent=turn*std::atan2(pose.position.x,20-turn*pose.position.y);
    EXPECT_LT(std::abs(geometry::wrap(pose.yaw_rad-tangent)),.001);
  }
}

TEST(CorePathCentering, SmallLocalizationOffsetAndHeadingRecoverToCenterWithoutOldBias) {
  for(double turn:{-1.0,1.0}) {
    const auto reference=circle(20,turn);
    EgoState ego{{{0,turn*.002},turn*.001},10}; ego.curvature_inv_m=turn/20;
    std::vector<PathPoint> previous;
    for(int i=0;i<=100;++i) {
      const double s=i*.25,a=s/20;
      previous.push_back({{{19.88*std::sin(a),turn*(20-19.88*std::cos(a))},turn*a},s,s/3,turn/19.88});
    }
    const PathGenerationCore core;
    const auto path=core.process(ego,reference,emptyScene(),{{0,0},{25,25.0/3}},previous);
    ASSERT_FALSE(path.empty());
    EXPECT_DOUBLE_EQ(path.front().pose.position.y,ego.pose.position.y);
    EXPECT_DOUBLE_EQ(path.front().pose.yaw_rad,ego.pose.yaw_rad);
    double maximum_error=0;
    for(const auto& point:path) {
      if(point.progress_m>=5) maximum_error=std::max(maximum_error,std::abs(inwardOffset(point.pose,20,turn)));
    }
    EXPECT_LT(maximum_error,.003);
    EXPECT_TRUE(core.checkTimedPath(ego,reference,emptyScene(),path));
    expectInsideCircle(path,20,turn);
  }
}

TEST(CorePathCentering, SmoothingDoesNotMoveExistingAllowedRoadPolygon) {
  std::vector<ReferencePoint> points;
  for(int i=-3;i<=5;++i) {
    const double a=i*.5;
    points.push_back({{20*std::sin(a),20*(1-std::cos(a))},-1.75,1.75});
  }
  const PathGenerationCore::Reference reference(points);
  const auto polygon=reference.boundary(6); // Two subdivisions per 9.9-m chord.
  // The prior Hermite midpoint of [0, .5 rad], with averaged unit chord
  // derivatives, is radially inward by this independently derived amount.
  const double midpoint_radius=20*std::cos(.25)*(1+.5*std::sin(.25)*std::sin(.25));
  for(double side:{-1.75,1.75}) {
    const double radial=midpoint_radius-side;
    const Vec2 expected{radial*std::sin(.25),20-radial*std::cos(.25)};
    double nearest=std::numeric_limits<double>::infinity();
    for(const auto& point:polygon) nearest=std::min(nearest,std::hypot(point.x-expected.x,point.y-expected.y));
    EXPECT_LT(nearest,1e-8);
  }
}

TEST(CorePathCentering, InfeasibleInitialFootprintIsNotTeleportedToCompensatedPath) {
  for(double turn:{-1.0,1.0}) {
    const auto reference=circle(8,turn);
    EgoState ego{{{0,0},0},10}; ego.speed_mps=2; ego.curvature_inv_m=turn/8;
    const PathGenerationCore core;
    EXPECT_TRUE(core.process(ego,reference,emptyScene(),{{0,0},{25,12.5}}).empty());
    PlanningConstraints limits; limits.maximum_speed_mps=2;
    EXPECT_TRUE(core.process(ego,reference,emptyScene(),limits).empty());
    EXPECT_DOUBLE_EQ(ego.pose.position.x,0);
    EXPECT_DOUBLE_EQ(ego.pose.position.y,0);
  }
}

TEST(CorePathCentering, IntegratedSpeedPlanningPreservesRearAxleInitialStateOnCurve) {
  for(double turn:{-1.0,1.0}) {
    const auto reference=circle(20,turn);
    EgoState ego{{{0,0},0},10}; ego.speed_mps=3; ego.curvature_inv_m=turn/20;
    PlanningConstraints limits; limits.maximum_speed_mps=3;
    const PathGenerationCore core;
    const auto path=core.process(ego,reference,emptyScene(),limits);
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.back().time_sec,5,1e-10);
    EXPECT_DOUBLE_EQ(path.front().pose.position.x,0);
    EXPECT_DOUBLE_EQ(path.front().pose.position.y,0);
    EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
    EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
    EXPECT_DOUBLE_EQ(path.front().curvature_inv_m,ego.curvature_inv_m);
    EXPECT_TRUE(core.checkPlannedPath(ego,reference,emptyScene(),limits,path));
    expectInsideCircle(path,20,turn);
  }
}

TEST(CorePathCentering, BlockedCurvedCenterlineStillAllowsRealLateralAvoidance) {
  for(double turn:{-1.0,1.0}) {
    auto reference=circle(20,turn);
    for(auto& point:reference) { point.min_l_m=-3.5; point.max_l_m=3.5; }
    EgoState ego{{{0,0},0},10}; ego.curvature_inv_m=turn/20;
    auto scene=emptyScene();
    PredictedObject obstacle;
    obstacle.id=1; obstacle.kind=ObjectKind::Static;
    obstacle.pose={{20*std::sin(.75),turn*20*(1-std::cos(.75))},turn*.75};
    obstacle.length_m=2; obstacle.width_m=.5; obstacle.source_stamp_sec=10;
    scene.current.push_back(obstacle);
    for(auto& slice:scene.future) slice.push_back(obstacle);
    const PathGenerationCore core;
    const auto path=core.process(ego,reference,scene,{{0,0},{25,25.0/3}});
    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(core.checkTimedPath(ego,reference,scene,path));
    expectInsideCircle(path,20,turn,3.5);
    double deviation=0;
    const VehicleShape vehicle;
    for(std::size_t i=1;i<path.size();++i) {
      const auto first=path[i-1].pose,last=path[i].pose;
      const double angle=std::remainder(last.yaw_rad-first.yaw_rad,2*std::acos(-1.0));
      for(int k=0;k<=16;++k) {
        const double u=k/16.0;
        const Pose2 own{{first.position.x+u*(last.position.x-first.position.x),
                         first.position.y+u*(last.position.y-first.position.y)},first.yaw_rad+u*angle};
        deviation=std::max(deviation,std::abs(inwardOffset(own,20,turn)));
        const double cs=std::cos(own.yaw_rad),sn=std::sin(own.yaw_rad);
        const Vec2 center{own.position.x+(vehicle.front_m-vehicle.rear_m)/2*cs,
                          own.position.y+(vehicle.front_m-vehicle.rear_m)/2*sn};
        bool separated=false;
        // Independent rectangle SAT, including the configured obstacle margin.
        for(double axis_angle:{own.yaw_rad,own.yaw_rad+std::acos(-1.0)/2,
                               obstacle.pose.yaw_rad,obstacle.pose.yaw_rad+std::acos(-1.0)/2}) {
          const double dx=obstacle.pose.position.x-center.x,dy=obstacle.pose.position.y-center.y;
          const double own_extent=(vehicle.front_m+vehicle.rear_m)/2*std::abs(std::cos(axis_angle-own.yaw_rad))+
                                  vehicle.width_m/2*std::abs(std::sin(axis_angle-own.yaw_rad));
          const double other_extent=(obstacle.length_m/2+.25)*std::abs(std::cos(axis_angle-obstacle.pose.yaw_rad))+
                                    (obstacle.width_m/2+.25)*std::abs(std::sin(axis_angle-obstacle.pose.yaw_rad));
          if(std::abs(dx*std::cos(axis_angle)+dy*std::sin(axis_angle))>own_extent+other_extent) separated=true;
        }
        ASSERT_TRUE(separated);
      }
    }
    EXPECT_GT(deviation,.5); // Centering preference must not override avoidance.
  }
}

TEST(CorePathCentering, DistantRoadEndCapCannotRejectAnUpstreamWindingSection) {
  for(double turn:{-1.0,1.0}) {
    auto reference=circle(20,turn);
    for(int i=141;i<=200;++i) {
      const double angle=i*.025;
      reference.push_back({{20*std::sin(angle),turn*20*(1-std::cos(angle))},-1.75,1.75});
    }
    EgoState ego{{{0,0},0},10}; ego.curvature_inv_m=turn/20;
    const PathGenerationCore core;
    const auto path=core.process(ego,reference,emptyScene(),{{0,0},{25,25.0/3}});
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.back().progress_m,15,1e-8);
    EXPECT_TRUE(core.checkTimedPath(ego,reference,emptyScene(),path));
    expectInsideCircle(path,20,turn);
  }
}

TEST(CorePathCentering, NominalRecoveryRespectsConfiguredLateralSlopeBound) {
  std::vector<ReferencePoint> reference;
  for(int x=-10;x<=70;++x) reference.push_back({{static_cast<double>(x),0},-3.5,3.5});
  const EgoState ego{{{0,1},0},10};
  PathParameters parameters;
  parameters.dp_step_m=10; parameters.max_lateral_slope=.1; parameters.dp_slope_sample=.1;
  const PathGenerationCore core(parameters);
  const auto path=core.process(ego,reference,emptyScene(),{{0,0},{25,5}});
  ASSERT_FALSE(path.empty());
  double maximum_slope=0;
  for(std::size_t i=1;i<path.size();++i)
    maximum_slope=std::max(maximum_slope,std::abs(path[i].pose.position.y-path[i-1].pose.position.y)/
                                       (path[i].progress_m-path[i-1].progress_m));
  EXPECT_LE(maximum_slope,parameters.max_lateral_slope+2*parameters.qp_tolerance);
  EXPECT_TRUE(core.checkTimedPath(ego,reference,emptyScene(),path));
}

TEST(CorePathCentering, CloserWideObstacleOnCurveStillAllowsReachableBraking) {
  for(double turn:{-1.0,1.0}) {
    const auto reference=circle(20,turn);
    EgoState ego{{{0,0},0},10}; ego.speed_mps=3; ego.curvature_inv_m=turn/20;
    auto scene=emptyScene();
    PredictedObject obstacle;
    obstacle.id=1; obstacle.kind=ObjectKind::Static;
    obstacle.pose={{20*std::sin(.6),turn*20*(1-std::cos(.6))},turn*.6};
    obstacle.length_m=2; obstacle.width_m=1.2; obstacle.source_stamp_sec=10;
    scene.current.push_back(obstacle);
    for(auto& slice:scene.future) slice.push_back(obstacle);
    PlanningConstraints limits; limits.maximum_speed_mps=3;
    const PathGenerationCore core;
    const auto path=core.process(ego,reference,scene,limits);
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.back().time_sec,5,1e-10);
    EXPECT_NEAR(path.back().speed_mps,0,1e-10);
    EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,path));
    expectInsideCircle(path,20,turn);
  }
}
