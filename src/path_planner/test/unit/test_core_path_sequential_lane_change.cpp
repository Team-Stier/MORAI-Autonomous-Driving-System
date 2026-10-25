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
struct SequentialScene {
  lanelet::Lanelets lanes;
  path_planner::SharedMap shared;
  std::unique_ptr<path_planner::CorePathPlanner> core;
  bool curve;
  explicit SequentialScene(bool curved=false) : shared({makeMap(curved)}),curve(curved) {
    ros::Time::init();
    for(int i=0;i<3;++i)lanes.push_back(shared.map->laneletLayer.get(300+i));
    path_planner::PathParameters p;
    p.max_lateral_accel_mps2=3;p.preferred_lateral_accel_mps2=2.7;
    core.reset(new path_planner::CorePathPlanner(p,shared));
  }
  static lanelet::LaneletMapPtr makeMap(bool curved) {
    std::vector<lanelet::LineString3d> boundaries;
    for(int j=0;j<4;++j) {
      lanelet::Points3d points;
      if(curved) {
        for(int i=0;i<=250;++i) {
          const double a=-.5+i*.01,r=31.75-3.5*j;
          points.emplace_back(1000+j*300+i,r*std::sin(a),30-r*std::cos(a),0);
        }
      } else {
        points.emplace_back(100+2*j,0,-1.75+3.5*j,0);
        points.emplace_back(101+2*j,300,-1.75+3.5*j,0);
      }
      lanelet::LineString3d line(200+j,points);
      line.attributes()["type"]="line_thin";line.attributes()["subtype"]="dashed";
      boundaries.push_back(line);
    }
    lanelet::Lanelets lanes;
    for(int i=0;i<3;++i) {
      lanelet::Lanelet lane(300+i,boundaries[i+1],boundaries[i]);
      lane.attributes()["subtype"]="road";lane.attributes()["location"]="urban";lane.attributes()["one_way"]="yes";lanes.push_back(lane);
    }
    return lanelet::utils::createMap(lanes);
  }
  interfaces::Path call(double sec,double x,double y,double speed=10,double heading=0,
                        double steering=0,bool advanced=false,double rate=0,const interfaces::Objects* scene_objects=nullptr) {
    nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp.fromSec(sec);ego.child_frame_id="base_link";
    ego.pose.pose.position.x=x;ego.pose.pose.position.y=y;
    ego.pose.pose.orientation.z=std::sin(heading/2);ego.pose.pose.orientation.w=std::cos(heading/2);
    ego.twist.twist.linear.x=speed;ego.twist.twist.angular.z=rate;
    interfaces::CompetitionVehicleStatus status;status.header=ego.header;
    status.wheelbase_m=3.0;status.velocity_x_mps=status.signed_velocity_mps=speed;status.steering_angle_rad=steering;
    interfaces::Objects objects;if(scene_objects)objects=*scene_objects;objects.header=ego.header;
    path_planner::PredictedObjects predictions;predictions.fill(objects);
    for(std::size_t i=0;i<predictions.size();++i)for(auto& object:predictions[i].objects) {
      object.position.x+=object.velocity.x*.5*(i+1);
      object.position.y+=object.velocity.y*.5*(i+1);
      object.bbox_pose.position=object.position;
    }
    sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.z=9.81;
    lanelet::ConstLanelets route;
    for(size_t i=advanced?1:0;i<lanes.size();++i)route.push_back(lanes[i]);
    return core->process(ego,status,objects,{},imu,lanelet::routing::LaneletPath(route),predictions,ego.header.stamp);
  }
};
void expectFirstTarget(const interfaces::Path& path) {
  ASSERT_GE(path.points.size(),2u);
  for(const auto& p:path.points)EXPECT_LT(p.pose.pose.position.y,4.1);
  EXPECT_NEAR(path.points.back().pose.pose.position.y,3.5,.3);
}
// R173에서 차선1로 당겨 이동한 뒤, 아직 남은 종방향 경유 차선 때문에
// 차선2로 돌아왔다가 다시 차선1로 이동했다. GlobalPath가 제공한 변경 edge의
// source 이전으로 LocalPath가 독자적으로 다음 변경을 당기지 않아야 한다.
TEST(CorePathSequentialLaneChange, PreservesLongitudinalMissionBeforeTheNextLateralEdge) {
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/shared_map.yaml",package)));
  path_planner::CorePathPlanner core(path_planner::PathParameters{},map);
  lanelet::ConstLanelets lanes;
  for(auto id:{97788,96941,96955,97499,97500,114202,97079,97096,112288,97452})
    lanes.push_back(map.map->laneletLayer.get(id));
  const path_planner::Route route=lanelet::routing::LaneletPath(lanes);
  path_planner::EgoState ego;ego.pose.position={70.07094791,178.6678906};
  ego.pose.yaw_rad=-1.55;ego.speed_mps=10.55;
  core.laneChangeRoute(route,ego,0,100);
  EXPECT_FALSE(static_cast<bool>(core.lane_change_));
}
TEST(CorePathSequentialLaneChange, StartsTheFollowingChangeOnItsDeclaredSource) {
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  path_planner::SharedMap map(path_planner::loadSharedMapParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/shared_map.yaml",package)));
  path_planner::CorePathPlanner core(path_planner::PathParameters{},map);
  lanelet::ConstLanelets lanes;
  for(auto id:{96955,97499,97500,114202,97079,97096,112288,97452})
    lanes.push_back(map.map->laneletLayer.get(id));
  const path_planner::Route route=lanelet::routing::LaneletPath(lanes);
  path_planner::EgoState ego;ego.pose.position={70.49838162,65.33213951};
  ego.pose.yaw_rad=-1.55;ego.speed_mps=16.17;
  core.laneChangeRoute(route,ego,0,100);
  ASSERT_TRUE(static_cast<bool>(core.lane_change_));
  EXPECT_EQ(core.lane_change_->target.front().id(),97500);
}
TEST(CorePathSequentialLaneChange, ChoosesOnlyOneAdjacentLaneFromAChainedRoute) {
  SequentialScene s;
  const auto path=s.call(100,10,0);
  expectFirstTarget(path);
  EXPECT_GT(path.points.back().pose.pose.position.x,50);
}
TEST(CorePathSequentialLaneChange, RightwardChangesAlsoStopAtTheImmediateNeighbor) {
  SequentialScene s;std::reverse(s.lanes.begin(),s.lanes.end());
  const auto path=s.call(100,10,7);
  ASSERT_FALSE(path.points.empty());
  for(const auto& p:path.points)EXPECT_GT(p.pose.pose.position.y,2.9);
  EXPECT_NEAR(path.points.back().pose.pose.position.y,3.5,.3);
}
TEST(CorePathSequentialLaneChange, CopyKeepsIndependentConfirmationHistory) {
  SequentialScene original;original.call(100,10,0);original.call(101,60,3.5,10,0,0,true);
  SequentialScene copy;*copy.core=*original.core;
  for(int i=1;i<=10;++i)copy.call(101+.1*i,60+i,3.5,10,0,0,true);
  interfaces::Path path;
  for(int i=1;i<=10;++i)path=original.call(101+.1*i,60+i,3.5,10,0,0,true);
  ASSERT_FALSE(path.points.empty());EXPECT_GT(path.points.back().pose.pose.position.y,6.7);
}
TEST(CorePathSequentialLaneChange, FastRearTrafficDoesNotCauseADoubleLaneAvoidance) {
  SequentialScene s;interfaces::Objects objects;interfaces::Object rear;
  rear.id=1;rear.label="vehicle";rear.position.x=-5;rear.position.y=3.5;
  rear.velocity.x=30;rear.velocity_available=true;
  rear.bbox_size.x=4.5;rear.bbox_size.y=1.8;rear.bbox_size.z=1.5;rear.bbox_pose.orientation.w=1;
  objects.objects.push_back(rear);
  const auto path=s.call(100,10,0,10,0,0,false,0,&objects);
  ASSERT_FALSE(path.points.empty());
  for(const auto& p:path.points) {
    EXPECT_LT(p.pose.pose.position.y,4.1);
    const double yaw=2*std::atan2(p.pose.pose.orientation.z,p.pose.pose.orientation.w);
    const double cx=p.pose.pose.position.x+(3.845008-.790)/2*std::cos(yaw);
    const double cy=p.pose.pose.position.y+(3.845008-.790)/2*std::sin(yaw);
    const double hx=(3.845008+.790)/2*std::abs(std::cos(yaw))+2.160835/2*std::abs(std::sin(yaw));
    const double hy=(3.845008+.790)/2*std::abs(std::sin(yaw))+2.160835/2*std::abs(std::cos(yaw));
    EXPECT_TRUE(std::abs(cx-(-5+30*p.time_from_start.toSec()))>hx+2.25 || std::abs(cy-3.5)>hy+.9);
  }
}
TEST(CorePathSequentialLaneChange, HoldsTheTargetAfterRouteRematchingUntilOneSecondOfStableMotion) {
  SequentialScene s;s.call(100,10,0);
  for(int i=0;i<10;++i)expectFirstTarget(s.call(101+.1*i,60+i,3.5,10,0,0,true));
  const auto next=s.call(102,70,3.5,10,0,0,true);
  ASSERT_FALSE(next.points.empty());EXPECT_GT(next.points.back().pose.pose.position.y,6.7);
}
TEST(CorePathSequentialLaneChange, RepeatedObservationCannotReleaseTheSecondLane) {
  SequentialScene s;s.call(100,10,0);
  for(int i=0;i<30;++i)expectFirstTarget(s.call(101,60,3.5,10,0,0,true));
}
TEST(CorePathSequentialLaneChange, StandstillAndObservationGapsCannotCountAsStableDriving) {
  SequentialScene s;s.call(100,10,0);
  for(int i=0;i<12;++i)expectFirstTarget(s.call(101+.1*i,60,3.5,0,0,0,true));
  for(int i=0;i<10;++i)expectFirstTarget(s.call(103+.1*i,60+i,3.5,10,0,0,true));
  expectFirstTarget(s.call(104.5,75,3.5,10,0,0,true));
}
TEST(CorePathSequentialLaneChange, ZeroSteeringDoesNotCompleteWithAnUnalignedHeading) {
  SequentialScene s;s.call(100,10,0);
  for(int i=0;i<15;++i) {
    const auto path=s.call(101+.1*i,60+i,3.5,10,.12,0,true);
    ASSERT_FALSE(path.points.empty());
    // 큰 방향 오차에서의 보정 궤적은 출발 차선을 이용할 수 있지만,
    // 완료로 오인해 그 다음 차선으로 진행하면 안 된다.
    for(const auto& p:path.points)EXPECT_LT(p.pose.pose.position.y,4.1);
  }
}
TEST(CorePathSequentialLaneChange, RearAxleInsideDoesNotReplaceWholeBodyCompletion) {
  SequentialScene s;s.call(100,10,0);
  for(int i=0;i<15;++i)expectFirstTarget(s.call(101+.1*i,60+i,2.5,10,0,0,true));
}
TEST(CorePathSequentialLaneChange, SteeringChangeAtADuplicatePoseResetsConfirmation) {
  SequentialScene s;s.call(100,10,0);
  for(int i=0;i<8;++i)expectFirstTarget(s.call(101+.1*i,60+i,3.5,10,0,0,true));
  expectFirstTarget(s.call(101.7,67,3.5,10,0,.2,true));
  expectFirstTarget(s.call(101.8,68,3.5,10,0,0,true));
  expectFirstTarget(s.call(101.9,69,3.5,10,0,0,true));
  expectFirstTarget(s.call(102,70,3.5,10,0,0,true));
}
TEST(CorePathSequentialLaneChange, CurveRequiredSteeringDoesNotPreventCompletion) {
  SequentialScene s(true);
  s.call(100,0,0,6,0,-std::atan(3.0/30)/.78,false,6.0/30);
  interfaces::Path path;
  for(int i=0;i<=10;++i) {
    const double a=.4+.1*i*6/26.5;
    path=s.call(101+.1*i,26.5*std::sin(a),30-26.5*std::cos(a),6,a,-std::atan(3.0/26.5)/.78,true,6.0/26.5);
    ASSERT_FALSE(path.points.empty());
    if(i<10)for(const auto& p:path.points)
      EXPECT_GT(std::hypot(p.pose.pose.position.x,p.pose.pose.position.y-30),25.2);
  }
  const auto& p=path.points.back();EXPECT_LT(std::hypot(p.pose.pose.position.x,p.pose.pose.position.y-30),24.0);
}
}

TEST(CorePathSequentialLaneChange, AStraddlingBodyCannotRestartSourceLaneHolding) {
  SequentialScene scene;
  path_planner::Route route=lanelet::routing::LaneletPath({scene.lanes[0],scene.lanes[1]});
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=10;ego.pose={{20,0},0};
  EXPECT_TRUE(static_cast<bool>(scene.core->laneHoldReference(route,ego)));
  // 출발 차선의 왼쪽 경계는 y=1.75다. 뒷차축 y=1.0은 아직 안쪽이지만
  // 폭 2.160835m인 차체는 이미 목표 차선과 겹친다. 변경을 되돌리는 현재
  // 차선 유지 후보를 새로 허용하지 않는다. 실제 목표 변경 계산은 유지한다.
  ego.pose.position.y=1.0;
  EXPECT_FALSE(static_cast<bool>(scene.core->laneHoldReference(route,ego)));
}

TEST(CorePathRelativeMerge, JoinsBehindACloseFasterVehicleWithoutYieldBraking) {
  SequentialScene scene;interfaces::Objects objects;interfaces::Object lead;
  lead.id=11;lead.label="vehicle";lead.position.x=10+3.845008+2.25+3;
  lead.position.y=3.5;lead.velocity.x=12;lead.velocity_available=true;
  lead.bbox_size.x=4.5;lead.bbox_size.y=1.8;lead.bbox_size.z=1.5;lead.bbox_pose.orientation.w=1;
  objects.objects.push_back(lead);
  const auto path=scene.call(100,10,0,9,0,0,false,0,&objects);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_GT(path.points.back().pose.pose.position.y,3.0);
  for(const auto& point:path.points)if(point.time_from_start.toSec()<=1)EXPECT_GE(point.velocity,9-1e-6);
}
TEST(CorePathRelativeMerge, AlongsideFasterVehicleDoesNotRequireAStandstill) {
  SequentialScene scene;interfaces::Objects objects;interfaces::Object other;
  other.id=12;other.label="vehicle";other.position.x=10;other.position.y=3.5;
  other.velocity.x=12;other.velocity_available=true;
  other.bbox_size.x=4.5;other.bbox_size.y=1.8;other.bbox_size.z=1.5;other.bbox_pose.orientation.w=1;
  objects.objects.push_back(other);
  const auto path=scene.call(100,10,0,9,0,0,false,0,&objects);
  ASSERT_GE(path.points.size(),2u);
  for(const auto& point:path.points)EXPECT_GT(point.velocity,2);
}
TEST(CorePathRelativeMerge, SlowerAlongsideVehiclePreservesForwardOvertakingOpportunity) {
  SequentialScene scene;interfaces::Objects objects;interfaces::Object other;
  other.id=13;other.label="vehicle";other.position.x=10;other.position.y=3.5;
  other.velocity.x=8;other.velocity_available=true;
  other.bbox_size.x=4.5;other.bbox_size.y=1.8;other.bbox_size.z=1.5;other.bbox_pose.orientation.w=1;
  objects.objects.push_back(other);
  const auto path=scene.call(100,10,0,9,0,0,false,0,&objects);
  ASSERT_GE(path.points.size(),2u);
  for(const auto& point:path.points)if(point.time_from_start.toSec()<=1)EXPECT_GE(point.velocity,9-1e-6);
}
