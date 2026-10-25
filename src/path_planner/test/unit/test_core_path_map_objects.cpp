#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <path_planner/validators/core_path_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <lanelet2_core/utility/Utilities.h>

TEST(CorePathPlanner, UsesMapObjectsWithTranslatedAndRotatedMovingEgo) {
  lanelet::Lanelet lane(1,
      lanelet::LineString3d(2, {lanelet::Point3d(4, 95, 190, 0),
                              lanelet::Point3d(5, 95, 300, 0)}),
      lanelet::LineString3d(3, {lanelet::Point3d(6, 105, 190, 0),
                              lanelet::Point3d(7, 105, 300, 0)}));
  path_planner::Route route = lanelet::routing::LaneletPath({lane});
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(10.0);
  ego.header.frame_id = "map";
  ego.child_frame_id = "base_link";
  ego.pose.pose.position.x = 100;
  ego.pose.pose.position.y = 200;
  ego.pose.pose.orientation.z = ego.pose.pose.orientation.w = std::sqrt(0.5);
  ego.twist.twist.linear.x = 2;
  interfaces::CompetitionVehicleStatus competition;
  competition.header = ego.header;
  competition.velocity_x_mps = 2;
  interfaces::Objects objects;
  objects.header = ego.header;
  path_planner::PredictedObjects predictions;
  predictions.fill(objects);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.z = 9.81;
  path_planner::PathParameters parameters;
  parameters.trajectory_horizon_m = 25;
  parameters.speed_beam_width = 16;
  parameters.speed_time_step_sec = 1;
  path_planner::CorePathPlanner core(parameters);
  const auto free = core.process(ego, competition, objects,
      tf2_msgs::TFMessage{}, imu, route, predictions,ego.header.stamp);
  ASSERT_FALSE(free.points.empty());
  EXPECT_DOUBLE_EQ(free.points.front().velocity, 2);
  EXPECT_EQ(free.points.front().time_from_start.toNSec(), 0);
  for (std::size_t i = 1; i < free.points.size(); ++i)
    EXPECT_GT(free.points[i].time_from_start.toNSec(),
              free.points[i-1].time_from_start.toNSec());
  EXPECT_NEAR(free.points.back().time_from_start.toSec(), 5.0, 1e-8);
  interfaces::Object object;
  object.id = 1;
  object.label = "vehicle";
  object.position.x = 100;
  object.position.y = 201;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.z = object.bbox_pose.orientation.w = std::sqrt(0.5);
  object.bbox_size.x = 4;
  object.bbox_size.y = 40;
  object.bbox_size.z = 1.5;
  object.velocity_available = true;
  objects.objects.push_back(object);
  predictions.fill(objects);
  EXPECT_TRUE(core.process(ego, competition, objects,
      tf2_msgs::TFMessage{}, imu, route, predictions,ego.header.stamp).points.empty());
}

TEST(CorePathPlanner, UsesMeasuredForwardSpeedWhenEkfDriftsNegative) {
  lanelet::Lanelet lane(1,
      lanelet::LineString3d(2, {lanelet::Point3d(4, 0, 5, 0),
                              lanelet::Point3d(5, 100, 5, 0)}),
      lanelet::LineString3d(3, {lanelet::Point3d(6, 0, -5, 0),
                              lanelet::Point3d(7, 100, -5, 0)}));
  path_planner::Route route = lanelet::routing::LaneletPath({lane});
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(10);
  ego.pose.pose.position.x = 10;
  ego.pose.pose.orientation.w = 1;
  ego.twist.twist.linear.x = -0.046;
  interfaces::CompetitionVehicleStatus status;
  status.header = ego.header;
  status.velocity_x_mps = 0.032601453f;
  interfaces::Objects objects;
  objects.header = ego.header;
  path_planner::PredictedObjects predictions;
  predictions.fill(objects);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.z = 9.81;
  path_planner::PathParameters parameters;
  parameters.trajectory_horizon_m = 25;
  parameters.speed_beam_width = 16;
  parameters.speed_time_step_sec = 1;
  path_planner::CorePathPlanner core(parameters);
  const auto path = core.process(ego, status, objects, tf2_msgs::TFMessage{},
                                imu, route, predictions, ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  EXPECT_DOUBLE_EQ(path.points.front().velocity, status.velocity_x_mps);
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 10, 1e-8);
  // 실제 차량의 후진은 EKF 추정 부호와 무관하게 전진 계산으로 위장하지 않는다.
  status.velocity_x_mps = -0.03;
  ego.twist.twist.linear.x = 0.04;
  EXPECT_THROW(core.process(ego, status, objects, tf2_msgs::TFMessage{},
                           imu, route, predictions, ego.header.stamp), std::invalid_argument);
}

TEST(CorePathPlanner, StationaryVelocityAndYawRateRoundingDoesNotBecomeReverseMotion) {
  lanelet::Lanelet lane(1,
      lanelet::LineString3d(2, {lanelet::Point3d(4, 0, 5, 0), lanelet::Point3d(5, 100, 5, 0)}),
      lanelet::LineString3d(3, {lanelet::Point3d(6, 0, -5, 0), lanelet::Point3d(7, 100, -5, 0)}));
  path_planner::Route route = lanelet::routing::LaneletPath({lane});
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(10);
  ego.pose.pose.position.x = 10;
  ego.pose.pose.orientation.w = 1;
  // 실제 정지 로그의 반올림 잔차: 기존 yaw/v 검사로는 후진으로 판정됐다.
  ego.twist.twist.linear.x = -7.453491e-6;
  ego.twist.twist.angular.z = 3.152150e-5;
  interfaces::CompetitionVehicleStatus status;
  status.header = ego.header;
  status.velocity_x_mps = -7.453491e-6f;
  interfaces::Objects objects;
  objects.header = ego.header;
  path_planner::PredictedObjects predictions;
  predictions.fill(objects);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.z = 9.81;
  path_planner::PathParameters parameters;
  parameters.trajectory_horizon_m = 25;
  parameters.speed_beam_width = 16;
  parameters.speed_time_step_sec = 1;
  path_planner::CorePathPlanner core(parameters);
  interfaces::Path path;
  ASSERT_NO_THROW(path = core.process(ego, status, objects, tf2_msgs::TFMessage{},
                                     imu, route, predictions, ego.header.stamp));
  ASSERT_FALSE(path.points.empty());
  EXPECT_DOUBLE_EQ(path.points.front().velocity, 0);
  EXPECT_GT(path.points.back().velocity, 0);
  // 제동 정착 중 실제 로그: 0.000175 m/s와 -0.0263 m/s²가 남아도
  // 차체의 작은 진동을 전진 계획 불능이나 후진으로 판정하지 않는다.
  for(double residual:{-0.0002,0.000174706}) {
    status.velocity_x_mps=residual;status.signed_velocity_mps=residual;status.brake=1;status.accel=0;
    ego.twist.twist.angular.z=-0.000052432;
    imu.linear_acceleration.x=-0.0263209;
    ASSERT_NO_THROW(path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp));
    ASSERT_FALSE(path.points.empty());
    EXPECT_DOUBLE_EQ(path.points.front().velocity,0);
    EXPECT_DOUBLE_EQ(path.points.front().acceleration,0);
  }
  // 1 mm/s 범위 밖의 실제 저속 전진과 큰 초기 가속도는 덮어쓰지 않는다.
  status.velocity_x_mps=.002f;ego.twist.twist.angular.z=0;imu.linear_acceleration.x=0;
  path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_FALSE(path.points.empty());EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  status.velocity_x_mps=0;imu.linear_acceleration.x=.1;
  path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_FALSE(path.points.empty());EXPECT_DOUBLE_EQ(path.points.front().acceleration,.1);
  status.velocity_x_mps = -0.03f;
  EXPECT_THROW(core.process(ego, status, objects, tf2_msgs::TFMessage{},
                           imu, route, predictions, ego.header.stamp), std::invalid_argument);
}

TEST(CorePathInputs, RejectsUnavailableVelocityWithoutReadingItsNaNValues) {
  interfaces::Objects objects;
  objects.objects.resize(1);
  auto& velocity = objects.objects.front().velocity;
  velocity.x = velocity.y = velocity.z = std::numeric_limits<double>::quiet_NaN();
  std::vector<node_pipeline::Kind> issues;
  const path_planner::CorePathInputs validator;
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);

  objects.objects.front().velocity_available = true;
  objects.objects.front().velocity = geometry_msgs::Vector3{};
  path_planner::PredictedObjects predictions;
  predictions.fill(objects);
  predictions[4].objects.front().velocity_available = false;
  auto& unavailable = predictions[4].objects.front().velocity;
  unavailable.x = unavailable.y = unavailable.z = std::numeric_limits<double>::quiet_NaN();
  issues.clear();
  validator(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);
}

TEST(CorePathInputs, RequiresFiniteAvailableVelocityAndPreservesRouteValidation) {
  interfaces::Objects objects;
  objects.objects.resize(1);
  objects.objects.front().velocity_available = true;
  std::vector<node_pipeline::Kind> issues;
  const path_planner::CorePathInputs validator;
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_TRUE(issues.empty());
  objects.objects.front().velocity.y = std::numeric_limits<double>::quiet_NaN();
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::NaN}), issues);
  issues.clear();
  validator(path_planner::Route{}, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);
}

namespace {
struct NativeLaneChangeScene {
  path_planner::SharedMap shared;
  path_planner::Route route;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus status;
  interfaces::Objects objects;
  path_planner::PredictedObjects predictions;
  sensor_msgs::Imu imu;
  explicit NativeLaneChangeScene(bool legal) : shared(makeMap(legal)) {
    route=lanelet::routing::LaneletPath({shared.map->laneletLayer.get(101)});
    ego.header.stamp.fromSec(10);ego.header.frame_id="map";ego.child_frame_id="base_link";
    ego.pose.pose.position.x=10;ego.pose.pose.orientation.w=1;
    ego.twist.twist.linear.x=2;
    status.header=ego.header;status.velocity_x_mps=2;
    objects.header=ego.header;
    interfaces::Object obstacle;obstacle.id=1;obstacle.label="obstacle";
    obstacle.position.x=35;obstacle.bbox_pose.position=obstacle.position;
    obstacle.bbox_pose.orientation.w=1;obstacle.bbox_size.x=2;
    obstacle.bbox_size.y=1.8;obstacle.bbox_size.z=1;
    obstacle.velocity_available=true;obstacle.acceleration_available=true;
    objects.objects.push_back(obstacle);predictions.fill(objects);
    imu.linear_acceleration.z=9.81;
  }
  static path_planner::SharedMapParameters makeMap(bool legal) {
    lanelet::LineString3d right(11,{lanelet::Point3d(1,-20,-1.75,0),lanelet::Point3d(2,180,-1.75,0)});
    lanelet::LineString3d middle(12,{lanelet::Point3d(3,-20,1.75,0),lanelet::Point3d(4,180,1.75,0)});
    lanelet::LineString3d left(13,{lanelet::Point3d(5,-20,5.25,0),lanelet::Point3d(6,180,5.25,0)});
    for(auto* line:{&right,&middle,&left}) {
      line->attributes()["type"]="line_thin";
      line->attributes()["subtype"]="solid";
    }
    middle.attributes()["subtype"]=legal?"dashed":"solid";
    lanelet::Lanelet base(101,middle,right),neighbor(102,left,middle);
    for(auto* lane:{&base,&neighbor}) {
      lane->attributes()["subtype"]="road";lane->attributes()["location"]="urban";
      lane->attributes()["one_way"]="yes";
    }
    return {lanelet::utils::createMap({base,neighbor})};
  }
};
}

namespace {
bool nativeRectanglesSeparated(const interfaces::PathPoint& p,double ox,double oy,
                               double object_length,double object_width) {
  const auto& q=p.pose.pose.orientation;
  const double yaw=std::atan2(2*q.w*q.z,1-2*q.z*q.z);
  const double cs=std::cos(yaw),sn=std::sin(yaw);
  const double cx=p.pose.pose.position.x+(3.845008-.790)/2*cs;
  const double cy=p.pose.pose.position.y+(3.845008-.790)/2*sn;
  for(const auto& axis:std::vector<std::pair<double,double>>{{1,0},{0,1},{cs,sn},{-sn,cs}}) {
    const double distance=std::abs((cx-ox)*axis.first+(cy-oy)*axis.second);
    const double own=(3.845008+.790)/2*std::abs(cs*axis.first+sn*axis.second)+
                      2.160835/2*std::abs(-sn*axis.first+cs*axis.second);
    const double other=(object_length/2+.25)*std::abs(axis.first)+
                       (object_width/2+.25)*std::abs(axis.second);
    if(distance>own+other)return true;
  }
  return false;
}
void expectNativeUnionAndStaticObstacleClear(const interfaces::Path& path) {
  for(const auto& p:path.points) {
    const auto& q=p.pose.pose.orientation;
    const double yaw=std::atan2(2*q.w*q.z,1-2*q.z*q.z),cs=std::cos(yaw),sn=std::sin(yaw);
    for(double x:{3.845008+.05,-.790-.05})for(double y:{-2.160835/2-.05,2.160835/2+.05}) {
      const double wx=p.pose.pose.position.x+x*cs-y*sn;
      const double wy=p.pose.pose.position.y+x*sn+y*cs;
      EXPECT_GT(wx,-20);EXPECT_LT(wx,180);EXPECT_GT(wy,-1.75);EXPECT_LT(wy,5.25);
    }
    EXPECT_TRUE(nativeRectanglesSeparated(p,35,0,2,1.8));
  }
}
}

TEST(CorePathPlanner, LegalDashedNeighborAllowsAvoidanceBeyondOriginalLaneBoundary) {
  NativeLaneChangeScene scene(true);
  const auto rules=lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany,lanelet::Participants::Vehicle);
  const auto graph=lanelet::routing::RoutingGraph::build(*scene.shared.map,*rules);
  ASSERT_TRUE(static_cast<bool>(graph->left(scene.shared.map->laneletLayer.get(101))));
  path_planner::CorePathPlanner core(path_planner::PathParameters{},scene.shared);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  double maximum_y=0;
  for(const auto& p:path.points)maximum_y=std::max(maximum_y,p.pose.pose.position.y);
  EXPECT_GT(maximum_y,2.2);
  EXPECT_GT(path.points.back().pose.pose.position.x,30);
  expectNativeUnionAndStaticObstacleClear(path);
  // 첫 5초에 완전히 통과할 거리를 가정하지 않고, 같은 Core로 다음 계획을
  // 계산한다. 완벽한 추종 입력의 계산 시험이며 실제 MPC 주행 시험과 구분한다.
  const auto& end=path.points.back();const auto& before=path.points[path.points.size()-2];
  const double yaw=2*std::atan2(end.pose.pose.orientation.z,end.pose.pose.orientation.w);
  const double before_yaw=2*std::atan2(before.pose.pose.orientation.z,before.pose.pose.orientation.w);
  const double ds=std::hypot(end.pose.pose.position.x-before.pose.pose.position.x,
                             end.pose.pose.position.y-before.pose.pose.position.y);
  scene.ego.header.stamp+=ros::Duration(5);scene.ego.pose.pose=end.pose.pose;
  scene.ego.twist.twist.linear.x=end.velocity;
  scene.ego.twist.twist.angular.z=ds>1e-9?std::remainder(yaw-before_yaw,2*std::acos(-1.0))/ds*end.velocity:0;
  scene.status.header=scene.ego.header;scene.status.velocity_x_mps=end.velocity;
  scene.imu.linear_acceleration.x=end.acceleration;
  scene.objects.header=scene.ego.header;scene.predictions.fill(scene.objects);
  const auto next=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                                scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(next.points.empty());
  expectNativeUnionAndStaticObstacleClear(next);
  EXPECT_GT(next.points.back().pose.pose.position.x,37);
}

TEST(CorePathPlanner, SolidNeighborDoesNotGrantPermissionForObstacleAvoidance) {
  NativeLaneChangeScene scene(false);
  path_planner::CorePathPlanner core(path_planner::PathParameters{},scene.shared);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  for(const auto& p:path.points)EXPECT_LT(p.pose.pose.position.y,1.75);
  EXPECT_LT(path.points.back().pose.pose.position.x,31);
  EXPECT_LT(path.points.back().velocity,scene.status.velocity_x_mps);
}

TEST(CorePathPlanner, ParallelLaneChangeInNativeRouteDoesNotFoldRoadBackOverItself) {
  NativeLaneChangeScene scene(true);
  scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(101),
                                          scene.shared.map->laneletLayer.get(102)});
  scene.objects.objects.clear();scene.predictions.fill(scene.objects);
  path_planner::CorePathPlanner core(path_planner::PathParameters{},scene.shared);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(path.points.front().pose.pose.position.x,10,1e-8);
  EXPECT_NEAR(path.points.front().pose.pose.position.y,0,1e-8);
  EXPECT_GT(path.points.back().pose.pose.position.x,12);
  for(std::size_t i=1;i<path.points.size();++i)
    EXPECT_GE(path.points[i].pose.pose.position.x,path.points[i-1].pose.pose.position.x-1e-8);
}

TEST(CorePathPlanner, ParallelRouteKeepsMovingTowardAdjacentTargetAfterEarlyAvoidance) {
  NativeLaneChangeScene scene(true);
  scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(101),
                                          scene.shared.map->laneletLayer.get(102)});
  scene.objects.objects.clear();scene.predictions.fill(scene.objects);
  scene.ego.pose.pose.position.x=30;scene.ego.pose.pose.position.y=1.2;
  path_planner::PathParameters parameters;
  parameters.trajectory_horizon_m=25;parameters.speed_beam_width=16;
  parameters.speed_time_step_sec=1;
  path_planner::CorePathPlanner core(parameters,scene.shared);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  // 목표는 y=3.5인 바로 옆 차선이다. 이미 y=1.2까지 이동한 차량을
  // 긴 원본 lanelet의 출발 차선 기준선으로 되돌리는 경로를 잡는다.
  for(const auto& point:path.points)
    EXPECT_GE(point.pose.pose.position.y,1.15);
  EXPECT_GT(path.points.back().pose.pose.position.y,3.0);
}

TEST(CorePathPlanner, AdjacentTargetRouteSwitchDoesNotReplaceAnUnfinishedSourceReference) {
  NativeLaneChangeScene scene(true);
  scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(101),
                                          scene.shared.map->laneletLayer.get(102)});
  scene.objects.objects.clear();scene.predictions.fill(scene.objects);
  scene.ego.pose.pose.position.x=30;scene.ego.pose.pose.position.y=2;
  path_planner::PathParameters parameters;
  parameters.trajectory_horizon_m=25;parameters.speed_beam_width=16;
  parameters.speed_time_step_sec=1;
  path_planner::CorePathPlanner core(parameters,scene.shared);
  const auto before=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                                 scene.route,scene.predictions,scene.ego.header.stamp);
  scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(102)});
  const auto after=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                                scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(before.points.empty());ASSERT_FALSE(after.points.empty());
  const auto lateral_at=[](const interfaces::Path& path,double x) {
    for(std::size_t i=1;i<path.points.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position;
      const auto& b=path.points[i].pose.pose.position;
      if(a.x<=x && x<=b.x && b.x>a.x)return a.y+(b.y-a.y)*(x-a.x)/(b.x-a.x);
    }
    return std::numeric_limits<double>::quiet_NaN();
  };
  const double before_y=lateral_at(before,40),after_y=lateral_at(after,40);
  ASSERT_TRUE(std::isfinite(before_y));ASSERT_TRUE(std::isfinite(after_y));
  EXPECT_NEAR(before_y,after_y,.3);
}

TEST(CorePathPlanner, OneWayLaneChangeCannotReturnAcrossItsForbiddenSide) {
  NativeLaneChangeScene scene(true);
  scene.shared.map->lineStringLayer.get(12).attributes()["subtype"]="solid_dashed";
  const auto rules=lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany,lanelet::Participants::Vehicle);
  const auto graph=lanelet::routing::RoutingGraph::build(*scene.shared.map,*rules);
  ASSERT_TRUE(static_cast<bool>(graph->left(scene.shared.map->laneletLayer.get(101))));
  ASSERT_FALSE(static_cast<bool>(graph->right(scene.shared.map->laneletLayer.get(102))));
  scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(101),
                                          scene.shared.map->laneletLayer.get(102)});
  scene.ego.pose.pose.position.y=3.5;
  scene.objects.objects.front().position.x=25;scene.objects.objects.front().position.y=3.5;
  scene.objects.objects.front().bbox_pose.position=scene.objects.objects.front().position;
  scene.predictions.fill(scene.objects);
  path_planner::CorePathPlanner core(path_planner::PathParameters{},scene.shared);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  for(const auto& p:path.points) {
    EXPECT_GE(p.pose.pose.position.y,1.75);
    EXPECT_TRUE(nativeRectanglesSeparated(p,25,3.5,2,1.8));
  }
  EXPECT_LT(path.points.back().pose.pose.position.x,21);
  EXPECT_DOUBLE_EQ(path.points.back().velocity,0);
}

TEST(CorePathPlanner, OneWayLaneChangeKeepsStraddlingBodyAfterGlobalRouteSwitchesToTarget) {
  NativeLaneChangeScene scene(true);
  scene.shared.map->lineStringLayer.get(12).attributes()["subtype"]="solid_dashed";
  scene.objects.objects.clear();scene.predictions.fill(scene.objects);
  const auto source=scene.shared.map->laneletLayer.get(101);
  const auto target=scene.shared.map->laneletLayer.get(102);
  path_planner::CorePathPlanner core(path_planner::PathParameters{},scene.shared);
  scene.route=lanelet::routing::LaneletPath({source,target});
  ASSERT_FALSE(core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                            scene.route,scene.predictions,scene.ego.header.stamp).points.empty());
  // 실제 차선 변경 중 뒷차축이 경계 1.75m를 통과해 GlobalPath가 target만
  // 반환해도 차체 오른쪽은 source에 남아 있다. 출발 차선 공간을 잃지 않고
  // 목표 차선으로 계속 들어가되 금지된 source 방향 복귀는 허용하지 않는다.
  scene.route=lanelet::routing::LaneletPath({target});
  scene.ego.pose.pose.position.y=2.0;
  scene.ego.header.stamp+=ros::Duration(1);scene.status.header=scene.ego.header;
  scene.objects.header=scene.ego.header;scene.predictions.fill(scene.objects);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  for(const auto& p:path.points) {
    EXPECT_GE(p.pose.pose.position.y,1.75);
    const auto& q=p.pose.pose.orientation;
    const double yaw=2*std::atan2(q.z,q.w);
    for(double x:{3.845008+.05,-.790-.05})for(double y:{-2.160835/2-.05,2.160835/2+.05}) {
      const double lateral=p.pose.pose.position.y+x*std::sin(yaw)+y*std::cos(yaw);
      EXPECT_GT(lateral,-1.75);EXPECT_LT(lateral,5.25);
    }
  }
  EXPECT_GT(path.points.back().pose.pose.position.y,3.0);
  EXPECT_GT(path.points.back().pose.pose.position.x,20.0);
}

TEST(CorePathPlanner, IncomingOneWaySpaceDoesNotPermitForbiddenReturnFromTargetOnlyRoute) {
  NativeLaneChangeScene scene(true);
  scene.shared.map->lineStringLayer.get(12).attributes()["subtype"]="solid_dashed";
  scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(102)});
  scene.ego.pose.pose.position.y=3.5;
  scene.objects.objects.front().position.x=25;scene.objects.objects.front().position.y=3.5;
  scene.objects.objects.front().bbox_pose.position=scene.objects.objects.front().position;
  scene.predictions.fill(scene.objects);
  path_planner::CorePathPlanner core(path_planner::PathParameters{},scene.shared);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  for(const auto& p:path.points) {
    EXPECT_GE(p.pose.pose.position.y,1.75);
    EXPECT_TRUE(nativeRectanglesSeparated(p,25,3.5,2,1.8));
  }
  EXPECT_LT(path.points.back().pose.pose.position.x,21);
  EXPECT_DOUBLE_EQ(path.points.back().velocity,0);
}

TEST(CorePathPlanner, NativeReferenceCacheKeepsLaneletTravelOrientationInItsKey) {
  NativeLaneChangeScene scene(true);
  scene.shared.map->laneletLayer.get(101).attributes()["one_way"]="no";
  scene.shared.map->laneletLayer.get(102).attributes()["one_way"]="no";
  scene.objects.objects.clear();scene.predictions.fill(scene.objects);
  path_planner::PathParameters p;p.trajectory_horizon_m=25;p.speed_beam_width=16;p.speed_time_step_sec=1;
  path_planner::CorePathPlanner core(p,scene.shared);
  const auto forward=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                                  scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(forward.points.empty());
  scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(101).invert()});
  scene.ego.header.stamp+=ros::Duration(5);scene.ego.pose.pose.position.x=100;
  scene.ego.pose.pose.orientation.z=1;scene.ego.pose.pose.orientation.w=0;
  scene.status.header=scene.ego.header;scene.objects.header=scene.ego.header;
  scene.predictions.fill(scene.objects);
  const auto reverse_heading=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                                          scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(reverse_heading.points.empty());
  EXPECT_NEAR(reverse_heading.points.front().pose.pose.position.x,100,1e-8);
  EXPECT_LT(reverse_heading.points.back().pose.pose.position.x,100);
}

TEST(CorePathPlanner, ShortLongitudinalPredecessorKeepsExistingRoadUnderRearBumper) {
  for(bool include_older:{true,false}) {
    SCOPED_TRACE(include_older);
    const std::array<double,4> x{{-20,-.1,0,100}};
    std::vector<lanelet::Point3d> left,right;
    for(std::size_t i=0;i<x.size();++i) {
      left.emplace_back(400+2*i,x[i],1.75,0);
      right.emplace_back(401+2*i,x[i],-1.75,0);
    }
    lanelet::Lanelets lanes;
    for(std::size_t i=include_older?0:1;i+1<x.size();++i) {
      lanelet::Lanelet lane(301+i,lanelet::LineString3d(500+2*i,{left[i],left[i+1]}),
                                  lanelet::LineString3d(501+2*i,{right[i],right[i+1]}));
      lane.attributes()["subtype"]="road";lane.attributes()["location"]="urban";
      lane.attributes()["one_way"]="yes";lanes.push_back(lane);
    }
    path_planner::SharedMap shared(path_planner::SharedMapParameters{lanelet::utils::createMap(lanes)});
    path_planner::Route route=lanelet::routing::LaneletPath(
        {shared.map->laneletLayer.get(302),shared.map->laneletLayer.get(303)});
    nav_msgs::Odometry ego;ego.header.stamp.fromSec(10);ego.header.frame_id="map";
    ego.child_frame_id="base_link";ego.pose.pose.position.x=.2;ego.pose.pose.orientation.w=1;
    interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.brake=1;
    interfaces::Objects objects;objects.header=ego.header;
    path_planner::PredictedObjects predictions;predictions.fill(objects);
    sensor_msgs::Imu imu;imu.linear_acceleration.z=9.81;
    path_planner::PathParameters parameters;parameters.trajectory_horizon_m=25;
    parameters.speed_time_step_sec=1;parameters.speed_beam_width=16;
    path_planner::CorePathPlanner core(parameters,shared);
    const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
    if(include_older) {
      // 10cm predecessor 앞의 실제 연결 도로에 뒷범퍼가 남아 있다.
      ASSERT_FALSE(path.points.empty());
      EXPECT_GT(path.points.back().pose.pose.position.x,1.2);
    } else {
      // 실제 지도에도 이전 도로가 없으면 경계를 가상 연장하지 않는다.
      EXPECT_TRUE(path.points.empty());
    }
  }
}

TEST(CorePathPlanner, LegalNeighborUsesPredictedRearTrafficBeforeMerging) {
  NativeLaneChangeScene scene(true);
  interfaces::Object rear;rear.id=2;rear.label="vehicle";
  rear.position.x=0;rear.position.y=3.5;rear.bbox_pose.position=rear.position;
  rear.bbox_pose.orientation.w=1;rear.bbox_size.x=4.6;rear.bbox_size.y=2.1;
  rear.bbox_size.z=1.6;rear.velocity.x=10;rear.velocity_available=true;
  rear.acceleration_available=true;scene.objects.objects.push_back(rear);
  scene.predictions.fill(scene.objects);
  for(std::size_t i=0;i<scene.predictions.size();++i) {
    auto& future=scene.predictions[i].objects.back();
    future.position.x=10*.5*(i+1);future.bbox_pose.position=future.position;
  }
  path_planner::CorePathPlanner core(path_planner::PathParameters{},scene.shared);
  const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,
                               scene.route,scene.predictions,scene.ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  expectNativeUnionAndStaticObstacleClear(path);
  for(const auto& p:path.points)
    EXPECT_TRUE(nativeRectanglesSeparated(p,10*p.time_from_start.toSec(),3.5,4.6,2.1));
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace {
struct PedestrianCautionScene {
  path_planner::Route route;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus status;
  sensor_msgs::Imu imu;
  tf2_msgs::TFMessage tf;
  interfaces::Objects objects;
  PedestrianCautionScene(double x,double y) {
    lanelet::Lanelet lane(9001,
        lanelet::LineString3d(9002,{lanelet::Point3d(9004,-10,1.75,0),lanelet::Point3d(9005,150,1.75,0)}),
        lanelet::LineString3d(9003,{lanelet::Point3d(9006,-10,-1.75,0),lanelet::Point3d(9007,150,-1.75,0)}));
    route=lanelet::routing::LaneletPath({lane});
    ego.header.frame_id="map";ego.child_frame_id="base_link";ego.header.stamp.fromSec(10);
    ego.pose.pose.orientation.w=1;ego.twist.twist.linear.x=6;
    status.header=ego.header;status.header.frame_id="base_link";status.velocity_x_mps=status.signed_velocity_mps=6;
    imu.header=ego.header;imu.header.frame_id="imu_link";imu.linear_acceleration.z=9.81;
    geometry_msgs::TransformStamped transform;transform.header=ego.header;transform.child_frame_id="base_link";
    transform.transform.rotation.w=1;tf.transforms.push_back(transform);
    objects.header=ego.header;
    interfaces::Object pedestrian;pedestrian.id=37;pedestrian.label="pedestrian";
    pedestrian.position.x=x;pedestrian.position.y=y;pedestrian.bbox_pose.position=pedestrian.position;
    pedestrian.bbox_pose.orientation.w=1;pedestrian.bbox_size.x=pedestrian.bbox_size.y=.6;
    pedestrian.bbox_size.z=1.8;pedestrian.velocity_available=pedestrian.acceleration_available=true;
    objects.objects.push_back(pedestrian);
  }
  static path_planner::CorePathPlanner makeCore() {
    const std::string package=NP_PACKAGE_DIR;
    auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
        package+"/config/path_planner/parameters/core_path_planner.yaml",package));
    parameters.max_speed_mps=6;
    // 이 fixture는 4m/s 접근 한도 자체를 검사하므로 운용 YAML 상한과 분리한다.
    parameters.pedestrian_caution_speed_mps=4;
    // 기존 bbox 경계와 ID 유지·해제 동작을 새 도로 밖 중심점 설정과 분리한다.
    parameters.pedestrian_caution_lateral_margin_m=0;
    return path_planner::CorePathPlanner(parameters);
  }
  interfaces::Path run() const {
    auto core=makeCore();return run(core);
  }
  interfaces::Path run(path_planner::CorePathPlanner& core) const {
    path_planner::PredictedObjects predictions;predictions.fill(objects);
    return core.process(ego,status,objects,tf,imu,route,predictions,ego.header.stamp);
  }
};
}

TEST(CorePathPlanner, RoadsidePedestrianApproachLowersSpeedBeforeMotionWithoutInventingAStop) {
  // 도로 옆 정지 보행자가 움직인 뒤에만 반응하면 계획 계산·전달 동안
  // 제동 거리를 잃을 수 있다. 관측 속도 0과 bbox는 유지하고, 이미 존재하는
  // 도로/동역학 검사와 ceiling recovery로 전방 접근 속도를 미리 낮춘다.
  ros::Time::init();const PedestrianCautionScene scene(20,3);
  const auto path=scene.run();ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,6);
  const auto at_two=std::lower_bound(path.points.begin(),path.points.end(),2.,
      [](const interfaces::PathPoint& q,double t){return q.time_from_start.toSec()<t;});
  ASSERT_NE(at_two,path.points.end());EXPECT_LT(at_two->velocity,5);
  EXPECT_GT(at_two->velocity,3);EXPECT_GT(path.points.back().velocity,0);
  EXPECT_GT(path.points.back().pose.pose.position.x,10);
  EXPECT_NEAR(path.points.back().time_from_start.toSec(),5,1e-8);
  for(std::size_t i=1;i<path.points.size();++i) {
    const auto& a=path.points[i-1];const auto& b=path.points[i];
    const double dt=(b.time_from_start-a.time_from_start).toSec();ASSERT_GT(dt,0);
    EXPECT_LE(std::abs(b.acceleration-a.acceleration),4*dt+1e-6);
    EXPECT_GE(b.acceleration,-5-1e-6);EXPECT_LE(b.acceleration,2+1e-6);
    EXPECT_TRUE(std::isfinite(b.velocity));
  }
}

TEST(CorePathPlanner, PedestriansOutsideTheForwardRoadPreviewPreserveTheCruisePlan) {
  ros::Time::init();
  for(const auto position:std::vector<std::pair<double,double>>{{20,20},{-20,0},{120,0}}) {
    SCOPED_TRACE(position.first);
    SCOPED_TRACE(position.second);
    const PedestrianCautionScene scene(position.first,position.second);const auto path=scene.run();
    ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,6);
    EXPECT_NEAR(path.points.back().velocity,6,1e-8);
    EXPECT_NEAR(path.points.back().pose.pose.position.x,30,1e-5);
  }
}

TEST(CorePathPlanner, PedestrianCautionPreservesAnExplicitZeroSpeedRequest) {
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  parameters.max_speed_mps=6;path_planner::CorePathPlanner core(parameters);
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=6;
  std::vector<stier::ReferencePoint> road;
  for(int x=-10;x<=150;x+=5) road.push_back({{static_cast<double>(x),0},-1.75,1.75});
  stier::PredictedObject pedestrian;pedestrian.id=37;pedestrian.kind=stier::ObjectKind::Pedestrian;
  pedestrian.pose.position={20,3};pedestrian.length_m=pedestrian.width_m=.6;pedestrian.source_stamp_sec=10;
  stier::Predictions predictions;predictions.reference_stamp_sec=10;predictions.current.push_back(pedestrian);
  for(auto& slice:predictions.future) slice=predictions.current;
  stier::PlanningConstraints limits;limits.maximum_speed_mps=0;
  const auto path=core.plan(ego,road,predictions,limits);
  ASSERT_GE(path.size(),2u);EXPECT_DOUBLE_EQ(path.front().speed_mps,6);
  EXPECT_DOUBLE_EQ(path.back().speed_mps,0);EXPECT_DOUBLE_EQ(path.back().acceleration_mps2,0);
  EXPECT_GT(path.back().pose.position.x,0);EXPECT_LT(path.back().pose.position.x,15);
}

TEST(CorePathPlanner, PedestrianCautionPersistsAcrossObservedBoxChangesUntilTheObjectPasses) {
  // 같은 추적 객체의 자세·크기 변동으로 주변 판정이 경계를 오가도 이미
  // 시작한 접근 감속을 취소하지 않는다. 실제 뒤범퍼를 지난 뒤에는 해제한다.
  auto core=PedestrianCautionScene::makeCore();PedestrianCautionScene scene(20,3.45);
  auto path=scene.run(core);ASSERT_GE(path.points.size(),2u);
  EXPECT_LE(path.points.back().velocity,4+1e-8);
  scene.objects.objects.front().bbox_size.x=scene.objects.objects.front().bbox_size.y=.2;
  path=scene.run(core);ASSERT_GE(path.points.size(),2u);
  EXPECT_LE(path.points.back().velocity,4+1e-8);
  auto& object=scene.objects.objects.front();object.position.x=-5;object.bbox_pose.position=object.position;
  path=scene.run(core);ASSERT_GE(path.points.size(),2u);EXPECT_NEAR(path.points.back().velocity,6,1e-8);
  object.position.x=20;object.bbox_pose.position=object.position;
  path=scene.run(core);ASSERT_GE(path.points.size(),2u);EXPECT_NEAR(path.points.back().velocity,6,1e-8);
}

TEST(CorePathPlanner, PedestrianCautionDoesNotRetainAnAbsentOrOutOfPreviewTrack) {
  for(bool disappear:{false,true}) {
    SCOPED_TRACE(disappear);auto core=PedestrianCautionScene::makeCore();PedestrianCautionScene scene(20,3.45);
    auto path=scene.run(core);ASSERT_GE(path.points.size(),2u);EXPECT_LE(path.points.back().velocity,4+1e-8);
    auto object=scene.objects.objects.front();
    if(disappear) scene.objects.objects.clear();
    else {scene.objects.objects.front().position.y=100;scene.objects.objects.front().bbox_pose.position=scene.objects.objects.front().position;}
    path=scene.run(core);ASSERT_GE(path.points.size(),2u);EXPECT_NEAR(path.points.back().velocity,6,1e-8);
    object.bbox_size.x=object.bbox_size.y=.2;scene.objects.objects={object};
    path=scene.run(core);ASSERT_GE(path.points.size(),2u);EXPECT_NEAR(path.points.back().velocity,6,1e-8);
  }
}

TEST(CorePathPlanner, MissionNormalSpeedLimitIsPlannedBeforeTheFinalFilter) {
 ros::Time::init();NativeLaneChangeScene scene(true);
 scene.objects.objects.clear();scene.predictions.fill(scene.objects);
 scene.ego.twist.twist.linear.x=15;scene.status.velocity_x_mps=15;
 path_planner::PathParameters p;p.max_speed_mps=100./3.6;p.trajectory_horizon_m=125;
 p.normal_speed_limit_mps=60./3.6;p.exception_target_speed_mps=100./3.6;
 p.speed_exemption_link_ids={"EXEMPT"};
 path_planner::CorePathPlanner core(p,scene.shared);
 const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,scene.route,scene.predictions,scene.ego.header.stamp);
 ASSERT_GE(path.points.size(),2u);
 for(const auto& point:path.points)EXPECT_LE(point.velocity,60./3.6+1e-8);
}

namespace {
void prepareMissionSpeedExit(NativeLaneChangeScene& scene) {
 auto base=scene.shared.map->laneletLayer.get(101);base.attributes()["mgeo:link_id"]="EXEMPT";
 lanelet::Lanelet next(201,
   lanelet::LineString3d(211,{base.leftBound().back(),lanelet::Point3d(221,300,1.75,0)}),
   lanelet::LineString3d(212,{base.rightBound().back(),lanelet::Point3d(222,300,-1.75,0)}));
 next.attributes()["subtype"]="road";next.attributes()["location"]="urban";
 next.attributes()["one_way"]="yes";next.attributes()["mgeo:link_id"]="LOCAL";
 scene.shared.map->add(next);scene.route=lanelet::routing::LaneletPath({base,next});
 scene.objects.objects.clear();scene.predictions.fill(scene.objects);scene.status.link_id="EXEMPT";
}
path_planner::PathParameters missionSpeedParameters() {
 path_planner::PathParameters p;p.max_speed_mps=100./3.6;p.trajectory_horizon_m=150;
 p.normal_speed_limit_mps=60./3.6;p.exception_target_speed_mps=100./3.6;
 p.speed_exemption_link_ids={"EXEMPT"};return p;
}
}
TEST(CorePathPlanner, PlansNormalSpeedBeforeTheHighwayExceptionEnds) {
 ros::Time::init();NativeLaneChangeScene scene(true);prepareMissionSpeedExit(scene);
 scene.ego.pose.pose.position.x=110;scene.ego.twist.twist.linear.x=25;scene.status.velocity_x_mps=25;
 auto p=missionSpeedParameters();path_planner::CorePathPlanner core(p,scene.shared);
 auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,scene.route,scene.predictions,scene.ego.header.stamp);
 ASSERT_GE(path.points.size(),2u);EXPECT_LE(path.points.back().velocity,60./3.6+1e-8);
 for(const auto& point:path.points)if(point.pose.pose.position.x+p.vehicle.front_m>=180)
   EXPECT_LE(point.velocity,60./3.6+1e-8);
 // 진행 중 속도가 낮아져 감속 개시 거리 계산이 줄어도 목표를 고주로 속도로 되돌리지 않는다.
 scene.ego.pose.pose.position.x=120;scene.ego.twist.twist.linear.x=20.1;scene.status.velocity_x_mps=20.1;
 scene.ego.header.stamp+=ros::Duration(.05);scene.status.header=scene.ego.header;
 scene.objects.header=scene.ego.header;scene.predictions.fill(scene.objects);
 path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,scene.route,scene.predictions,scene.ego.header.stamp);
 ASSERT_GE(path.points.size(),2u);EXPECT_LE(path.points.back().velocity,60./3.6+1e-8);
}
TEST(CorePathPlanner, DoesNotSlowToNormalLimitWhileHighwayExitIsStillDistant) {
 ros::Time::init();NativeLaneChangeScene scene(true);prepareMissionSpeedExit(scene);
 scene.ego.twist.twist.linear.x=15;scene.status.velocity_x_mps=15;
 auto p=missionSpeedParameters();path_planner::CorePathPlanner core(p,scene.shared);
 const auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,scene.route,scene.predictions,scene.ego.header.stamp);
 ASSERT_GE(path.points.size(),2u);EXPECT_GT(path.points.back().velocity,60./3.6);
}
TEST(CorePathPlanner, LoadsExistingOperatingTargetsAndOfficialExemptionLinks) {
 const std::string pkg=NP_PACKAGE_DIR;
 const auto p=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
     pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
 EXPECT_DOUBLE_EQ(p.max_speed_mps,100./3.6);
 EXPECT_DOUBLE_EQ(p.normal_speed_limit_mps,60./3.6);
 EXPECT_DOUBLE_EQ(p.exception_target_speed_mps,100./3.6);
 EXPECT_DOUBLE_EQ(p.trajectory_horizon_m,150);
 EXPECT_DOUBLE_EQ(p.output_step_m,.25);
 EXPECT_EQ(p.speed_exemption_link_ids.count("A2256W000411"),1u);
 EXPECT_EQ(p.speed_exemption_link_ids.count("A2256W000153"),1u);
 EXPECT_EQ(p.speed_exemption_link_ids.count("A2256W000451"),0u);
}

TEST(CorePathPlanner, ConfiguredPedestrianApproachUsesTenKphWithoutDisablingStopConstraints) {
  ros::Time::init();
  const std::string package=NP_PACKAGE_DIR;
  const auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  EXPECT_DOUBLE_EQ(parameters.pedestrian_caution_speed_mps,10.0/3.6);
  // 속도별 접근 거리 안에서 설정한 10km/h 상한과 초기 속도 보존을 확인한다.
  PedestrianCautionScene scene(30,3);
  scene.ego.twist.twist.linear.x=12;
  scene.status.velocity_x_mps=scene.status.signed_velocity_mps=12;
  path_planner::CorePathPlanner core(parameters);
  const auto path=scene.run(core);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,12.0);
  EXPECT_LE(path.points.back().velocity,10.0/3.6+1e-6);
  EXPECT_GE(path.points.back().velocity,2.5);
  // 10km/h는 하한이 아니다. jerk 제한 감속의 작은 undershoot도 허용한다.
  for(const auto& point:path.points) {
    EXPECT_GE(point.velocity,0);EXPECT_GE(point.acceleration,-5-1e-6);
    EXPECT_LE(point.acceleration,2+1e-6);
  }
}

TEST(CorePathPlanner, NearbyPedestrianSlowsBeforeMotionWhilePreservingTenKphCeiling) {
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  const auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  EXPECT_DOUBLE_EQ(parameters.pedestrian_caution_speed_mps,10.0/3.6);
  PedestrianCautionScene scene(20,3);scene.ego.twist.twist.linear.x=12;
  scene.status.velocity_x_mps=scene.status.signed_velocity_mps=12;
  path_planner::CorePathPlanner core(parameters);const auto path=scene.run(core);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,12);
  EXPECT_LE(path.points.back().velocity,10.0/3.6+1e-6);EXPECT_GT(path.points.back().velocity,0);
  EXPECT_GT(path.points.back().pose.pose.position.x,10);
}

// 옆 차선 뒤차를 먼저 보내려고 감속하지 않고 현재 차선에서 전진한 뒤,
// 간격이 생긴 다음 재계획에서 합법적 변경을 마치는 native 입출력 회귀다.
TEST(CorePathPlanner, KeepsSpeedAndCompletesTheChangeWhenTheRearVehicleFallsBehind) {
 NativeLaneChangeScene scene(true);scene.objects.objects.clear();
 scene.route=lanelet::routing::LaneletPath({scene.shared.map->laneletLayer.get(101),scene.shared.map->laneletLayer.get(102)});
 scene.ego.twist.twist.linear.x=10;scene.status.velocity_x_mps=scene.status.signed_velocity_mps=10;
 path_planner::PathParameters params;path_planner::CorePathPlanner core(params,scene.shared);
 double elapsed=0,minimum_speed=10;bool completed=false;
 for(int turn=0;turn<16;++turn) {
  scene.ego.header.stamp.fromSec(10+elapsed);scene.status.header=scene.ego.header;scene.objects.header=scene.ego.header;
  interfaces::Object rear;rear.id=2;rear.label="vehicle";rear.position.x=8+12*elapsed;rear.position.y=3.5;rear.bbox_pose.position=rear.position;rear.bbox_pose.orientation.w=1;rear.bbox_size.x=4.6;rear.bbox_size.y=2.1;rear.bbox_size.z=1.6;rear.velocity.x=12;rear.velocity_available=rear.acceleration_available=true;scene.objects.objects={rear};scene.predictions.fill(scene.objects);
  for(std::size_t j=0;j<scene.predictions.size();++j){auto& q=scene.predictions[j].objects[0];q.position.x+=12*.5*(j+1);q.bbox_pose.position=q.position;}
  auto path=core.process(scene.ego,scene.status,scene.objects,{},scene.imu,scene.route,scene.predictions,scene.ego.header.stamp);ASSERT_GE(path.points.size(),2u);
  for(const auto& q:path.points)EXPECT_TRUE(nativeRectanglesSeparated(q,rear.position.x+12*q.time_from_start.toSec(),3.5,4.6,2.1));
  auto it=std::lower_bound(path.points.begin(),path.points.end(),.5,[](const interfaces::PathPoint& q,double t){return q.time_from_start.toSec()<t;});ASSERT_NE(it,path.points.end());
  elapsed+=it->time_from_start.toSec();scene.ego.pose.pose=it->pose.pose;scene.status.velocity_x_mps=scene.status.signed_velocity_mps=scene.ego.twist.twist.linear.x=it->velocity;scene.imu.linear_acceleration.x=it->acceleration;
  const auto& prev=*(it-1);double yaw=2*std::atan2(it->pose.pose.orientation.z,it->pose.pose.orientation.w),old_yaw=2*std::atan2(prev.pose.pose.orientation.z,prev.pose.pose.orientation.w);double ds=std::hypot(it->pose.pose.position.x-prev.pose.pose.position.x,it->pose.pose.position.y-prev.pose.pose.position.y);scene.ego.twist.twist.angular.z=it->velocity*std::remainder(yaw-old_yaw,2*M_PI)/ds;
  minimum_speed=std::min(minimum_speed,it->velocity);
  if(it->pose.pose.position.y>3.2){completed=true;break;}
 }
 EXPECT_GE(minimum_speed,9.5);EXPECT_TRUE(completed);
}

TEST(CorePathPlanner, LowSpeedDoesNotApplyTheDistantPedestrianCeiling) {
  ros::Time::init();PedestrianCautionScene scene(35,3);
  auto core=PedestrianCautionScene::makeCore();const auto path=scene.run(core);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_NEAR(path.points.back().velocity,6,1e-6);
}
TEST(CorePathPlanner, DistantPedestrianPreservesCruiseUntilTheSpeedDependentApproach) {
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  parameters.max_speed_mps=12;
  PedestrianCautionScene scene(55,3);scene.ego.twist.twist.linear.x=12;
  scene.status.velocity_x_mps=scene.status.signed_velocity_mps=12;
  path_planner::CorePathPlanner core(parameters);const auto path=scene.run(core);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,12);
  EXPECT_NEAR(path.points.back().velocity,12,1e-6);
}
TEST(CorePathPlanner, SixtyKphApproachKeepsMoreThanFortyMetersOfBrakingReserve) {
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  PedestrianCautionScene scene(50,3);scene.ego.twist.twist.linear.x=60./3.6;
  scene.status.velocity_x_mps=scene.status.signed_velocity_mps=60./3.6;
  path_planner::CorePathPlanner core(parameters);const auto path=scene.run(core);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,scene.status.velocity_x_mps);
  EXPECT_LE(path.points.back().velocity,30./3.6+1e-6);
}
TEST(CorePathPlanner, DistantStaticBlockerDoesNotForceRestAtThePredictionHorizon) {
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  PedestrianCautionScene scene(80,0);scene.objects.objects[0].label="obstacle";
  scene.objects.objects[0].bbox_size.x=scene.objects.objects[0].bbox_size.y=1.5;
  scene.ego.twist.twist.linear.x=12;scene.status.velocity_x_mps=scene.status.signed_velocity_mps=12;
  path_planner::CorePathPlanner core(parameters);const auto path=scene.run(core);
  ASSERT_GE(path.points.size(),2u);EXPECT_DOUBLE_EQ(path.points.front().velocity,12);
  const auto two=std::lower_bound(path.points.begin(),path.points.end(),2.,
      [](const interfaces::PathPoint& q,double t){return q.time_from_start.toSec()<t;});
  ASSERT_NE(two,path.points.end());EXPECT_GE(two->velocity,11.8);
  EXPECT_GT(path.points.back().velocity,0);
  EXPECT_NEAR(path.points.back().time_from_start.toSec(),5,1e-8);
  for(const auto& point:path.points) EXPECT_LT(point.pose.pose.position.x+3.845008,79.0);
}

TEST(CorePathPlanner, SlowingDoesNotReleaseTheSamePedestrianApproachBeforePassing) {
  // 속도가 내려가 계산 거리가 짧아져도 이미 접근 중인 동일 관측을 놓쳐
  // 다시 가속하면 안 된다. 객체가 실제로 멀어져 기존 접근 범위를 벗어나면 해제한다.
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  auto parameters=path_planner::loadCorePathPlannerParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  parameters.max_speed_mps=12;
  PedestrianCautionScene scene(30,3);scene.ego.twist.twist.linear.x=12;
  scene.status.velocity_x_mps=scene.status.signed_velocity_mps=12;
  path_planner::CorePathPlanner core(parameters);auto path=scene.run(core);
  ASSERT_GE(path.points.size(),2u);EXPECT_LE(path.points.back().velocity,30./3.6+1e-6);
  scene.ego.twist.twist.linear.x=6;scene.status.velocity_x_mps=scene.status.signed_velocity_mps=6;
  auto& object=scene.objects.objects.front();object.position.x=28;object.bbox_pose.position=object.position;
  path=scene.run(core);ASSERT_GE(path.points.size(),2u);
  EXPECT_LE(path.points.back().velocity,30./3.6+1e-6);
  object.position.x=55;object.bbox_pose.position=object.position;
  path=scene.run(core);ASSERT_GE(path.points.size(),2u);EXPECT_GT(path.points.back().velocity,30./3.6+1e-6);
}
