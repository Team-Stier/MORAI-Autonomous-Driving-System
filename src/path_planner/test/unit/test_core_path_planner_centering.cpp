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
TEST(CorePathCentering, TightCurveKeepsTheForwardGeometryBeyondTheBrakingPrefix) {
  // R74 S자 구간의 관측 상태다. 25/50m C2 조각이 곡선을 따라가지 못하면
  // 짧은 기하 창 끝 때문에 2초 뒤 약 2.95m/s로 감속한다. 같은 연결기로
  // 긴 곡선이 검증되면 가상의 공간 끝을 위한 감속을 반복하지 않아야 한다.
  // 객체를 제외하여 기하 창만 재현하며 실제 지도·차체·0.05m 도로 여유와
  // 시간별 곡률·횡가속도·가감속·jerk 검사는 그대로 검사한다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  // #FIXTURE_CRUISE_CONTRACT: R74 기하 창 회귀의 원본 72km/h·100m 조건을 고정한다.
  // 실제 운용 YAML의 규정 속도 변경이 이 계산 회귀의 조건을 바꾸지 않게 한다.
  // 일반 60km/h·고주로 예외/종료 연결은 native mission speed 시험에서 검사한다.
  parameters.max_speed_mps=20;
  parameters.trajectory_horizon_m=100;
  parameters.normal_speed_limit_mps=0;
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
  ego.header.stamp=ros::Time(1792346069,402929279);
  ego.pose.pose.position.x=-128.85057285425788;
  ego.pose.pose.position.y=-325.0347262303798;
  auto& q=ego.pose.pose.orientation;
  q.x=-.0034704245954729255;q.y=-.00011249434197791548;
  q.z=.7760105097287147;q.w=.6307104187253696;
  ego.twist.twist.linear.x=4.650110985315921;
  ego.twist.twist.linear.y=.007550997357022694;
  ego.twist.twist.angular.z=-.3504519760608673;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;
  status.velocity_x_mps=4.657820701599121;status.signed_velocity_mps=4.657819747924805;
  status.accel=.07716958224773407;
  sensor_msgs::Imu imu;imu.header=ego.header;
  imu.linear_acceleration.x=-.206875741481781;
  imu.linear_acceleration.y=-1.613662838935852;
  imu.linear_acceleration.z=9.774835586547852;
  interfaces::Objects objects;objects.header.frame_id="map";
  objects.header.stamp=ros::Time(1792346069,397505000);
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  path_planner::GlobalPath global(shared);
  const path_planner::Checkpoints remaining(checkpoints.begin()+2,checkpoints.end());
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  PathGenerationCore core(parameters,shared);
  ros::Time stamp;stamp.fromSec(1792346069.4218667);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,stamp);
  ASSERT_GE(path.points.size(),2u);
  const auto after_two=std::lower_bound(path.points.begin(),path.points.end(),2.,
      [](const interfaces::PathPoint& point,double t){return point.time_from_start.toSec()<t;});
  ASSERT_NE(after_two,path.points.end());
  EXPECT_GT(after_two->velocity,4.5);
  EXPECT_GT(core.previous_path_.back().progress_m,20.0);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EgoState initial;initial.pose=core.previous_path_.front().pose;
  initial.stamp_sec=stamp.toSec();initial.speed_mps=status.velocity_x_mps;
  initial.curvature_inv_m=ego.twist.twist.angular.z/initial.speed_mps;
  initial.acceleration_mps2=imu.linear_acceleration.x-
      (Eigen::Quaterniond(q.w,q.x,q.y,q.z).conjugate()*Eigen::Vector3d(0,0,9.81)).x();
  Predictions scene;scene.reference_stamp_sec=objects.header.stamp.toSec();
  EXPECT_TRUE(core.checkPlannedPath(initial,core.road_geometry_->input,scene,
                                   PlanningConstraints{},core.previous_path_));
  const auto region=core.nativeRoadRegion(core.rearFootprintRoute(route));
  ASSERT_TRUE(static_cast<bool>(region));
  for(const auto& point:core.previous_path_)
    EXPECT_TRUE(region->containsBox(geometry::sweptEgoEnvelope(
        point.pose,point.pose,parameters.vehicle,parameters.road_margin_m)));
}

TEST(CorePathCentering, BrakingIntoARealCurveDoesNotMultiplyUnalignedLateralExtrema) {
  // R39 6구간 실측: 감속하면서 곡률이 증가한다. 한 구간 앞의 최대 속도와
  // 뒤의 최대 곡률을 곱하면 안전한 시간 계획까지 거부해 Watchdog이 작동했다.
  // 원래 map·차체·2m/s² 한도로 현재 상태에서 전진 경로를 계산해야 한다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  nav_msgs::Odometry ego;ego.header.frame_id="map";
  ego.header.stamp=ros::Time(1792125888,638088632);ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-58.46786460817488;ego.pose.pose.position.y=88.81910787264961;
  auto& q=ego.pose.pose.orientation;
  q.x=-.005869429591239738;q.y=-.000439657524967549;
  q.z=.5196130745639082;q.w=.8543814190628074;
  ego.twist.twist.linear.x=6.0296901345933795;
  ego.twist.twist.linear.y=.03209767818701358;
  ego.twist.twist.angular.z=-.3045022785663605;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;
  status.velocity_x_mps=5.996948719024658;
  status.accel=.09347914904356003;
  sensor_msgs::Imu imu;imu.header=ego.header;
  imu.linear_acceleration.x=-1.9227986335754395;
  imu.linear_acceleration.y=-2.077871322631836;
  imu.linear_acceleration.z=9.736671447753906;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  path_planner::GlobalPath global(shared);
  const path_planner::Checkpoints checkpoints(loaded.begin()+6,loaded.end());
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePathPlanner core(parameters,shared);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,
                               ros::Time(1792125888,660216121));
  ASSERT_GE(path.points.size(),2u);
  EXPECT_GT(path.points.back().time_from_start.toSec(),1.0);
  EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-path.points.front().pose.pose.position.x,
                       path.points.back().pose.pose.position.y-path.points.front().pose.pose.position.y),1.0);
}

TEST(CorePathLateralAcceleration, RejectsAnInteriorSpeedPeakEvenWhenBothEndpointsAreSafe) {
  // 반지름 20m 원호, v(t)=6+2t-2t²: 양 끝은 1.8m/s²이지만
  // t=0.5의 6.5m/s는 2.1125m/s²다. 구간 분할이 실제 위반을 숨기면 안 된다.
  std::vector<PathPoint> points;
  for(int i=0;i<=80;++i) {
    const double s=.25*i,angle=s/20;
    points.push_back({{{20*std::sin(angle),20*(1-std::cos(angle))},angle},s,s,.05});
  }
  const PathGenerationCore::ArcPath arc(points);
  const PathGenerationCore::SpeedEdge edge{0,6,2,6,-2,0,1};
  EXPECT_FALSE(PathGenerationCore{}.lateralAccelerationClear(arc,edge));
}

TEST(CorePathQp, CoupledObjectiveKeepsTheConstrainedOptimumWhenActiveBoundsChange) {
  // min 1/2 x'Hx+q'x, 0<=x<=1, 0<=y<=10의 해는 (1,1.5)다.
  // x 상한의 multiplier=4.5이며, y는 자유 변수다. 처음 고른 활성
  // 경계가 틀려도 같은 해에 도달하고 원래 부등식/KKT 검사를 만족해야 한다.
  Eigen::MatrixXd h(2,2); h<<4,1,1,2;
  Eigen::VectorXd q(2); q<<-10,-4;
  const Eigen::MatrixXd a=Eigen::MatrixXd::Identity(2,2);
  Eigen::VectorXd lower(2),upper(2); lower<<0,0; upper<<1,10;
  const PathGenerationCore core;
  for(bool initially_at_lower : {false,true}) {
    Eigen::VectorXd z(2); z<<1,1.5;
    if(initially_at_lower) z.setZero();
    Eigen::VectorXd x=Eigen::VectorXd::Zero(2);
    ASSERT_TRUE(core.refineActiveSet(h,q,a,lower,upper,z,x));
    EXPECT_NEAR(x[0],1,1e-10);
    EXPECT_NEAR(x[1],1.5,1e-10);
  }
}

TEST(CorePathQp, DependentActiveEqualitiesDoNotPublishAnUncertifiedSolution) {
  const Eigen::MatrixXd h=Eigen::MatrixXd::Identity(2,2);
  const Eigen::VectorXd q=Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd a(2,2); a<<1,0,2,0;
  Eigen::VectorXd bound(2); bound<<1,2;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(2);
  const PathGenerationCore core;
  EXPECT_FALSE(core.refineActiveSet(h,q,a,bound,bound,bound,x));
  EXPECT_EQ(x.squaredNorm(),0);
}

TEST(CorePathCentering, AvoidsUnnecessaryHardBrakingBeforeALaneChange) {
  // 실측 12구간 진입 직선: 원래 75m 후보는 2초 뒤 7.4m/s까지 감속하지만
  // 같은 도로에서 검증된 50m 후보는 13m/s로 진행한다. 먼 객체를 제외해
  // 첫 유효 중앙 후보를 즉시 선택하는 기하/시간 비용 문제만 재현한다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(10);
  ego.child_frame_id="base_link";
  ego.pose.pose.position.x=67.57710692353058;
  ego.pose.pose.position.y=-68.02610326316237;
  auto& q=ego.pose.pose.orientation;
  q.x=-.006221928502643271;q.y=.002038126464496298;
  q.z=.705414258414694;q.w=-.7087650228894412;
  ego.twist.twist.linear.x=15.019328075755691;
  ego.twist.twist.linear.y=-.022434403738229847;
  ego.twist.twist.angular.z=-.0026179063133895397;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;
  status.velocity_x_mps=15.015807151794434;
  sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=-.7640783190727234;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  path_planner::GlobalPath global(shared);
  const path_planner::Checkpoints checkpoints(loaded.begin()+12,loaded.end());
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  path_planner::CorePathPlanner core(parameters,shared);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  const auto two_seconds=std::lower_bound(path.points.begin(),path.points.end(),2.,
      [](const interfaces::PathPoint& point,double time){return point.time_from_start.toSec()<time;});
  ASSERT_NE(two_seconds,path.points.end());
  EXPECT_GT(two_seconds->velocity,11.0);
  EXPECT_GT(core.previous_path_.back().progress_m,45.0);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EgoState initial{{{ego.pose.pose.position.x,ego.pose.pose.position.y},
      std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))},10};
  initial.speed_mps=status.velocity_x_mps;
  initial.curvature_inv_m=ego.twist.twist.angular.z/initial.speed_mps;
  initial.acceleration_mps2=imu.linear_acceleration.x-
      (Eigen::Quaterniond(q.w,q.x,q.y,q.z).conjugate()*Eigen::Vector3d(0,0,9.81)).x();
  Predictions scene;scene.reference_stamp_sec=10;
  EXPECT_TRUE(core.checkPlannedPath(initial,core.road_geometry_->input,scene,
                                   PlanningConstraints{},core.previous_path_));
}

TEST(CorePathCentering, ChecksSmoothGeometryWhenNoisyLaneShapeHasNoFeasibleSpeed) {
  // 충돌 이전 두 실측 입력이다. 전방의 먼 객체를 제거해 지도·시간 계획만
  // 재현한다. 허용 도로와 초기 운동을 유지하면서 전진 가능한 후보가 있어야 한다.
  struct Fixture {
    double x,y,qx,qy,qz,qw,vx,vy,yaw_rate,speed,ax;
    int section;
  };
  const Fixture fixtures[]{
    {-98.58806368050867,-369.6130567874727,
     -.004352586915908048,-.003487733824971155,-.5009642899846843,-.8654499817205191,
     7.10801647565341,.29210223536397933,.10387687385082245,
     7.352736949920654,-.06724134087562561,2},
    {-107.76154769136677,-280.605950819436,
     .00022577229085690188,-.006530599628932612,-.5063377214148662,-.8623105079771212,
     6.079037264477745,.3878783022232586,.019434068351984024,
     6.236171722412109,-.6935005784034729,3}
  };
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  for(const auto& f:fixtures) {
    SCOPED_TRACE(f.section);
    nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(10);
    ego.child_frame_id="base_link";
    ego.pose.pose.position.x=f.x;ego.pose.pose.position.y=f.y;
    auto& q=ego.pose.pose.orientation;q.x=f.qx;q.y=f.qy;q.z=f.qz;q.w=f.qw;
    ego.twist.twist.linear.x=f.vx;ego.twist.twist.linear.y=f.vy;
    ego.twist.twist.angular.z=f.yaw_rate;
    interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.velocity_x_mps=f.speed;
    sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=f.ax;
    interfaces::Objects objects;objects.header=ego.header;
    path_planner::PredictedObjects predictions;predictions.fill(objects);
    path_planner::GlobalPath global(shared);
    const path_planner::Checkpoints checkpoints(loaded.begin()+f.section,loaded.end());
    const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
    ASSERT_TRUE(static_cast<bool>(route));
    path_planner::CorePathPlanner core(parameters,shared);
    const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
    ASSERT_GE(path.points.size(),2u);
    EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
    EXPECT_NEAR(path.points.front().pose.pose.position.x,f.x,1e-8);
    EXPECT_NEAR(path.points.front().pose.pose.position.y,f.y,1e-8);
    EXPECT_GT(path.points.back().time_from_start.toSec(),4.9);
    EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-f.x,
                         path.points.back().pose.pose.position.y-f.y),3);
    EgoState initial{{{f.x,f.y},std::atan2(2*(f.qw*f.qz+f.qx*f.qy),
                                         1-2*(f.qy*f.qy+f.qz*f.qz))},10};
    initial.speed_mps=f.speed;initial.curvature_inv_m=f.yaw_rate/f.speed;
    const Eigen::Quaterniond orientation(f.qw,f.qx,f.qy,f.qz);
    initial.acceleration_mps2=f.ax-(orientation.conjugate()*Eigen::Vector3d(0,0,9.81)).x();
    Predictions scene;scene.reference_stamp_sec=10;
    EXPECT_TRUE(core.checkPlannedPath(initial,core.road_geometry_->input,scene,
                                     PlanningConstraints{},core.previous_path_));
  }
}

TEST(CorePathCentering, KeepsAFeasibleBrakingWindowBeforeNativeLaneChange) {
  // 12구간의 실제 빈 경로 입력이다. 먼 객체를 제외해 지도·창 탐색을
  // 분리한다. 측정 초기 상태와 전체 안전 검사를 유지한 전진 경로를 확인한다.
  struct Fixture {
    double x,y,qx,qy,qz,qw,vx,vy,yaw_rate,speed,ax;
    int section;
    double prediction_offset_sec=0;
  };
  const Fixture fixtures[]{
    {67.69685204753364,-106.76000251094372,-0.00598987231601158,0.0008743797732639266,0.7062998762800949,-0.7078868847890689,14.290490091702285,-0.21479330505921498,-0.00988618191331625,14.289347648620605,-1.167344331741333,12},
    {67.75382816647239,-111.01970536876755,-0.008055636385624652,0.0011965275062943066,0.7053679073292627,-0.7087946037846832,13.899310565565452,-0.0759626001822353,-0.00870203971862793,13.899821281433105,-1.527190089225769,12},
    // R26의 기존 유효 제동 경로: 연속 차선 변경 연결 수정 뒤에도 보존한다.
    {67.77158864082907,-118.08695253865905,.0029021956774281077,.0030855908325633646,-.710925568309807,.7032545006701643,12.168428574435325,.007353249839278616,-.14190725982189178,12.188450813293457,-2.7208735942840576,12,.008561849594116211}
  };
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  for(const auto& f:fixtures) {
    SCOPED_TRACE(f.section);
    nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(10);
    ego.child_frame_id="base_link";
    ego.pose.pose.position.x=f.x;ego.pose.pose.position.y=f.y;
    auto& q=ego.pose.pose.orientation;q.x=f.qx;q.y=f.qy;q.z=f.qz;q.w=f.qw;
    ego.twist.twist.linear.x=f.vx;ego.twist.twist.linear.y=f.vy;
    ego.twist.twist.angular.z=f.yaw_rate;
    interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.velocity_x_mps=f.speed;
    sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=f.ax;
    interfaces::Objects objects;objects.header=ego.header;
    objects.header.stamp-=ros::Duration(f.prediction_offset_sec);
    path_planner::PredictedObjects predictions;predictions.fill(objects);
    path_planner::GlobalPath global(shared);
    const path_planner::Checkpoints checkpoints(loaded.begin()+f.section,loaded.end());
    const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
    ASSERT_TRUE(static_cast<bool>(route));
    path_planner::CorePathPlanner core(parameters,shared);
    const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
    ASSERT_GE(path.points.size(),2u);
    EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
    EXPECT_NEAR(path.points.front().pose.pose.position.x,f.x,1e-8);
    EXPECT_NEAR(path.points.front().pose.pose.position.y,f.y,1e-8);
    EXPECT_GT(path.points.back().time_from_start.toSec(),4.9);
    EXPECT_GT(std::hypot(path.points.back().pose.pose.position.x-f.x,
                         path.points.back().pose.pose.position.y-f.y),3);
    EgoState initial{{{f.x,f.y},std::atan2(2*(f.qw*f.qz+f.qx*f.qy),
                                         1-2*(f.qy*f.qy+f.qz*f.qz))},10};
    initial.speed_mps=f.speed;initial.curvature_inv_m=f.yaw_rate/f.speed;
    const Eigen::Quaterniond orientation(f.qw,f.qx,f.qy,f.qz);
    initial.acceleration_mps2=f.ax-(orientation.conjugate()*Eigen::Vector3d(0,0,9.81)).x();
    Predictions scene;scene.reference_stamp_sec=objects.header.stamp.toSec();
    EXPECT_TRUE(core.checkPlannedPath(initial,core.road_geometry_->input,scene,
                                     PlanningConstraints{},core.previous_path_));
  }
}


TEST(CorePathCentering, DoesNotPlanAnArtificialStopAtAShortWindowOnAnOpenStraight) {
  // 실측 직선 세 지점. 현재 6.2~6.7m/s에서 5초 안에 8m/s 이상으로
  // 진행할 도로가 있는데, 25m 계산 창만 택하면 매번 가상의 정지를 계획한다.
  struct Fixture { double x,y,qx,qy,qz,qw,vx,vy,yaw_rate,speed,ax; };
  const Fixture fixtures[]{
    {-59.58989571641752,-125.97293105673282,-.0006176140652380956,.0006144125164382362,
      .7330612313819156,.6801620925154207,6.193809914310645,.09377249892269718,
      -.028012247756123543,6.191277027130127,.5039507150650024},
    {-59.87012021944071,-75.70906821272345,-.00018173151427478928,-.0019574116428687565,
      -.7066087394388463,-.7076017417035955,6.675904614743462,-.10854264720158258,
      .028270572423934937,6.678629398345947,.13209278881549835},
    {-59.958505465051815,-69.42642089802682,-.0018529591147623736,.0003375537479355606,
      .710705384737407,.703487248431157,6.6767522952291625,.05999687734764793,
      -.04283970966935158,6.680270671844482,.06742474436759949}
  };
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  for(const auto& f:fixtures) {
    SCOPED_TRACE(f.y);
    nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(10);
    ego.child_frame_id="base_link";
    ego.pose.pose.position.x=f.x;ego.pose.pose.position.y=f.y;
    auto& q=ego.pose.pose.orientation;q.x=f.qx;q.y=f.qy;q.z=f.qz;q.w=f.qw;
    ego.twist.twist.linear.x=f.vx;ego.twist.twist.linear.y=f.vy;
    ego.twist.twist.angular.z=f.yaw_rate;
    interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.velocity_x_mps=f.speed;
    sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=f.ax;
    interfaces::Objects objects;objects.header=ego.header;
    path_planner::PredictedObjects predictions;predictions.fill(objects);
    path_planner::GlobalPath global(shared);
    const path_planner::Checkpoints checkpoints(loaded.begin()+5,loaded.end());
    const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
    ASSERT_TRUE(static_cast<bool>(route));
    path_planner::CorePathPlanner core(parameters,shared);
    const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
    ASSERT_GE(path.points.size(),2u);
    EXPECT_GT(path.points.back().velocity,8.0);
    EXPECT_DOUBLE_EQ(path.points.front().velocity,f.speed);
    EXPECT_NEAR(path.points.front().pose.pose.position.x,f.x,1e-8);
    EXPECT_NEAR(path.points.front().pose.pose.position.y,f.y,1e-8);
    EXPECT_GT(path.points.back().time_from_start.toSec(),4.9);
    EgoState initial{{{f.x,f.y},std::atan2(2*(f.qw*f.qz+f.qx*f.qy),
                                         1-2*(f.qy*f.qy+f.qz*f.qz))},10};
    initial.speed_mps=f.speed;initial.curvature_inv_m=f.yaw_rate/f.speed;
    const Eigen::Quaterniond orientation(f.qw,f.qx,f.qy,f.qz);
    initial.acceleration_mps2=f.ax-(orientation.conjugate()*Eigen::Vector3d(0,0,9.81)).x();
    Predictions scene;scene.reference_stamp_sec=10;
    EXPECT_TRUE(core.checkPlannedPath(initial,core.road_geometry_->input,scene,
                                     PlanningConstraints{},core.previous_path_));
  }
}

TEST(CorePathSpeedLimits, LocalTimePartitionStillRejectsExcessAndInteriorPeakLateralAcceleration) {
  const PathGenerationCore core;
  for(double sign:{-1.0,1.0}) {
    std::vector<PathPoint> points;
    for(int i=0;i<=80;++i) {
      const double s=i*.25,angle=s/20;
      points.push_back({{{20*std::sin(angle),sign*20*(1-std::cos(angle))},sign*angle},
                        s,0,sign/20});
    }
    const PathGenerationCore::ArcPath arc(points);
    // R=20m: 6m/s -> 1.8m/s², 7m/s -> 2.45m/s².
    EXPECT_TRUE(core.lateralAccelerationClear(arc,{0,6,0,6,0,0,.5}));
    EXPECT_FALSE(core.lateralAccelerationClear(arc,{0,7,0,7,0,0,.5}));
    // 양 끝 5m/s라도 v(t)=5+16t-32t²의 중간 7m/s는 거부한다.
    EXPECT_FALSE(core.lateralAccelerationClear(arc,{0,5,16,5,-16,0,.5}));
  }
}

TEST(CorePathNativeUnion, RejectsConcaveNotchesHolesAndDisconnectedGapsForWholeBody) {
  PathGenerationCore::NativeRoadRegion region;
  PathGenerationCore::RoadPolygon polygon;
  for(const auto& p:std::vector<Vec2>{{-10,-10},{10,-10},{10,10},{1,10},{1,2},{-1,2},{-1,10},{-10,10}})
    polygon.outer().emplace_back(p.x,p.y);
  polygon.inners().resize(1);
  for(const auto& p:std::vector<Vec2>{{-1,-1},{1,-1},{1,1},{-1,1}})
    polygon.inners().front().emplace_back(p.x,p.y);
  boost::geometry::correct(polygon);region.polygons.push_back(polygon);region.prepare();
  EXPECT_TRUE(region.containsBox({{{-5,0},.3},1,1}));
  EXPECT_FALSE(region.containsBox({{{0,0},0},2,2})); // 네 꼭짓점만 보면 내부의 구멍을 놓친다.
  EXPECT_FALSE(region.containsBox({{{0,3},0},3,2})); // Full body spans an inward notch.
  PathGenerationCore::RoadPolygon separate;
  for(const auto& p:std::vector<Vec2>{{20,-2},{25,-2},{25,2},{20,2}})
    separate.outer().emplace_back(p.x,p.y);
  boost::geometry::correct(separate);region.polygons.push_back(separate);region.prepare();
  EXPECT_TRUE(region.containsBox({{{22,0},0},1,1}));
  EXPECT_FALSE(region.containsBox({{{15,0},0},6,1}));
  // 이산화·BVH 구현과 독립인 polygon 포함 연산과 비교한다. 경계에 정확히
  // 닿는 경우의 보수적 거부는 위 기존 경계 테스트가 별도로 검사한다.
  for(double x=-12.17;x<28;x+=.731)for(double y=-12.13;y<12;y+=.913)
    for(double yaw:{0.,.3,1.2}) {
      const geometry::Box box{{{x,y},yaw},2.3175,1.0804};
      PathGenerationCore::RoadPolygon footprint;
      for(const auto& corner:geometry::corners(box))footprint.outer().emplace_back(corner.x,corner.y);
      boost::geometry::correct(footprint);
      EXPECT_EQ(boost::geometry::covered_by(footprint,region.polygons),region.containsBox(box));
    }
}

TEST(CorePathBoundaryIndex, KeepsWholeFootprintAndClosedContactChecks) {
  const std::vector<Vec2> rectangle{{-10,-10},{10,-10},{10,10},{-10,10}};
  const geometry::BoundaryIndex index(rectangle);
  EXPECT_TRUE(index.containsBox({{{0,0},.4},2,1}));
  EXPECT_FALSE(index.containsBox({{{8,0},0},2,1}));
  EXPECT_FALSE(index.containsBox({{{11,0},0},2,1}));
  const std::vector<Vec2> notch{{-10,-10},{10,-10},{10,10},{1,10},
                               {1,-1},{-1,-1},{-1,10},{-10,10}};
  const geometry::BoundaryIndex notched(notch);
  EXPECT_TRUE(notched.containsBox({{{0,-5},0},3,2}));
  // 네 모서리는 도로 안에 있지만 차체 중간을 침범하는 오목 경계는 거부한다.
  EXPECT_FALSE(notched.containsBox({{{0,-2},0},3,2}));
}

TEST(CorePathBoundaryIndex, MatchesOriginalChecksAcrossRotationsAndLargeMapCoordinates) {
  for(double offset : {0.0,1000000.0}) {
    std::vector<Vec2> polygon;
    for(int i=0;i<48;++i) {
      const double angle=i*2*std::acos(-1.0)/48;
      polygon.push_back({offset+10*std::cos(angle),offset+10*std::sin(angle)});
    }
    const geometry::BoundaryIndex index(polygon);
    for(int x=-12;x<=12;x+=2) for(int y=-12;y<=12;y+=2)
      for(double yaw : {0.0,.17,.78,1.57}) {
        const geometry::Box box{{{offset+x,offset+y},yaw},2.317504,1.0804175};
        EXPECT_EQ(geometry::polygonContainsBox(polygon,box),index.containsBox(box));
      }
  }
}

TEST(CorePathCentering, RealKCityStandstillWithEmptyGroundTruthProducesVerifiedDeparture) {
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin(),loaded.end());
  path_planner::GlobalPath fixture_route(shared);
  nav_msgs::Odometry ego;
  ego.header.frame_id="map"; ego.header.stamp.fromSec(10); ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-131.636156; ego.pose.pose.position.y=-428.276841;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.0030573728238533736; q.y=-0.003071550805597548;
  q.z=-0.5193928283193193; q.w=-0.8545246093223545;
  ego.twist.twist.angular.z=5e-7;
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x=-0.02042577788233757;
  imu.linear_acceleration.y=0.08219410479068756;
  imu.linear_acceleration.z=9.809725761413574;
  interfaces::Objects objects; objects.header=ego.header;
  path_planner::PredictedObjects predictions; predictions.fill(objects);
  const auto route=std::get<5>(fixture_route.process(ego,{},objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  for(double speed:{-5e-6,0.0,5e-6}) {
    SCOPED_TRACE(speed);
    ego.twist.twist.linear.x=speed;
    path_planner::CorePathPlanner core(parameters);
    const auto path=core.process(ego,{},objects,{},imu,route,predictions,ego.header.stamp);
    ASSERT_GE(path.points.size(),2u);
    EXPECT_DOUBLE_EQ(path.points.front().velocity,0.0);
    EXPECT_DOUBLE_EQ(path.points.front().pose.pose.position.x,ego.pose.pose.position.x);
    EXPECT_DOUBLE_EQ(path.points.front().pose.pose.position.y,ego.pose.pose.position.y);
    bool moves=false;
    for(const auto& point:path.points) {
      EXPECT_GE(point.velocity,0.0); EXPECT_LE(point.velocity,parameters.max_speed_mps);
      EXPECT_GE(point.acceleration,-parameters.max_deceleration_mps2);
      EXPECT_LE(point.acceleration,parameters.max_acceleration_mps2);
      moves=moves || point.velocity>.1;
    }
    EXPECT_TRUE(moves);
  }
  // 실제 UDP는 IMU/ego보다 먼저 도착할 수 있다. Pipeline 허용 skew 안의 입력이다.
  objects.header.stamp.fromSec(10.05);
  predictions.fill(objects);
  ego.twist.twist.linear.x=0;
  path_planner::CorePathPlanner asynchronous_core(parameters);
  EXPECT_GE(asynchronous_core.process(ego,{},objects,{},imu,route,predictions,objects.header.stamp).points.size(),2u);
}

TEST(CorePathCentering, RealKCitySectionTwoReferenceWiggleDoesNotDeadlockAtStandstill) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+2,loaded.end());
  path_planner::GlobalPath fixture_route(shared);
  nav_msgs::Odometry ego;
  ego.header.frame_id="map"; ego.header.stamp.fromSec(10); ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-98.41966904482501;
  ego.pose.pose.position.y=-368.5511517061303;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.0021820764160697366; q.y=-0.001651233452616814;
  q.z=-0.4710157734853088; q.w=-0.8821205434058519;
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x=-0.008482811041176319;
  imu.linear_acceleration.y=0.05308746546506882;
  imu.linear_acceleration.z=9.810044288635254;
  interfaces::CompetitionVehicleStatus competition;
  competition.header=ego.header; competition.brake=1;
  interfaces::Objects objects; objects.header=ego.header;
  path_planner::PredictedObjects predictions; predictions.fill(objects);
  const auto route=std::get<5>(fixture_route.process(ego,competition,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  path_planner::CorePathPlanner core(parameters);
  const auto path=core.process(ego,competition,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().pose.pose.position.x,ego.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(path.points.front().pose.pose.position.y,ego.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,0);
  bool moves=false;
  for(const auto& point:path.points) {
    EXPECT_GE(point.velocity,0);
    EXPECT_LE(point.velocity,parameters.max_speed_mps);
    moves=moves || point.velocity>.1;
  }
  EXPECT_TRUE(moves);
  EgoState initial;
  initial.pose={{ego.pose.pose.position.x,ego.pose.pose.position.y},
      std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
  initial.stamp_sec=10;
  Predictions native_predictions; native_predictions.reference_stamp_sec=10;
  ASSERT_TRUE(core.road_geometry_);
  EXPECT_TRUE(core.checkPlannedPath(initial,core.road_geometry_->input,
                                  native_predictions,PlanningConstraints{},core.previous_path_));
}

TEST(CorePathCentering, NativeStaticObstacleWaitDoesNotSuppressOtherVerifiedMotionCandidates) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+4,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
  ego.header.stamp.fromSec(10);
  ego.pose.pose.position.x=-59.92859471883335;
  ego.pose.pose.position.y=-160.00050401509228;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.0002932542141176604;q.y=-0.0016408800023292006;
  q.z=-0.686765216962155;q.w=-0.7268774025141435;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.brake=1;
  sensor_msgs::Imu imu;imu.linear_acceleration.x=-0.019403161481022835;
  imu.linear_acceleration.y=0.026151755824685097;imu.linear_acceleration.z=9.810171127319336;
  interfaces::Objects objects;objects.header=ego.header;
  interfaces::Object obstacle;obstacle.id=1;obstacle.label="obstacle";
  obstacle.position.x=-60.61805725097656;obstacle.position.y=-142.192138671875;
  obstacle.bbox_pose.position=obstacle.position;obstacle.bbox_pose.orientation.w=1;
  obstacle.bbox_size.x=.75;obstacle.bbox_size.y=.75;obstacle.bbox_size.z=.75;
  obstacle.velocity_available=true;obstacle.acceleration_available=true;
  objects.objects.push_back(obstacle);
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  path_planner::CorePathPlanner core(parameters,shared);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  // 실제 정지 입력이다. 첫 창의 제자리 대기가 안전해도 나머지 창·횡 후보의
  // 검증된 전진을 숨겨서는 안 된다. 도로/충돌 검사를 없애 출발시키지 않는다.
  EXPECT_GT(path.points.back().pose.pose.position.y,ego.pose.pose.position.y+1);
  EXPECT_GT(std::max_element(path.points.begin(),path.points.end(),
      [](const interfaces::PathPoint& a,const interfaces::PathPoint& b){return a.velocity<b.velocity;})->velocity,.1);
}

TEST(CorePathCentering, StaticObstacleCreepDoesNotSuppressImmediateVerifiedMotion) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+4,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
  ego.header.stamp.fromSec(10);
  ego.pose.pose.position.x=-60.15469140694538;
  ego.pose.pose.position.y=-163.32559935778954;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.00035297370614659066;q.y=-0.0015498781235506846;
  q.z=-0.6903106547462127;q.w=-0.723511349759781;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.brake=1;
  sensor_msgs::Imu imu;imu.linear_acceleration.x=-0.017159465700387955;
  imu.linear_acceleration.y=0.026151755824685097;imu.linear_acceleration.z=9.810171127319336;
  interfaces::Objects objects;objects.header=ego.header;
  interfaces::Object obstacle;obstacle.id=1;obstacle.label="obstacle";
  obstacle.position.x=-60.607887268066406;obstacle.position.y=-142.17800903320312;
  obstacle.bbox_pose.position=obstacle.position;obstacle.bbox_pose.orientation.z=-0.6640495842253258;
  obstacle.bbox_pose.orientation.w=0.747686522616934;
  obstacle.bbox_size.x=1.497259259223938;obstacle.bbox_size.y=1.497259259223938;
  obstacle.bbox_size.z=1.3032472133636475;
  obstacle.velocity_available=true;obstacle.acceleration_available=true;
  objects.objects.push_back(obstacle);
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  path_planner::CorePathPlanner core(parameters,shared);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_FALSE(path.points.empty());
  // R51 실측: 매번 1초 대기 후 1m 이동/정지를 예약하면 재계획으로 출발이
  // 계속 뒤로 밀린다. 같은 입력에서 즉시 전진 가능한 검증된 창이 있어야 한다.
  const auto one_second=std::lower_bound(path.points.begin(),path.points.end(),1.,
      [](const interfaces::PathPoint& point,double t){return point.time_from_start.toSec()<t;});
  ASSERT_NE(one_second,path.points.end());
  EXPECT_GT(one_second->velocity,.1);
  EXPECT_GT(path.points.back().velocity,0);
  EXPECT_GT(core.previous_path_.back().progress_m,5);
}

std::vector<ReferencePoint> circle(double radius,double turn,double angle_step=.025) {
  std::vector<ReferencePoint> points;
  for(int i=-20;i<=140;++i) {
    const double angle=i*angle_step;
    points.push_back({{radius*std::sin(angle),turn*radius*(1-std::cos(angle))},-1.75,1.75});
  }
  return points;
}
Predictions emptyScene() { Predictions predictions; predictions.reference_stamp_sec=10; return predictions; }
TEST(CorePathSpeedReserve, PreferredCurveSpeedLeavesRoomBelowTheUnchangedHardLimit) {
  // 선호 횡가속도는 비용 목표다. 이미 그 목표보다 빠른 실측 초기 상태를
  // 거부하지 않고 기존 2m/s² 한도 안에서 연속적으로 감속해야 한다.
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromString("preferred_lateral_accel_mps2: 1.2"));
  const PathGenerationCore core(parameters);
  const auto road=circle(20,1);
  EgoState ego{{{0,0},0},10};ego.speed_mps=5.5;ego.curvature_inv_m=.05;
  const auto scene=emptyScene();
  const auto path=core.plan(ego,road,scene,PlanningConstraints{});
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
  EXPECT_LT(path.back().speed_mps,5.2);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,PlanningConstraints{},path));
}

TEST(CorePathSpeedReserve, CurveTargetDoesNotRewardWaitingBeforeAFasterRoadSection) {
  // R54 9구간 실측이다. 위치별 목표속도 오차의 기준값 때문에 낮은 목표
  // 구간에 3초 이상 머무는 후보가 유리해지면 회피/교통과 무관하게 움찔거린다.
  ros::Time::init();const std::string package_dir=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto loaded=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
  ego.header.stamp=ros::Time(1792323778,163772002);
  ego.pose.pose.position.x=-95.56028346229975;ego.pose.pose.position.y=324.90765580812416;
  auto& q=ego.pose.pose.orientation;
  q.x=1.2640565848383517e-05;q.y=-.002619666014114675;
  q.z=-.4325190582308337;q.w=-.9016209854796545;
  ego.twist.twist.linear.x=.19438114609396287;
  ego.twist.twist.linear.y=.021440749628386144;
  ego.twist.twist.angular.z=-.016423501074314117;
  interfaces::CompetitionVehicleStatus status;status.header.frame_id="base_link";
  status.header.stamp=ros::Time(1792323778,162721610);status.velocity_x_mps=.1959434598684311;
  status.accel=.15;
  sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=.7668373584747314;
  interfaces::Objects objects;objects.header.frame_id="map";
  objects.header.stamp=ros::Time(1792323778,160482000);
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  path_planner::GlobalPath global(shared);
  const path_planner::Checkpoints checkpoints(loaded.begin()+9,loaded.end());
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  PathGenerationCore core(parameters,shared);ros::Time stamp;stamp.fromSec(1792323778.1817777);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,stamp);
  ASSERT_FALSE(path.points.empty());
  const auto two_seconds=std::lower_bound(path.points.begin(),path.points.end(),2.,
      [](const interfaces::PathPoint& point,double t){return point.time_from_start.toSec()<t;});
  ASSERT_NE(two_seconds,path.points.end());
  EXPECT_GT(two_seconds->velocity,1);
  EXPECT_GT(core.previous_path_.back().progress_m,5);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
}

TEST(CorePathSpeedReserve, PreferredCurveTargetCannotExceedTheHardLimit) {
  EXPECT_THROW(path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromString("preferred_lateral_accel_mps2: 2.1")),std::invalid_argument);
}

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
    const auto path=core.plan(ego,reference,emptyScene(),{{0,0},{25,25.0/3}},previous);
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
    const auto path=core.plan(ego,reference,emptyScene(),{{0,0},{25,12.5}});
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
  const auto path=PathGenerationCore().plan(ego,reference,emptyScene(),{{0,0},{25,12.5}});
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
    const auto path=PathGenerationCore().plan(ego,reference,emptyScene(),{{0,0},{25,25.0/3}});
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
    const auto path=core.plan(ego,reference,emptyScene(),{{0,0},{25,25.0/3}},previous);
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
    EXPECT_TRUE(core.plan(ego,reference,emptyScene(),{{0,0},{25,12.5}}).empty());
    PlanningConstraints limits; limits.maximum_speed_mps=2;
    EXPECT_TRUE(core.plan(ego,reference,emptyScene(),limits).empty());
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
    const auto path=core.plan(ego,reference,emptyScene(),limits);
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
    const auto path=core.plan(ego,reference,scene,{{0,0},{25,25.0/3}});
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
    const auto path=core.plan(ego,reference,emptyScene(),{{0,0},{25,25.0/3}});
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
  const auto path=core.plan(ego,reference,emptyScene(),{{0,0},{25,5}});
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
    const auto path=core.plan(ego,reference,scene,limits);
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.back().time_sec,5,1e-10);
    EXPECT_NEAR(path.back().speed_mps,0,1e-10);
    EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,path));
    expectInsideCircle(path,20,turn);
  }
}

namespace {
Predictions performanceScene(Vec2 position) {
  Predictions pred; pred.reference_stamp_sec=10;
  PredictedObject object; object.id=1;object.kind=ObjectKind::Static;
  object.pose.position=position;object.length_m=2;object.width_m=2;
  object.source_stamp_sec=10;
  pred.current.push_back(object);
  for(auto& future:pred.future)future=pred.current;
  return pred;
}
TEST(CorePathPerformanceSafety, ReusedRoadChangesWithWidthAndCoordinates) {
  PathGenerationCore core;
  const EgoState ego{{{0,0},0},10};const auto pred=emptyScene();
  std::vector<ReferencePoint> road{{{-10,0},-5,5},{{40,0},-5,5}};
  const std::vector<PathPoint> path{{{{0,0},0},0,0,0},{{{2,0},0},2,1,0}};
  EXPECT_TRUE(core.checkTimedPath(ego,road,pred,path));
  EXPECT_TRUE(core.checkTimedPath(ego,road,pred,path));
  auto copy=core;
  for(auto& point:road){point.min_l_m=-.5;point.max_l_m=.5;}
  EXPECT_FALSE(core.checkTimedPath(ego,road,pred,path));
  for(auto& point:road){point.min_l_m=-5;point.max_l_m=5;}
  EXPECT_TRUE(copy.checkTimedPath(ego,road,pred,path));
  for(auto& point:road)point.position.y=20;
  EXPECT_FALSE(core.checkTimedPath(ego,road,pred,path));
}
TEST(CorePathPerformanceSafety, KeepsContactCrossingRotationAndUncertainty) {
  PathGenerationCore core;
  const std::vector<ReferencePoint> road{{{-50,0},-10,10},{{50,0},-10,10}};
  const PathGenerationCore::Reference ref(road);const auto polygon=ref.boundary(.25);
  const geometry::BoundaryIndex index(polygon);const std::vector<ProgressTime> schedule;
  const PathPoint first{{{0,0},0},0,0,0},last{{{0,0},0},0,.5,0};
  for(int scenario=0;scenario<4;++scenario) {
    auto pred=performanceScene({scenario==0?5.095008:20,0});
    if(scenario==1) {
      pred.current[0].pose.position.y=-20;
      for(auto& future:pred.future)future[0].pose.position.y=20;
      pred.current[0].pose.position.x=0;
      for(auto& future:pred.future)future[0].pose.position.x=0;
    } else if(scenario==2) {
      pred.current[0].radius_error_m=25;
      for(auto& future:pred.future)future=pred.current;
    } else if(scenario==3) {
      pred.current[0].interpolation_acceleration_mps2=512;
      for(auto& future:pred.future)future=pred.current;
    }
    const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,0,false,&index};
    EXPECT_TRUE(core.collisionNarrow(ctx,first,last))<<scenario;
    EXPECT_TRUE(core.collision(ctx,first,last))<<scenario;
  }
  auto pred=performanceScene({10,0});pred.current[0].length_m=14;
  pred.current[0].pose.yaw_rad=std::acos(-1.)/2;
  for(auto& future:pred.future){future=pred.current;future[0].pose.yaw_rad=-std::acos(-1.)/2;}
  const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,0,false,&index};
  EXPECT_TRUE(core.collisionNarrow(ctx,first,last));EXPECT_TRUE(core.collision(ctx,first,last));
}
TEST(CorePathPerformanceSafety, BroadCollisionMatchesOriginalSweptSat) {
  PathGenerationCore core;
  const std::vector<ReferencePoint> road{{{-50,0},-10,10},{{50,0},-10,10}};
  const PathGenerationCore::Reference ref(road);const auto polygon=ref.boundary(.25);
  const geometry::BoundaryIndex index(polygon);const std::vector<ProgressTime> schedule;
  for(int scene=0;scene<320;++scene) {
    const double x=(scene%20-10)*5.,y=(scene/20-8)*5.;
    auto pred=performanceScene({x,y});
    pred.current[0].pose.yaw_rad=scene*.173;
    pred.current[0].radius_error_m=scene%3;
    pred.current[0].interpolation_acceleration_mps2=scene%7;
    for(std::size_t k=0;k<pred.future.size();++k) {
      pred.future[k]=pred.current;auto& o=pred.future[k][0];
      o.pose.position={x+(k+1)*(scene%5-2.),y-(k+1)*2.};o.pose.yaw_rad+=.7*(k+1);
    }
    const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,.13,false,&index};
    for(double yaw:{0.,.5,2.8}) {
      const PathPoint a{{{0,0},yaw},0,.1,0},b{{{3,2},yaw+1.2},4,1.1,0};
      for(double pad:{0.,.25,3.})
        EXPECT_EQ(core.collisionNarrow(ctx,a,b,pad),core.collision(ctx,a,b,pad))<<scene;
    }
  }
}
TEST(CorePathPerformanceSafety, BroadSpeedEdgeMatchesOriginalSampling) {
  PathGenerationCore core;
  const std::vector<ReferencePoint> road{{{-50,0},-10,10},{{50,0},-10,10}};
  const PathGenerationCore::Reference ref(road);const auto polygon=ref.boundary(.25);
  const geometry::BoundaryIndex index(polygon);const std::vector<ProgressTime> schedule;
  for(double coordinate:{0.,1000000.}) {
    const std::vector<PathPoint> shape{{{{coordinate,coordinate},0},0,0,0},
      {{{coordinate+10,coordinate+1},.1},10,1,.01},{{{coordinate+20,coordinate+3},.2},20,2,.01}};
    const PathGenerationCore::ArcPath arc(shape);
    for(int scene=0;scene<60;++scene) {
      const double x=scene<30?scene-5.:300.;auto pred=performanceScene({coordinate+x,coordinate});
      for(std::size_t k=0;k<pred.future.size();++k) {
        pred.future[k]=pred.current;
        pred.future[k][0].pose.position.y+=k*(scene%5-2.);
        pred.future[k][0].pose.yaw_rad=.8*k;
        pred.future[k][0].radius_error_m=scene%7;
      }
      const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,0,false,&index};
      const std::array<PathGenerationCore::SpeedEdge,3> edges{{
        {0,10,0,10,0,0,2},{0,0,0,5,0,0,2},{0,8,1,12,-1,0,2}}};
      for(const auto& edge:edges)
        EXPECT_EQ(core.edgeClearNarrow(ctx,arc,edge),core.edgeClear(ctx,arc,edge))<<scene;
      if(scene==15)EXPECT_FALSE(core.edgeClear(ctx,arc,edges[0]));
    }
  }
}

TEST(CorePathPerformanceSafety, TemporalSweepMatchesNarrowAtPredictionKnotsAndOffsets) {
  const PathGenerationCore core;
  for(double coordinate:{0.,302000.}) {
    const std::vector<ReferencePoint> road{{{coordinate-100,coordinate},-100,100},
                                         {{coordinate+100,coordinate},-100,100}};
    const PathGenerationCore::Reference ref(road);
    const auto polygon=ref.boundary(.25);
    const geometry::BoundaryIndex index(polygon);
    const std::vector<ProgressTime> schedule;
    const std::vector<PathPoint> shape{{{{coordinate,coordinate},0},0,0,0},
                                     {{{coordinate+40,coordinate},0},40,0,0}};
    const PathGenerationCore::ArcPath arc(shape);
    for(int scene=0;scene<40;++scene) {
      auto pred=performanceScene({coordinate-20+scene,coordinate-30});
      pred.current[0].length_m=scene%2?14:2;
      for(std::size_t k=0;k<pred.future.size();++k) {
        pred.future[k]=pred.current;
        pred.future[k][0].pose.position.x+=4*(k+1);
        pred.future[k][0].pose.position.y+=6*(k+1);
        pred.future[k][0].pose.yaw_rad=.8*(k+1);
        pred.future[k][0].radius_error_m=scene%4;
        pred.future[k][0].interpolation_acceleration_mps2=3;
      }
      for(double offset:{0.,.07}) {
        const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,offset,false,&index};
        for(double time:{0.,.49,1.01,2.43,3.9}) {
          const PathGenerationCore::SpeedEdge edge{0,5,0,5,0,time,.9};
          EXPECT_EQ(core.edgeClearNarrow(ctx,arc,edge),core.edgeClear(ctx,arc,edge))
              <<coordinate<<":"<<scene<<":"<<offset<<":"<<time;
        }
      }
    }
  }
}

TEST(CorePathPerformanceSafety, TemporalSweepKeepsFastRearVehicleCollision) {
  const PathGenerationCore core;
  const std::vector<ReferencePoint> road{{{-100,0},-20,20},{{100,0},-20,20}};
  const PathGenerationCore::Reference ref(road);
  const auto polygon=ref.boundary(.25);
  const geometry::BoundaryIndex index(polygon);
  const std::vector<ProgressTime> schedule;
  auto pred=performanceScene({-40,0});
  for(std::size_t k=0;k<pred.future.size();++k) {
    pred.future[k]=pred.current;
    pred.future[k][0].pose.position.x+=8*(k+1);
  }
  const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,0,false,&index};
  const std::vector<PathPoint> shape{{{{0,0},0},0,0,0},{{{30,0},0},30,0,0}};
  const PathGenerationCore::ArcPath arc(shape);
  const PathGenerationCore::SpeedEdge edge{0,6,0,6,0,2,1};
  EXPECT_FALSE(core.edgeClearNarrow(ctx,arc,edge));
  EXPECT_FALSE(core.edgeClear(ctx,arc,edge));
}

TEST(CorePathPerformanceSafety, SweptBoundsMatchNarrowForNearbyTurningAndParallelVehicles) {
  const PathGenerationCore core;
  std::size_t collisions=0,clear=0;
  for(double coordinate:{0.,302000.}) {
    const std::vector<ReferencePoint> road{{{coordinate-100,coordinate},-100,100},
                                         {{coordinate+100,coordinate},-100,100}};
    const PathGenerationCore::Reference ref(road);
    const auto polygon=ref.boundary(.25);
    const geometry::BoundaryIndex index(polygon);
    const std::vector<ProgressTime> schedule;
    const std::vector<PathPoint> shape{{{{coordinate,coordinate},.1},0,0,.01},
      {{{coordinate+20,coordinate+2},.3},20,0,.01},
      {{{coordinate+40,coordinate+8},.8},40,0,.01}};
    const PathGenerationCore::ArcPath arc(shape);
    for(int scene=0;scene<240;++scene) {
      auto pred=performanceScene({coordinate+scene%17-8,coordinate+(scene/17%5-2)*1.7});
      pred.current[0].length_m=2+scene%7;
      pred.current[0].width_m=1+(scene%3)*.5;
      pred.current[0].radius_error_m=(scene%4)*.2;
      pred.current[0].pose.yaw_rad=(scene%3)*.5;
      const double turns[]{0.,.005,.03,.2,.8};
      for(std::size_t k=0;k<pred.future.size();++k) {
        pred.future[k]=pred.current;
        pred.future[k][0].pose.position.x+=(scene%5-2.)*(k+1)*.5;
        pred.future[k][0].pose.position.y+=(scene%7-3.)*(k+1)*.5;
        pred.future[k][0].pose.yaw_rad+=turns[scene%5]*(k+1);
        pred.future[k][0].interpolation_acceleration_mps2=scene%2?3:0;
      }
      const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,.3,false,&index};
      const PathGenerationCore::SpeedEdge edge{0,5,0,5,0,.49,.7};
      const bool expected=core.edgeClearNarrow(ctx,arc,edge);
      expected?++clear:++collisions;
      EXPECT_EQ(expected,core.edgeClear(ctx,arc,edge))<<coordinate<<":"<<scene;
      const PathPoint first{{{coordinate,coordinate},.1},0,.49,.01};
      const PathPoint last{{{coordinate+3,coordinate+.3},.3},3,1.19,.01};
      EXPECT_EQ(core.collisionNarrow(ctx,first,last,.1),core.collision(ctx,first,last,.1))
          <<coordinate<<":"<<scene;
    }
  }
  EXPECT_GT(collisions,0u);
  EXPECT_GT(clear,0u);
}

TEST(CorePathPerformanceSafety, SpeedEdgeBoundsKeepInteriorExcursionsOfCurvedArc) {
  PathGenerationCore core;
  const std::vector<ReferencePoint> road{{{-200,-200},-300,300},{{500,-200},-300,300}};
  const PathGenerationCore::Reference ref(road);const auto polygon=ref.boundary(.25);
  const geometry::BoundaryIndex index(polygon);const std::vector<ProgressTime> schedule;
  // 시작과 끝은 같지만 중간의 직각 꺾임에서 100 m까지 벗어나는 arc다.
  // 끝점 AABB만 사용하는 잘못된 선별은 중간 장애물을 놓친다.
  const std::vector<PathPoint> shape{{{{0,0},0},0,0,0},{{{100,0},0},100,0,0},
    {{{100,100},0},200,0,0},{{{0,100},0},300,0,0},{{{0,0},0},400,0,0}};
  const PathGenerationCore::ArcPath arc(shape);
  const PathGenerationCore::SpeedEdge edge{0,100,0,100,0,0,4};
  for(Vec2 position:std::vector<Vec2>{{100,50},{50,100},{500,500}}) {
    const auto pred=performanceScene(position);
    const PathGenerationCore::Context ctx{ref,pred,schedule,polygon,0,0,false,&index};
    EXPECT_EQ(core.edgeClearNarrow(ctx,arc,edge),core.edgeClear(ctx,arc,edge));
    if(position.x==100 || position.y==100)EXPECT_FALSE(core.edgeClear(ctx,arc,edge));
  }
}

TEST(CorePathScenarioValidation, NarrowCorridorStopsBeforeStaticObstacleWithoutInventingAdjacentPermission) {
  std::vector<ReferencePoint> reference;
  for(int x=-20;x<=160;x+=2)reference.push_back({{double(x),0},-1.75,1.75});
  EgoState ego{{{0,0},0},10};ego.speed_mps=3;
  auto scene=emptyScene();
  PredictedObject obstacle;obstacle.id=1;obstacle.kind=ObjectKind::Static;
  obstacle.pose={{15,0},0};obstacle.length_m=2;obstacle.width_m=1.8;obstacle.source_stamp_sec=10;
  scene.current.push_back(obstacle);for(auto& sample:scene.future)sample.push_back(obstacle);
  const PathGenerationCore core;const PlanningConstraints limits;
  const auto path=core.plan(ego,reference,scene,limits);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,path));
  EXPECT_DOUBLE_EQ(path.back().speed_mps,0);
  for(const auto& point:path) {
    EXPECT_LT(point.pose.position.x+VehicleShape{}.front_m,15-1-.25);
    EXPECT_LT(std::abs(point.pose.position.y)+VehicleShape{}.width_m/2,1.75);
  }
}

TEST(CorePathFollowing, SlowsBeforeReachingTheMovingLeadVehiclesBumper) {
  // R42 주도로의 14m/s 자차·11.1m/s 앞차를 단일 차로로 분리한다.
  // 0.25m 충돌 여유만 남기는 계획은 지연/추종 오차에 대응하지 못한다.
  // 아직 25m 간격이 있을 때 감속하고, 5초 계획 전체에서 5m 이상 남긴다.
  std::vector<ReferencePoint> road;
  for(int x=-20;x<=180;x+=2) road.push_back({{double(x),0},-1.75,1.75});
  EgoState ego{{{0,0},0},10};ego.speed_mps=14;
  auto scene=emptyScene();
  PredictedObject lead;lead.id=1;lead.kind=ObjectKind::Vehicle;lead.source_stamp_sec=10;
  lead.pose={{31.165008,0},0};lead.velocity_mps={11.1,0};lead.length_m=4.64;lead.width_m=2.16;
  scene.current.push_back(lead);
  for(std::size_t i=0;i<scene.future.size();++i) {
    auto next=lead;next.pose.position.x+=11.1*.5*(i+1);scene.future[i].push_back(next);
  }
  const PathGenerationCore core;const PlanningConstraints limits;
  const auto path=core.plan(ego,road,scene,limits);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
  for(const auto& point:path) {
    const double front_to_rear_gap=31.165008+11.1*point.time_sec-2.32-
                                    point.pose.position.x-3.845008;
    EXPECT_GT(front_to_rear_gap,5.0);
  }
  const auto early=std::min_element(path.begin(),path.end(),[](const PathPoint& a,const PathPoint& b) {
    return std::abs(a.time_sec-2)<std::abs(b.time_sec-2);
  });
  EXPECT_LT(early->speed_mps,14.0);
}

TEST(CorePathFollowing, AdjacentAndRearVehiclesDoNotCreateAnArtificialFollowingStop) {
  std::vector<ReferencePoint> road;
  for(int x=-20;x<=180;x+=2) road.push_back({{double(x),0},-1.75,1.75});
  EgoState ego{{{0,0},0},10};ego.speed_mps=10;
  const PathGenerationCore core;const PlanningConstraints limits;
  const auto clear=core.plan(ego,road,emptyScene(),limits);
  ASSERT_FALSE(clear.empty());
  for(Vec2 position:std::vector<Vec2>{{15,3.5},{-8,0}}) {
    SCOPED_TRACE(position.x);
    auto scene=emptyScene();
    PredictedObject actor;actor.id=1;actor.kind=ObjectKind::Vehicle;actor.source_stamp_sec=10;
    actor.pose={position,0};actor.velocity_mps={10,0};actor.length_m=4.64;actor.width_m=2.16;
    scene.current.push_back(actor);
    for(std::size_t i=0;i<scene.future.size();++i) {
      auto next=actor;next.pose.position.x+=10*.5*(i+1);scene.future[i].push_back(next);
    }
    const auto path=core.plan(ego,road,scene,limits);
    ASSERT_EQ(path.size(),clear.size());
    for(std::size_t i=0;i<path.size();++i) {
      EXPECT_DOUBLE_EQ(path[i].speed_mps,clear[i].speed_mps);
      EXPECT_DOUBLE_EQ(path[i].pose.position.x,clear[i].pose.position.x);
    }
  }
}

TEST(CorePathFollowing, ReplanningPreservesSpaceWhenTheLeadVehicleStartsBraking) {
  // 같은 앞차를 추종하다 4초 뒤 5m/s²로 제동하는 계산 폐루프다.
  // 매 회차 예측은 현재 속도의 CV만 제공하여 미래 제동을 미리 알려주지 않는다.
  // 자차는 계획의 약 0.2초 상태를 이상적으로 실행한다. 실제 MPC·지연 검증과는 별개다.
  std::vector<ReferencePoint> road;
  for(int x=-20;x<=250;x+=2) road.push_back({{double(x),0},-1.75,1.75});
  EgoState ego{{{0,0},0},10};ego.speed_mps=14;
  double lead_x=31.165008,lead_v=11.1;
  PathGenerationCore core;const PlanningConstraints limits;
  for(int tick=0;tick<60;++tick) {
    SCOPED_TRACE(tick);
    Predictions scene;scene.reference_stamp_sec=ego.stamp_sec;
    PredictedObject lead;lead.id=1;lead.kind=ObjectKind::Vehicle;lead.source_stamp_sec=ego.stamp_sec;
    lead.pose={{lead_x,0},0};lead.velocity_mps={lead_v,0};lead.length_m=4.64;lead.width_m=2.16;
    scene.current.push_back(lead);
    for(std::size_t i=0;i<scene.future.size();++i) {
      auto next=lead;next.pose.position.x+=lead_v*.5*(i+1);scene.future[i].push_back(next);
    }
    const auto path=core.plan(ego,road,scene,limits);
    ASSERT_FALSE(path.empty());
    ASSERT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
    const auto next=std::min_element(path.begin(),path.end(),[](const PathPoint& a,const PathPoint& b) {
      return std::abs(a.time_sec-.2)<std::abs(b.time_sec-.2);
    });
    const double dt=next->time_sec;
    ASSERT_GT(dt,0);ASSERT_LT(dt,.31);
    const double before_braking=std::min(dt,std::max(0.0,14-ego.stamp_sec));
    ego.pose=next->pose;ego.speed_mps=next->speed_mps;
    ego.acceleration_mps2=next->acceleration_mps2;ego.curvature_inv_m=next->curvature_inv_m;
    ego.stamp_sec+=dt;
    const double braking_time=std::min(dt-before_braking,lead_v/5);
    lead_x+=lead_v*(before_braking+braking_time)-2.5*braking_time*braking_time;
    lead_v=std::max(0.0,lead_v-5*braking_time);
    EXPECT_GT(lead_x-2.32-ego.pose.position.x-3.845008,.25);
  }
  EXPECT_LT(ego.speed_mps,.5);
}

TEST(CorePathFollowing, StoppedOrCreepingBehindAStoppedLeadDoesNotPlanAnotherDeparture) {
  // R46 정지 후 재출발 문제: 이미 정지 여유 안에 들어온 상태에서는
  // 단순 전진 거리 보상이 0속도 추종 목표를 덮어 앞차 쪽으로 밀면 안 된다.
  std::vector<ReferencePoint> road;
  for(int x=-20;x<=180;x+=2) road.push_back({{double(x),0},-1.75,1.75});
  auto scene=emptyScene();
  PredictedObject lead;lead.id=1;lead.kind=ObjectKind::Vehicle;lead.source_stamp_sec=10;
  lead.pose={{7.665008,0},0};lead.length_m=4.64;lead.width_m=2.16;
  scene.current.push_back(lead);for(auto& frame:scene.future)frame.push_back(lead);
  const PathGenerationCore core;const PlanningConstraints limits;
  for(double measured_speed:{0.0,.03}) {
    SCOPED_TRACE(measured_speed);
    EgoState ego{{{0,0},0},10};ego.speed_mps=measured_speed;
    const auto path=core.plan(ego,road,scene,limits);
    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
    // 초기 .03m/s를 1초 이내에 가속 없이 정리하는 이동 상한은 .03m다.
    EXPECT_LE(path.back().pose.position.x,.03);
    for(const auto& point:path)
      if(point.time_sec>=1) EXPECT_NEAR(point.speed_mps,0,1e-8);
  }
}

TEST(CorePathFollowing, AStoppedLeadMovingAwayAllowsDepartureAgain) {
  std::vector<ReferencePoint> road;
  for(int x=-20;x<=180;x+=2) road.push_back({{double(x),0},-1.75,1.75});
  const EgoState ego{{{0,0},0},10};auto scene=emptyScene();
  PredictedObject lead;lead.id=1;lead.kind=ObjectKind::Vehicle;lead.source_stamp_sec=10;
  lead.pose={{7.665008,0},0};lead.length_m=4.64;lead.width_m=2.16;
  scene.current.push_back(lead);
  for(std::size_t k=0;k<scene.future.size();++k) {
    auto next=lead;next.pose.position.x+=3*.5*(k+1);next.velocity_mps={3,0};
    scene.future[k].push_back(next);
  }
  const PathGenerationCore core;const PlanningConstraints limits;
  const auto path=core.plan(ego,road,scene,limits);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,limits,path));
  EXPECT_GT(path.back().pose.position.x,1);
  EXPECT_GT(path.back().speed_mps,1);
}

TEST(CorePathScenarioValidation, SuddenObstacleDistinguishesReachableStopFromInfeasibleBrakingDistance) {
  std::vector<ReferencePoint> reference;
  for(int x=-20;x<=160;x+=2)reference.push_back({{double(x),0},-1.75,1.75});
  auto scene=emptyScene();
  PredictedObject obstacle;obstacle.id=1;obstacle.kind=ObjectKind::Static;
  obstacle.pose={{8,0},0};obstacle.length_m=2;obstacle.width_m=1.8;obstacle.source_stamp_sec=10;
  scene.current.push_back(obstacle);for(auto& sample:scene.future)sample.push_back(obstacle);
  const PathGenerationCore core;const PlanningConstraints limits;
  EgoState ego{{{0,0},0},10};ego.speed_mps=3;
  const auto stop=core.plan(ego,reference,scene,limits);
  ASSERT_FALSE(stop.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,stop));
  EXPECT_DOUBLE_EQ(stop.back().speed_mps,0);
  for(const auto& point:stop)EXPECT_LT(point.pose.position.x+VehicleShape{}.front_m,8-1-.25);
  ego.speed_mps=6;
  EXPECT_TRUE(core.plan(ego,reference,scene,limits).empty());
}

TEST(CorePathScenarioValidation, FastRearVehicleInvalidatesPreviouslyClearAvoidanceSide) {
  std::vector<ReferencePoint> reference;
  for(int x=-20;x<=160;x+=2)reference.push_back({{double(x),0},-5,5});
  EgoState ego{{{0,0},0},10};ego.speed_mps=3;
  auto scene=emptyScene();
  PredictedObject obstacle;obstacle.id=1;obstacle.kind=ObjectKind::Static;
  obstacle.pose={{15,0},0};obstacle.length_m=2;obstacle.width_m=1.8;obstacle.source_stamp_sec=10;
  scene.current.push_back(obstacle);for(auto& sample:scene.future)sample.push_back(obstacle);
  const PathGenerationCore core;const PlanningConstraints limits;
  const auto clear=core.plan(ego,reference,scene,limits);
  ASSERT_FALSE(clear.empty());
  double side=0;for(const auto& point:clear)if(std::abs(point.pose.position.y)>std::abs(side))side=point.pose.position.y;
  ASSERT_GT(std::abs(side),2.2);
  PredictedObject rear;rear.id=2;rear.kind=ObjectKind::Vehicle;rear.source_stamp_sec=10;
  rear.pose={{-12,side},0};rear.velocity_mps={10,0};rear.length_m=4.64;rear.width_m=2.16;
  scene.current.push_back(rear);
  for(std::size_t i=0;i<scene.future.size();++i) {
    auto sample=rear;sample.pose.position.x+=10*scene.step_sec*(i+1);scene.future[i].push_back(sample);
  }
  EXPECT_FALSE(core.checkPlannedPath(ego,reference,scene,limits,clear));
  const auto revised=core.plan(ego,reference,scene,limits);
  ASSERT_FALSE(revised.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,revised));
}

TEST(CorePathScenarioValidation, RotatingRoundaboutTrafficIsCheckedAtItsFutureEntryPosition) {
  std::vector<ReferencePoint> reference;
  for(int x=-20;x<=160;x+=2)reference.push_back({{double(x),0},-1.75,1.75});
  EgoState ego{{{0,0},0},10};ego.speed_mps=3;
  auto scene=emptyScene();
  const auto actor=[](double time) {
    PredictedObject o;o.id=1;o.kind=ObjectKind::Vehicle;o.model=MotionModel::ConstantTurnRate;
    const double angle=std::acos(-1.0)/2-.6+.3*time;
    o.pose={{15+10*std::cos(angle),-10+10*std::sin(angle)},angle+std::acos(-1.0)/2};
    o.velocity_mps={-3*std::sin(angle),3*std::cos(angle)};
    o.length_m=4.64;o.width_m=2.16;o.source_stamp_sec=10;
    o.interpolation_acceleration_mps2=.9;
    return o;
  };
  scene.current.push_back(actor(0));
  for(std::size_t i=0;i<scene.future.size();++i)scene.future[i].push_back(actor(scene.step_sec*(i+1)));
  const PathGenerationCore core;const PlanningConstraints limits;
  const auto clear=core.plan(ego,reference,emptyScene(),limits);
  ASSERT_FALSE(clear.empty());
  EXPECT_FALSE(core.checkPlannedPath(ego,reference,scene,limits,clear));
  const auto path=core.plan(ego,reference,scene,limits);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,path));
  // 실제 원운동 식을 사용하는 독립 bbox SAT로 각 출력 시각도 확인한다.
  // 원 사이의 chord를 실제 회전 궤적으로 잘못 간주하지 않는다.
  for(const auto& point:path) {
    const auto other=actor(point.time_sec);
    const double rear_to_center=(VehicleShape{}.front_m-VehicleShape{}.rear_m)/2;
    const Vec2 own_center{point.pose.position.x+rear_to_center*std::cos(point.pose.yaw_rad),
                          point.pose.position.y+rear_to_center*std::sin(point.pose.yaw_rad)};
    bool separated=false;
    for(double axis:{point.pose.yaw_rad,point.pose.yaw_rad+std::acos(-1.0)/2,
                     other.pose.yaw_rad,other.pose.yaw_rad+std::acos(-1.0)/2}) {
      const double distance=std::abs((other.pose.position.x-own_center.x)*std::cos(axis)+
                                      (other.pose.position.y-own_center.y)*std::sin(axis));
      const double own=(VehicleShape{}.front_m+VehicleShape{}.rear_m)/2*
          std::abs(std::cos(axis-point.pose.yaw_rad))+VehicleShape{}.width_m/2*
          std::abs(std::sin(axis-point.pose.yaw_rad));
      const double obstacle=(other.length_m/2+.25)*std::abs(std::cos(axis-other.pose.yaw_rad))+
          (other.width_m/2+.25)*std::abs(std::sin(axis-other.pose.yaw_rad));
      separated=separated || distance>own+obstacle;
    }
    EXPECT_TRUE(separated);
  }
}
}  // namespace

namespace {
  std::vector<std::pair<double,double>> legacyRoadLateralIntervals(const PathGenerationCore::Context& c,const PathGenerationCore::RoadSample& road,
                                     double heading,double lower,double upper, const stier::PathParameters& p_) {
    auto box=geometry::egoBox({road.position,heading},p_.vehicle);
    box.half_length+=p_.road_margin_m; box.half_width+=p_.road_margin_m;
    const Vec2 direction=geometry::normal(road.yaw);
    const double separation=std::max(1e-6,2*p_.qp_tolerance);
    std::vector<std::pair<double,double>> intervals{{lower,upper}};
    // SAT between each road segment and the translated rectangle gives an
    // exact forbidden interval for a FIXED heading. Subtract all of them;
    // concave notches must not be reduced to four corner-distance tests.
    for(std::size_t i=0,j=c.boundary.size()-1;i<c.boundary.size();j=i++) {
      const Vec2 edge=geometry::sub(c.boundary[i],c.boundary[j]);
      const double length=geometry::norm(edge);
      if(length<1e-12) continue;
      const Vec2 segment_normal{-edge.y/length,edge.x/length};
      double enter=-std::numeric_limits<double>::infinity(),leave=std::numeric_limits<double>::infinity();
      bool intersects=true;
      for(Vec2 axis:{geometry::unit(heading),geometry::normal(heading),segment_normal}) {
        const double a=geometry::dot(geometry::sub(c.boundary[j],box.pose.position),axis);
        const double b=geometry::dot(geometry::sub(c.boundary[i],box.pose.position),axis);
        const double lo=std::min(a,b),hi=std::max(a,b);
        const double radius=geometry::extent(box,axis)+1e-9;
        const double coefficient=geometry::dot(direction,axis);
        if(std::abs(coefficient)<1e-12) {
          if(lo>radius || hi<-radius) { intersects=false; break; }
        } else {
          double first=(lo-radius)/coefficient,last=(hi+radius)/coefficient;
          if(first>last) std::swap(first,last);
          enter=std::max(enter,first); leave=std::min(leave,last);
          if(enter>leave) { intersects=false; break; }
        }
      }
      if(!intersects) continue;
      std::vector<std::pair<double,double>> remaining;
      for(const auto& range:intervals) {
        if(leave<range.first || enter>range.second) { remaining.push_back(range); continue; }
        if(range.first<enter-separation) remaining.emplace_back(range.first,std::min(range.second,enter-separation));
        if(range.second>leave+separation) remaining.emplace_back(std::max(range.first,leave+separation),range.second);
      }
      intervals=std::move(remaining);
      if(intervals.empty()) return intervals;
    }
    // No boundary intersection also describes rectangles entirely outside
    // the polygon. Admit only components whose interior is inside the road.
    intervals.erase(std::remove_if(intervals.begin(),intervals.end(),[&](const std::pair<double,double>& range) {
      auto translated=box;
      translated.pose.position=geometry::add(box.pose.position,geometry::mul(direction,(range.first+range.second)/2));
      return !geometry::polygonContainsBox(c.boundary,translated);
    }),intervals.end());
    return intervals;
  }

TEST(CorePathRoadIntervals, AcceleratedQueryMatchesEntireBoundaryAcrossConcavities) {
  PathGenerationCore core;
  for (double origin:{0.0,1000000.0}) {
    std::vector<std::vector<Vec2>> shapes{
      {{-30,-5},{30,-5},{30,5},{-30,5}},
      {{-30,-5},{30,-5},{30,5},{1,5},{1,-1},{-1,-1},{-1,5},{-30,5}},
      {{-30,-5},{30,-5},{30,5},{10,5},{10,3},{5,3},{5,5},{-30,5}}};
    std::vector<Vec2> many_edges;
    for(int i=0;i<96;++i) {
      const double angle=i*2*std::acos(-1.0)/96;
      many_edges.push_back({30*std::cos(angle),5*std::sin(angle)});
    }
    shapes.push_back(many_edges);
    for(auto polygon:shapes) {
      for(auto& q:polygon){q.x+=origin;q.y+=origin;}
      const PathGenerationCore::Reference reference({{{origin-30,origin},-5,5},{{origin+30,origin},-5,5}});
      const auto predictions=emptyScene();
      const std::vector<ProgressTime> schedule{{0,0},{60,10}};
      const PathGenerationCore::Context context(reference,predictions,schedule,polygon,0,0);
      for(double x:{-28.0,-10.0,-2.0,0.0,2.0,7.0,20.0,28.0})
        for(double y:{-6.0,-2.0,0.0,2.0,6.0})
          for(double yaw:{0.0,0.01,0.4,1.57,-1.0}) {
            PathGenerationCore::RoadSample sample;
            sample.position={origin+x,origin+y};sample.yaw=yaw;
            const auto expected=legacyRoadLateralIntervals(context,sample,yaw+0.17,-4,4,stier::PathParameters{});
            EXPECT_EQ(expected,core.roadLateralIntervals(context,sample,yaw+0.17,-4,4));
          }
    }
  }
}
TEST(CorePathCentering, RealMapSpatialHeadingAdmitsVerifiedCandidateBelowCurvatureLimit) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+2,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(10);ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-96.28843542781232;
  ego.pose.pose.position.y=-365.41460475549883;
  ego.pose.pose.orientation.x=-0.004977772597635423;
  ego.pose.pose.orientation.y=-0.004495747509295877;
  ego.pose.pose.orientation.z=-0.560457035939349;
  ego.pose.pose.orientation.w=-0.8281563384412258;
  ego.twist.twist.linear.x=6.244244619595516;
  ego.twist.twist.linear.y=-0.23745767247823574;
  ego.twist.twist.angular.z=0.26109930872917175;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.header.stamp+=ros::Duration(-0.0004971027374267578);status.velocity_x_mps=6.2643842697143555;
  sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=-0.9322239756584167;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  PathGenerationCore core(parameters);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,std::max(ego.header.stamp,status.header.stamp));
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_EQ(path.points.front().time_from_start.toNSec(),0u);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4);
}

TEST(CorePathCentering, DisposableGeometryTimingDoesNotLimitFinalSafeAvoidanceSpeed) {
  std::vector<ReferencePoint> reference;
  for(int x=-20;x<=180;x+=2)reference.push_back({{double(x),0},-1.75,5.25});
  EgoState ego{{{10,0},0},10};ego.speed_mps=2;
  auto scene=emptyScene();PredictedObject obstacle;obstacle.id=1;
  obstacle.kind=ObjectKind::Static;obstacle.pose={{35,0},0};
  obstacle.length_m=2;obstacle.width_m=1.8;obstacle.source_stamp_sec=10;
  scene.current.push_back(obstacle);for(auto& sample:scene.future)sample.push_back(obstacle);
  const PathGenerationCore core;
  // 20m/s는 형상 탐색용 가정일 때만 버릴 수 있다. 외부에서 실행할 schedule로
  // 제공한 경우 원래 횡가속도 한도를 그대로 적용해야 한다.
  EXPECT_TRUE(core.plan(ego,reference,scene,std::vector<ProgressTime>{{0,0},{100,5}}).empty());
  const auto path=core.plan(ego,reference,scene,PlanningConstraints{});
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,PlanningConstraints{},path));
  double lateral=0;
  for(const auto& point:path) {
    lateral=std::max(lateral,point.pose.position.y);
    EXPECT_LE(point.speed_mps*point.speed_mps*std::abs(point.curvature_inv_m),2.0+1e-6);
  }
  EXPECT_GT(lateral,2.2);
}

TEST(CorePathCentering, IntegratedStaticAvoidanceUsesReachableSlowGeometryInsteadOfOnlyFastGuesses) {
  std::vector<ReferencePoint> reference;
  for(int x=-20;x<=160;x+=2)reference.push_back({{double(x),0},-5.0,5.0});
  EgoState ego{{{0,0},0},10};ego.speed_mps=3;
  auto scene=emptyScene();
  PredictedObject obstacle;obstacle.id=1;obstacle.kind=ObjectKind::Static;
  obstacle.pose={{15,0},0};obstacle.length_m=2;obstacle.width_m=1.8;obstacle.source_stamp_sec=10;
  scene.current.push_back(obstacle);for(auto& sample:scene.future)sample.push_back(obstacle);
  const PathGenerationCore core;const PlanningConstraints limits;
  const auto path=core.plan(ego,reference,scene,limits);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,limits,path));
  double deviation=0;for(const auto& p:path)deviation=std::max(deviation,std::abs(p.pose.position.y));
  EXPECT_GT(deviation,2.2);
  EXPECT_GT(path.back().pose.position.x,16.0);
}


TEST(CorePathCentering, ComparesExistingLongConnectionBeforeUnnecessaryHardBraking) {
  // R102 고주로의 동일 입력은 두 기하 모두 전체 검사를 통과한다.
  // 첫 25m C2 연결의 -5m/s² 급감속보다 기존 50m 연결의 낮은 비용을 선택한다.
  // 객체를 제외해 경로 후보 조기 반환만 재현하고, 도로·차체·동역학 한도는 유지한다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
  ego.header.stamp=ros::Time(1792404138,148672992);
  ego.pose.pose.position.x=75.20861313412979;
  ego.pose.pose.position.y=-191.8366991095744;
  ego.pose.pose.position.z=28.69995845847714;
  ego.pose.pose.orientation.x=0.0001239998519783432;
  ego.pose.pose.orientation.y=0.0032863251350585716;
  ego.pose.pose.orientation.z=-0.7077248802907323;
  ego.pose.pose.orientation.w=0.7064804869977742;
  ego.twist.twist.linear.x=20.588552159370405;
  ego.twist.twist.linear.y=0.01185021440534278;
  ego.twist.twist.linear.z=0.09875108541267963;
  ego.twist.twist.angular.x=0.012221675366163254;
  ego.twist.twist.angular.y=0.004682562779635191;
  ego.twist.twist.angular.z=-0.027345117181539536;
  interfaces::CompetitionVehicleStatus status;status.header.frame_id="base_link";
  status.header.stamp=ros::Time(1792404138,199318169);
  status.velocity_x_mps=20.579195022583008;
  status.signed_velocity_mps=20.579195022583008;
  status.link_id="A2256W000423";
  sensor_msgs::Imu imu;imu.header.frame_id="imu_link";
  imu.header.stamp=ros::Time(1792404138,148672992);
  imu.linear_acceleration.x=-0.6609674692153931;
  imu.linear_acceleration.y=-0.5902129411697388;
  imu.linear_acceleration.z=9.408308982849121;
  interfaces::Objects objects;objects.header.frame_id="map";
  objects.header.stamp=ros::Time(1792404138,162097000);
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  path_planner::GlobalPath global(shared);
  const path_planner::Checkpoints remaining(checkpoints.begin()+12,checkpoints.end());
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  PathGenerationCore core(parameters,shared);
  ros::Time stamp;stamp.fromNSec(1792404138199318169ULL);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,stamp);
  ASSERT_GE(path.points.size(),2u);
  const auto two=std::lower_bound(path.points.begin(),path.points.end(),2.0,
      [](const interfaces::PathPoint& point,double t){return point.time_from_start.toSec()<t;});
  ASSERT_NE(two,path.points.end());
  EXPECT_GT(two->velocity,15.0);
  for(const auto& point:path.points) EXPECT_GE(point.acceleration,-3.0);
}

}  // namespace


TEST(CorePathCartesianRecovery, PreservesRearAxleYawAndMeasuredCurvature) {
  const std::vector<ReferencePoint> points{{{-10,0},-3,3},{{0,0},-3,3},
                                          {{20,0},-3,3},{{30,0},-3,3}};
  const PathGenerationCore::Reference reference(points);
  const auto predictions=emptyScene();
  const std::vector<ProgressTime> timing;
  const std::vector<Vec2> boundary{{-10,-3},{30,-3},{30,3},{-10,3}};
  const PathGenerationCore::Context context{reference,predictions,timing,boundary,0,0,true};
  EgoState ego{{{0,.2},.03},10}; ego.curvature_inv_m=.02;
  const PathGenerationCore core;
  const auto path=core.cartesianRecovery(context,ego,10,0);
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().pose.position.x,ego.pose.position.x);
  EXPECT_DOUBLE_EQ(path.front().pose.position.y,ego.pose.position.y);
  EXPECT_DOUBLE_EQ(path.front().pose.yaw_rad,ego.pose.yaw_rad);
  EXPECT_DOUBLE_EQ(path.front().curvature_inv_m,ego.curvature_inv_m);
  EXPECT_NEAR(path.back().pose.position.x,10,1e-8);
  EXPECT_NEAR(path.back().pose.position.y,0,1e-8);
  EXPECT_NEAR(path.back().curvature_inv_m,0,1e-10);
  for(const auto& point:path) EXPECT_LE(std::abs(point.curvature_inv_m),PathParameters().max_curvature_inv_m);
}

TEST(CorePathCartesianRecovery, RejectsWholeVehicleCrossingOriginalConcaveBoundary) {
  const std::vector<ReferencePoint> points{{{-10,0},-3,3},{{0,0},-3,3},
                                          {{20,0},-3,3},{{30,0},-3,3}};
  const PathGenerationCore::Reference reference(points);
  const auto predictions=emptyScene();
  const std::vector<ProgressTime> timing;
  const std::vector<Vec2> boundary{{-10,-3},{30,-3},{30,3},{10,3},
                                  {10,.5},{8,.5},{8,3},{-10,3}};
  const PathGenerationCore::Context context{reference,predictions,timing,boundary,0,0,true};
  const EgoState ego{{{0,0},0},10};
  const PathGenerationCore core;
  EXPECT_TRUE(core.cartesianRecovery(context,ego,15,0).empty());
}


TEST(CorePathCartesianRecovery, KeepsCurvedEndpointRatherThanForcingStraightTerminalGeometry) {
  const auto points=circle(8,1);
  const PathGenerationCore::Reference reference(points);
  const auto predictions=emptyScene();
  const std::vector<ProgressTime> timing;
  const PathGenerationCore core;
  const auto road=core.roadGeometry(points,reference);
  const PathGenerationCore::Context context{reference,predictions,timing,road->boundary,0,0,true,&road->index};
  EgoState ego{{{0,.3},0},10}; ego.curvature_inv_m=1/7.7;
  const auto path=core.cartesianRecovery(context,ego,5,.3);
  ASSERT_GE(path.size(),2u);
  EXPECT_NEAR(path.back().curvature_inv_m,1/7.7,1e-4);
  expectInsideCircle(path,8,1);
}


TEST(CorePathCartesianRecovery, NoisyCurveReturnsTowardMinimumNecessaryFootprintOffset) {
  auto points=circle(8,1);
  for(std::size_t i=0;i<points.size();++i) {
    const double angle=(static_cast<double>(i)-20)*.025;
    const double noise=.015*std::sin(static_cast<double>(i)*std::acos(-1.0)/2);
    points[i].position.x-=noise*std::sin(angle);
    points[i].position.y+=noise*std::cos(angle);
  }
  PathParameters parameters;
  parameters.speed_beam_width=16;
  const PathGenerationCore core(parameters);
  EgoState ego{{{0,.55},0},10}; ego.curvature_inv_m=1/7.45;
  const auto path=core.plan(ego,points,emptyScene(),PlanningConstraints{});
  ASSERT_FALSE(path.empty());
  double largest_progress=0;
  double later_minimum_inward=std::numeric_limits<double>::infinity();
  for(const auto& point:path) {
    largest_progress=std::max(largest_progress,point.progress_m);
    if(point.progress_m>=4) later_minimum_inward=std::min(later_minimum_inward,inwardOffset(point.pose,8,1));
  }
  EXPECT_GE(largest_progress,4);
  EXPECT_LT(later_minimum_inward,.48);
  EXPECT_TRUE(core.checkPlannedPath(ego,points,emptyScene(),PlanningConstraints{},path));
}

TEST(CorePathCentering, RealKCityMovingCurveDoesNotSkipFeasibleIntermediateWindow) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+2,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;
  ego.header.frame_id="map";ego.header.stamp=ros::Time(10);ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-98.09237300597431;ego.pose.pose.position.y=-368.3074912260218;
  auto& q=ego.pose.pose.orientation;
  q.x=-.0020636957251203175;q.y=-.004233154019449056;
  q.z=-.5062654846564469;q.w=-.8623648187470165;
  ego.twist.twist.linear.x=6.840279284398519;ego.twist.twist.linear.y=.21090440393248616;
  ego.twist.twist.angular.z=-.07224581390619278;
  interfaces::CompetitionVehicleStatus competition;competition.header=ego.header;
  competition.header.stamp+=ros::Duration(.047131275);
  competition.velocity_x_mps=6.828403472900391;
  sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=-1.944955825805664;
  interfaces::Objects objects;objects.header=ego.header;
  objects.header.stamp+=ros::Duration(.046127302);
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,competition,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  // 당시 2.0/1.8m/s² 조건의 수치 회귀를 주행 설정과 독립적으로 보존한다.
  parameters.max_lateral_accel_mps2=2.0;
  parameters.preferred_lateral_accel_mps2=1.8;
  path_planner::CorePathPlanner core(parameters);
  const auto path=core.process(ego,competition,objects,{},imu,route,predictions,competition.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,competition.velocity_x_mps);
  EXPECT_LT(path.points.back().velocity,competition.velocity_x_mps);
  EXPECT_GT(path.points.back().time_from_start.toSec(),4);
  EXPECT_LE(path.points.back().time_from_start.toSec(),5);
}

TEST(CorePathCentering, RealSecondCurveChecksOtherRetainedSpeedCandidates) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+2,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;
  ego.header.frame_id="map";ego.header.stamp=ros::Time(10);ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-121.85578205485385;
  ego.pose.pose.position.y=-334.86862524258464;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.0016587018973345028;q.y=0.0002850412405841421;
  q.z=0.9510159950879508;q.w=0.30913709668427275;
  ego.twist.twist.linear.x=4.453105830142626;
  ego.twist.twist.linear.y=0.10645565552608584;
  ego.twist.twist.angular.z=-0.14657840132713318;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;
  status.velocity_x_mps=4.463997840881348f;
  sensor_msgs::Imu imu;imu.header=ego.header;
  imu.linear_acceleration.x=0.10393084585666656;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  path_planner::CorePathPlanner core(parameters);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_NEAR(path.points.front().pose.pose.position.x,ego.pose.pose.position.x,1e-8);
  EXPECT_NEAR(path.points.front().pose.pose.position.y,ego.pose.pose.position.y,1e-8);
  EXPECT_EQ(path.points.front().time_from_start.toNSec(),0u);
  for(std::size_t i=1;i<path.points.size();++i)
    EXPECT_GT(path.points[i].time_from_start.toNSec(),path.points[i-1].time_from_start.toNSec());
}
TEST(CorePathCentering, RealNoisyCurveTangentDoesNotRejectSafeCartesianCandidate) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+2,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;
  ego.header.frame_id="map";ego.header.stamp=ros::Time(10);ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-96.25352007796423;
  ego.pose.pose.position.y=-365.1698966697655;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.00529341426206808;q.y=-0.00625197647718428;
  q.z=-0.5753792428810907;q.w=-0.8178457185907143;
  ego.twist.twist.linear.x=5.715742697404896;
  ego.twist.twist.linear.y=0.17766439858230873;
  ego.twist.twist.angular.z=0.30892759561538696;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;
  status.header.stamp+=ros::Duration(.007287264);
  status.velocity_x_mps=5.735223293304443f;
  sensor_msgs::Imu imu;imu.header=ego.header;
  imu.linear_acceleration.x=-0.7929072976112366;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  path_planner::CorePathPlanner core(parameters);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,status.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_GT(path.points.front().pose.pose.position.x,ego.pose.pose.position.x);
  EXPECT_GT(path.points.front().pose.pose.position.y,ego.pose.pose.position.y);
  EXPECT_EQ(path.points.front().time_from_start.toNSec(),0u);
  for(std::size_t i=1;i<path.points.size();++i)
    EXPECT_GT(path.points[i].time_from_start.toNSec(),path.points[i-1].time_from_start.toNSec());
}

TEST(CorePathCentering, CartesianAdmissionKeepsRealInitialHeadingLimit) {
  const std::vector<ReferencePoint> points{{{-10,0},-5,5},{{0,0},-5,5},
                                          {{30,0},-5,5},{{60,0},-5,5}};
  const PathGenerationCore core;
  for(double yaw:{-.6,.6,3.0}) {
    EgoState ego{{{5,0},yaw},10};
    EXPECT_TRUE(core.plan(ego,points,emptyScene(),PlanningConstraints{}).empty());
  }
}
TEST(CorePathCentering, RealSecondCurveRetainsTheExistingDpLateralGoalRange) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+2,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;
  ego.header.frame_id="map";ego.header.stamp=ros::Time(10);ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-120.00235257963791;
  ego.pose.pose.position.y=-336.15616486669563;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.0001977440153445753;q.y=0.0014963160223367244;
  q.z=-0.9566808652070073;q.w=-0.29113475244710624;
  ego.twist.twist.linear.x=4.4983141877126105;
  ego.twist.twist.linear.y=-0.0022796267745337933;
  ego.twist.twist.angular.z=-0.22414273023605347;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;
  status.header.stamp+=ros::Duration(.004727602);
  status.velocity_x_mps=4.509627342224121f;
  sensor_msgs::Imu imu;imu.header=ego.header;
  imu.linear_acceleration.x=0.5240482091903687;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  path_planner::CorePathPlanner core(parameters);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,status.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_LT(path.points.front().pose.pose.position.x,ego.pose.pose.position.x);
  EXPECT_GT(path.points.front().pose.pose.position.y,ego.pose.pose.position.y);
  EXPECT_EQ(path.points.front().time_from_start.toNSec(),0u);
  for(std::size_t i=1;i<path.points.size();++i)
    EXPECT_GT(path.points[i].time_from_start.toNSec(),path.points[i-1].time_from_start.toNSec());
}
TEST(CorePathCentering, RevisitedIntersectionUsesActualRoadUnionInsteadOfCancellingOverlap) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+9,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(10);
  ego.child_frame_id="base_link";
  ego.pose.pose.position.x=-86.60539707169094;ego.pose.pose.position.y=334.67904814927203;
  ego.pose.pose.orientation.x=-0.0020564713740315165;
  ego.pose.pose.orientation.y=-0.0012736971137438677;
  ego.pose.pose.orientation.z=-0.42379972332989524;
  ego.pose.pose.orientation.w=-0.9057526942420069;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.brake=1;
  sensor_msgs::Imu imu;imu.linear_acceleration.z=9.81;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  PathGenerationCore core(parameters,shared);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_GT(path.points.back().velocity,0);
  const auto region=core.nativeRoadRegion(core.rearFootprintRoute(route));
  ASSERT_TRUE(static_cast<bool>(region));
  for(const auto& p:path.points) {
    const auto& q=p.pose.pose.orientation;
    Pose2 pose{{p.pose.pose.position.x,p.pose.pose.position.y},
      std::atan2(2*q.w*q.z,1-2*q.z*q.z)};
    EXPECT_TRUE(region->containsBox(geometry::sweptEgoEnvelope(pose,pose,parameters.vehicle,parameters.road_margin_m)));
  }
}

TEST(CorePathCentering, RealMainRoadMergeKeepsFeasibleLongProgress) {
  // R50 주도로 진입 실측이다. 장애물 없는 동일 초기 상태에서 40m C2
  // 후보는 원래 전체 검사에 통과하지만, 37.5m와 25m 사이만 추가 탐색하면
  // 짧은 창 끝 때문에 5초 뒤 속도를 0.63m/s까지 낮추는 경로를 택했다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto loaded=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+10,loaded.end());
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(
          package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
  ego.header.stamp.fromSec(1792316944.7754571);
  ego.pose.pose.position.x=59.027244502570476;
  ego.pose.pose.position.y=291.23872050819097;
  auto& q=ego.pose.pose.orientation;
  q.x=-0.004178234675173381;q.y=-0.0027003755624904523;
  q.z=0.6340805643515949;q.w=-0.7732509865744652;
  ego.twist.twist.linear.x=8.724589210002007;
  ego.twist.twist.linear.y=-0.0905660404708791;
  ego.twist.twist.angular.z=-0.0653684139251709;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;
  status.header.stamp.fromSec(1792316944.790207);
  status.velocity_x_mps=8.70865535736084;
  sensor_msgs::Imu imu;imu.header=ego.header;
  imu.header.stamp.fromSec(1792316944.7902741);
  imu.linear_acceleration.x=-0.29675713181495667;
  imu.linear_acceleration.y=-0.5541955828666687;
  imu.linear_acceleration.z=9.969154357910156;
  interfaces::Objects objects;objects.header=ego.header;
  objects.header.stamp.fromSec(1792316944.773022);
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  path_planner::GlobalPath global(shared);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  for(double preferred:{0.,parameters.preferred_lateral_accel_mps2}) {
  SCOPED_TRACE(preferred);
  auto configured=parameters;configured.preferred_lateral_accel_mps2=preferred;
  PathGenerationCore core(configured,shared);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,imu.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  // 기존 목표의 속도 기준은 유지한다. 선호 목표를 낮춘 경우에도 직접
  // 진행 거리를 확인해 원래의 25m 창/0.63m/s 선택이 다시 통과하지 못하게 한다.
  if(preferred==0) EXPECT_GT(path.points.back().velocity,2.0);
  EXPECT_GT(path.points.back().velocity,0);
  EXPECT_GT(core.previous_path_.back().progress_m,35.0);
  const auto region=core.nativeRoadRegion(core.rearFootprintRoute(route));
  for(const auto& point:path.points) {
    const auto& orientation=point.pose.pose.orientation;
    const Pose2 pose{{point.pose.pose.position.x,point.pose.pose.position.y},
        std::atan2(2*orientation.w*orientation.z,1-2*orientation.z*orientation.z)};
    EXPECT_TRUE(region->containsBox(geometry::sweptEgoEnvelope(
        pose,pose,parameters.vehicle,parameters.road_margin_m)));
  }
  }
}

TEST(CorePathCentering, ConsecutiveNativeLaneChangesKeepTheirSharedConnector) {
  // 실제 12구간의 연속된 두 번의 합법적 옆 차선 변경이다. 112314를 첫
  // 변경 연장에 소비하면 두 번째 변경이 빠져, 평행한 도로에서 5cm 전진에
  // 3.5m 횡이동하는 가짜 직각 기준선이 만들어진다. 지도나 목표 차선을
  // 바꾸지 않고 두 번째 변경도 전진하는 연속 기준선으로 전달해야 한다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  PathGenerationCore core(parameters,shared);
  lanelet::ConstLanelets lanes;
  for(const auto id:{97093,97095,112314,112288,97452})
    lanes.push_back(shared.map->laneletLayer.get(id));
  const path_planner::Route route=lanelet::routing::LaneletPath(lanes);
  const auto first_target=core.legalNeighbor(lanes[0],true);
  const auto second_target=core.legalNeighbor(lanes[2],true);
  ASSERT_TRUE(static_cast<bool>(first_target));ASSERT_EQ(first_target->id(),97095);
  ASSERT_TRUE(static_cast<bool>(second_target));ASSERT_EQ(second_target->id(),112288);
  const auto reference=core.nativeReference(route);
  ASSERT_GE(reference->points.size(),3u);
  bool reaches_target=false;
  for(std::size_t i=1;i<reference->points.size();++i) {
    const auto a=reference->points[i-1].position,b=reference->points[i].position;
    if(a.y> -142.82 || b.y< -170) continue;
    const double forward=a.y-b.y,lateral=std::abs(b.x-a.x);
    // 이 평행 직선의 옆 차선 변경을 가짜 횡방향 도로로 해석하지 않는다.
    EXPECT_GE(forward,-1e-8);
    EXPECT_LT(std::atan2(lateral,forward),geometry::kPi/4);
    reaches_target=reaches_target || b.x>74.9;
  }
  EXPECT_TRUE(reaches_target);
}

TEST(CorePathCentering, CentimetreLaneChangeConnectorUsesFollowingPermittedParallelRoad) {
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto map_params=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_params));
  const auto loaded=path_planner::readCheckpoints(map_params.asset("checkpoints_file"));
  const path_planner::Checkpoints checkpoints(loaded.begin()+12,loaded.end());
  path_planner::GlobalPath global(shared);
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(10);
  ego.child_frame_id="base_link";
  ego.pose.pose.position.x=71.13796222513805;ego.pose.pose.position.y=-141.7949939060139;
  ego.pose.pose.orientation.x=-0.0038987419012773594;
  ego.pose.pose.orientation.y=0.0019089054033379735;
  ego.pose.pose.orientation.z=0.5953000215507231;
  ego.pose.pose.orientation.w=-0.8034917798169791;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.brake=1;
  sensor_msgs::Imu imu;imu.linear_acceleration.z=9.81;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  PathGenerationCore core(parameters,shared);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_GT(path.points.back().velocity,0);
  EXPECT_LT(path.points.back().pose.pose.position.y,-145.0);
  const auto region=core.nativeRoadRegion(core.rearFootprintRoute(route));
  ASSERT_TRUE(static_cast<bool>(region));
  for(const auto& p:path.points) {
    const auto& q=p.pose.pose.orientation;
    Pose2 pose{{p.pose.pose.position.x,p.pose.pose.position.y},
      std::atan2(2*q.w*q.z,1-2*q.z*q.z)};
    EXPECT_TRUE(region->containsBox(geometry::sweptEgoEnvelope(pose,pose,parameters.vehicle,parameters.road_margin_m)));
  }
}

TEST(CorePathCentering, StaticAvoidanceStartsWithinTheLegalAdjacentLaneInsteadOfBrakingOnAPrefix) {
  // R55의 두 접근 상태다. 37.5m 중앙 창이 장애물 직전에 끝나 회피 탐색을
  // 선점하면, 5초 뒤 약 0.7~0.8m/s로 감속하며 점점 가까이 접근한다.
  // 실측 크기의 정적 장애물과 실제 지도에서 옆 차선이 열리는 위치를 지키며
  // 전진 가능한 회피 후보를 선택해야 한다. 속도 하한이나 경계 완화는 없다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  const path_planner::Checkpoints remaining(checkpoints.begin()+4,checkpoints.end());
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(
          package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  const std::array<std::array<double,16>,3> fixtures{{
    {{-65.48519201510871,-195.93612979643015,-0.00041088881733557974,-0.002518170257229604,-0.6046080085110999,-0.7965190807715641,10.619282996052329,-0.060632582632296976,0.07413938641548157,10.626062393188477,0.25447559356689453,1792325317,247246569,1792325317,241383000,1792325317.2725592}},
    {{-61.031383103261355,-172.39449889293934,-0.0006925814082300282,-0.0047077682037129425,-0.6681950675940121,-0.7439709059449615,7.86987702151623,0.022233109459449507,0.025633515790104866,7.79890775680542,-1.4913793802261353,1792325319,802201010,1792325319,795586000,1792325319.8307297}},
    {{-59.8588885407245,-157.17528776290973,-.0009670383778599232,-.004630903339509838,-.6972388971515542,-.7168232277695252,5.5222148024437905,.03725584920295548,.02481391467154026,5.477463722229004,-1.7752766609191895,1792329636,708362424,1792329636,700643000,1792329636.7347567}}
  }};
  // R58의 세 번째 상태는 회피 형상이 장애물 직후 끝나 tail 속도가 0이었다.
  // 같은 도로에서 회피 후 검증된 형상을 이어야 하며, 멀리 있는 두 번째
  // 정적 장애물이 있으면 그 앞 정지 가능성도 유지해야 한다.
  for(const auto& f:fixtures) for(bool later_static:{false,true}) {
    if(later_static && f[1]<-160) continue;
    SCOPED_TRACE(f[1]);
    SCOPED_TRACE(later_static);
    nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
    ego.header.stamp=ros::Time(static_cast<uint32_t>(f[11]),static_cast<uint32_t>(f[12]));
    ego.pose.pose.position.x=f[0];ego.pose.pose.position.y=f[1];
    auto& q=ego.pose.pose.orientation;q.x=f[2];q.y=f[3];q.z=f[4];q.w=f[5];
    ego.twist.twist.linear.x=f[6];ego.twist.twist.linear.y=f[7];ego.twist.twist.angular.z=f[8];
    interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.velocity_x_mps=f[9];
    sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=f[10];
    interfaces::Objects objects;objects.header=ego.header;
    objects.header.stamp=ros::Time(static_cast<uint32_t>(f[13]),static_cast<uint32_t>(f[14]));
    interfaces::Object cube;cube.id=1;cube.label="obstacle";
    cube.position.x=-60.60789108276367;cube.position.y=-142.1780242919922;
    cube.bbox_pose.position=cube.position;
    cube.bbox_pose.orientation.x=-.001272961015163604;
    cube.bbox_pose.orientation.y=.0011796844896673337;
    cube.bbox_pose.orientation.z=-.664049584598853;
    cube.bbox_pose.orientation.w=.7476865232895732;
    cube.bbox_size.x=1.497259259223938;cube.bbox_size.y=1.497259259223938;
    cube.bbox_size.z=1.3032472133636475;
    cube.velocity_available=true;cube.acceleration_available=true;
    objects.objects.push_back(cube);
    if(later_static) {
      auto later=cube;later.id=2;later.position.y=-95;
      later.bbox_pose.position=later.position;objects.objects.push_back(later);
    }
    path_planner::PredictedObjects predictions;predictions.fill(objects);
    path_planner::GlobalPath global(shared);
    const auto route=std::get<5>(global.process(ego,status,objects,{},imu,remaining));
    ASSERT_TRUE(static_cast<bool>(route));
    PathGenerationCore core(parameters,shared);
    ros::Time stamp;stamp.fromSec(f[15]);
    const auto path=core.process(ego,status,objects,{},imu,route,predictions,stamp);
    ASSERT_GE(path.points.size(),2u);
    EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
    EXPECT_GT(path.points.back().velocity,3.0);
    if(f[1]>-180 && f[1]<-160) EXPECT_GT(path.points.back().pose.pose.position.x,-58.2);
    if(f[1]>-160) EXPECT_GT(path.points.back().pose.pose.position.y,-130);
    const auto region=core.nativeRoadRegion(core.rearFootprintRoute(route));
    ASSERT_TRUE(static_cast<bool>(region));
    for(const auto& point:core.previous_path_)
      EXPECT_TRUE(region->containsBox(geometry::sweptEgoEnvelope(
          point.pose,point.pose,parameters.vehicle,parameters.road_margin_m)));
    const auto& first=core.previous_path_.front();
    EgoState initial;initial.pose=first.pose;initial.stamp_sec=stamp.toSec();
    initial.speed_mps=f[9];initial.curvature_inv_m=f[8]/f[9];
    initial.acceleration_mps2=f[10]-(Eigen::Quaterniond(f[5],f[2],f[3],f[4]).conjugate()*
                                    Eigen::Vector3d(0,0,9.81)).x();
    Predictions scene;scene.reference_stamp_sec=objects.header.stamp.toSec();
    PredictedObject obstacle;obstacle.id=1;
    obstacle.pose={{cube.position.x,cube.position.y},std::atan2(
        2*(cube.bbox_pose.orientation.w*cube.bbox_pose.orientation.z+
           cube.bbox_pose.orientation.x*cube.bbox_pose.orientation.y),
        1-2*(std::pow(cube.bbox_pose.orientation.y,2)+std::pow(cube.bbox_pose.orientation.z,2)))};
    obstacle.length_m=cube.bbox_size.x;obstacle.width_m=cube.bbox_size.y;
    obstacle.source_stamp_sec=objects.header.stamp.toSec();
    scene.current.push_back(obstacle);
    if(later_static) {
      auto later=obstacle;later.id=2;later.pose.position.y=-95;scene.current.push_back(later);
    }
    scene.future.fill(scene.current);
    const PathGenerationCore::NativeRegionScope scope(core.active_native_region_,region);
    EXPECT_TRUE(core.checkPlannedPath(initial,core.road_geometry_->input,scene,
                                     PlanningConstraints{},core.previous_path_));
  }
}

TEST(CorePathReferenceProjection, IncludesSplineBulgesBeyondTheChordAtLargeMapCoordinates) {
  // Chord AABB로 가지치기하면 y<0인 실제 C2 곡선이 누락된다.
  for(double origin:{0.,300000.,4124145.}) {
    SCOPED_TRACE(origin);
    const PathGenerationCore::Reference reference({
        {{origin,origin},-5,5},{{origin+10,origin},-5,5},
        {{origin+10,origin+10},-5,5},{{origin,origin+10},-5,5}});
    const auto point=reference.at(5);
    ASSERT_LT(point.position.y,origin-1);
    const auto projected=reference.project(point.position,0,reference.length());
    EXPECT_NEAR(projected.s,5,1e-6);
    EXPECT_LT(projected.distance,1e-6);
  }
}

TEST(CorePathReferenceProjection, PreservesTheEarliestEqualMatchOnARevisitedRoad) {
  const PathGenerationCore::Reference reference({
      {{0,0},-5,5},{{10,0},-5,5},{{10,10},-5,5},
      {{0,10},-5,5},{{0,0},-5,5},{{-10,0},-5,5}});
  const auto projected=reference.project({0,0},0,reference.length());
  EXPECT_DOUBLE_EQ(projected.s,0);
  EXPECT_DOUBLE_EQ(projected.distance,0);
}

TEST(CorePathReferenceProjection, PreservesTheExistingAdmittedSegmentRange) {
  // lo/hi는 포함할 구간을 고른다. 원래 투영을 점 단위로 잘라 다른 값으로
  // 바꾸지 않으며, 구간 순서가 뒤집혀 하나도 포함되지 않으면 기존 빈 결과다.
  const PathGenerationCore::Reference reference({
      {{-10,0},-5,5},{{0,0},-5,5},{{10,0},-5,5},{{20,0},-5,5}});
  const auto projected=reference.project({12,2},5,15);
  EXPECT_DOUBLE_EQ(projected.s,20);
  EXPECT_DOUBLE_EQ(projected.l,2);
  EXPECT_NEAR(projected.distance,std::sqrt(8.),1e-12);
  const auto reversed=reference.project({12,2},15,5);
  EXPECT_DOUBLE_EQ(reversed.s,0);
  EXPECT_TRUE(std::isinf(reversed.distance));
}

TEST(CorePathCentering, UsesTheRepresentedCurvePeakForSpeedMargin) {
  // R61 교차로 출구 커브 진입 전 실측 상태다. 명시 곡률만 비용에 사용하면
  // 5초 궤적의 실제 위치/방향 곡률이 1.99m/s²까지 올라 선호 여유를 잃는다.
  // 먼 객체는 제외하고 같은 map·초기 운동에서 미래 곡률 목표만 검증한다.
  ros::Time::init();
  const std::string package_dir=NP_PACKAGE_DIR;
  const auto reader=node_pipeline::ParamReader::fromFile(
      package_dir+"/config/path_planner/parameters/shared_map.yaml",package_dir);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(reader));
  const auto checkpoints=path_planner::readCheckpoints(reader.asset("checkpoints_file"));
  const path_planner::Checkpoints remaining(checkpoints.begin()+10,checkpoints.end());
  auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(
          package_dir+"/config/path_planner/parameters/core_path_planner.yaml",package_dir));
  // 당시 2.0/1.8m/s² 조건의 수치 회귀를 주행 설정과 독립적으로 보존한다.
  parameters.max_lateral_accel_mps2=2.0;
  parameters.preferred_lateral_accel_mps2=1.8;
  const std::array<double,16> f{{-39.94926048603136,344.1870716673704,-0.004823559878040202,-0.0004145418794970713,-0.032736239290370395,-0.999452300043507,7.35938471627136,-0.08770120580704979,-0.008913299068808556,7.365354537963867,0.3106602132320404,1792332608,487155928,1792332608,496263000,1792332608.5156608}};
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";
  ego.header.stamp=ros::Time(static_cast<uint32_t>(f[11]),static_cast<uint32_t>(f[12]));
  ego.pose.pose.position.x=f[0];ego.pose.pose.position.y=f[1];
  auto& q=ego.pose.pose.orientation;q.x=f[2];q.y=f[3];q.z=f[4];q.w=f[5];
  ego.twist.twist.linear.x=f[6];ego.twist.twist.linear.y=f[7];ego.twist.twist.angular.z=f[8];
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.velocity_x_mps=f[9];
  sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.x=f[10];
  interfaces::Objects objects;objects.header=ego.header;
  objects.header.stamp=ros::Time(static_cast<uint32_t>(f[13]),static_cast<uint32_t>(f[14]));
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  path_planner::GlobalPath global(shared);
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  PathGenerationCore core(parameters,shared);
  ros::Time stamp;stamp.fromSec(f[15]);
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,status.velocity_x_mps);
  EXPECT_DOUBLE_EQ(parameters.max_lateral_accel_mps2,2.0);
  EXPECT_GT(core.previous_path_.back().progress_m,30);
  EXPECT_GT(path.points.back().velocity,6.0);
  // Path 메시지의 방향은 실제 제어 참조다. 인접한 출력점 사이의 yaw 변화와
  // 같은 구간의 속도 상한으로 횡가속도를 계산한다. 공간 knot 사이에 시간
  // knot가 추가되어도 같은 선분의 yaw/거리 비는 변하지 않는다. 불균일한
  // 세 점의 위치 곡률은 짧은 보간 chord 옆에서 과대 추정될 수 있으므로
  // 이 비용 목표 검증에 섞지 않는다. 위치 형상·hard 한도는 기존 최종
  // checkPlannedPath와 별도 회귀 테스트가 계속 검사한다.
  double maximum_lateral=0;
  const auto yaw=[](const geometry_msgs::Quaternion& q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  };
  for(std::size_t i=1;i<path.points.size();++i) {
    const auto& a=path.points[i-1];const auto& b=path.points[i];
    const double distance=std::hypot(b.pose.pose.position.x-a.pose.pose.position.x,
                                     b.pose.pose.position.y-a.pose.pose.position.y);
    if(distance<=1e-9) continue;
    const double curvature=std::abs(geometry::wrap(
        yaw(b.pose.pose.orientation)-yaw(a.pose.pose.orientation)))/distance;
    maximum_lateral=std::max(maximum_lateral,
        std::max(a.velocity*a.velocity,b.velocity*b.velocity)*curvature);
  }
  EXPECT_LT(maximum_lateral,1.85);
  const auto region=core.nativeRoadRegion(core.rearFootprintRoute(route));
  for(const auto& point:core.previous_path_)
    EXPECT_TRUE(region->containsBox(geometry::sweptEgoEnvelope(
        point.pose,point.pose,parameters.vehicle,parameters.road_margin_m)));
}

TEST(CorePathFollowing, SamePredictionTimeKeepsPoseCapAndSceneSpecificTargets) {
  // 같은 미래 객체라도 자차 자세·현재 속도 상한·다음 관측 장면은 다르다.
  // 예측 보간 재사용이 최종 목표속도나 이전 장면까지 재사용해서는 안 된다.
  const PathGenerationCore core;
  const std::vector<ReferencePoint> road{{{-20,0},-6,6},{{100,0},-6,6}};
  const PathGenerationCore::Reference reference(road);
  const auto boundary=reference.boundary(.25);
  const geometry::BoundaryIndex index(boundary);
  const std::vector<ProgressTime> schedule;
  auto scene=emptyScene();
  PredictedObject lead;lead.id=1;lead.kind=ObjectKind::Vehicle;
  lead.pose={{12,0},0};lead.length_m=4;lead.width_m=2;
  lead.velocity_mps={4,0};lead.source_stamp_sec=10;
  scene.current.push_back(lead);
  for(std::size_t i=0;i<scene.future.size();++i) {
    auto future=lead;future.pose.position.x+=2*(i+1);
    scene.future[i].push_back(future);
  }
  const PathGenerationCore::Context c{reference,scene,schedule,boundary,0,0,false,&index};
  EXPECT_NEAR(core.followingSpeedTarget(c,{{0,0},0},0,20,4,0),
              (12-3.845008-2.25-2)/1.5,1e-12);
  EXPECT_NEAR(core.followingSpeedTarget(c,{{0,0},0},.5,20,4,0),
              (14-3.845008-2.25-2)/1.5,1e-12);
  EXPECT_NEAR(core.followingSpeedTarget(c,{{0,0},0},0,20,6,0),1.9366613333333333,1e-12);
  EXPECT_DOUBLE_EQ(core.followingSpeedTarget(c,{{4,0},0},0,20,4,0),0);
  EXPECT_DOUBLE_EQ(core.followingSpeedTarget(c,{{0,4},0},0,20,4,0),20);
  EXPECT_DOUBLE_EQ(core.followingSpeedTarget(c,{{0,0},geometry::kPi},0,20,4,0),20);
  EXPECT_DOUBLE_EQ(core.followingSpeedTarget(c,{{0,0},0},0,1,4,0),1);
  auto distant=scene;distant.current[0].pose.position.x+=28;
  for(auto& future:distant.future)future[0].pose.position.x+=28;
  const PathGenerationCore::Context next{reference,distant,schedule,boundary,0,0,false,&index};
  EXPECT_DOUBLE_EQ(core.followingSpeedTarget(next,{{0,0},0},0,20,4,0),20);
  EXPECT_NEAR(core.followingSpeedTarget(c,{{0,0},0},0,20,4,0),
              (12-3.845008-2.25-2)/1.5,1e-12);
}

TEST(CorePathCentering, LoweredCeilingKeepsCruiseAfterAFeasibleJerkLimitedRecovery) {
  // 전방이 비어 있는 완만한 중앙 복귀에서는 4m/s까지 도달한 뒤 다시
  // 1~2m/s로 불필요하게 감속하지 않는다. 짧은 출력 구간의 Hermite 재구성
  // 반올림 때문에 원래 가능한 긴 경로가 버려지는 회귀를 재현한다.
  ros::Time::init();const std::string package=NP_PACKAGE_DIR;
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(package+"/config/path_planner/parameters/core_path_planner.yaml",package));
  path_planner::CorePathPlanner core(parameters);
  std::vector<stier::ReferencePoint> road;
  for(int i=-10;i<=150;++i)road.push_back({{static_cast<double>(i),0},-1.85,1.85});
  stier::EgoState ego;ego.stamp_sec=10.028301457;ego.pose.position={0,.3};
  ego.pose.yaw_rad=-.00155;ego.speed_mps=8.5012884140014648;
  ego.acceleration_mps2=-1.3261539736292345;ego.curvature_inv_m=.000710222044168453;
  stier::Predictions predictions;predictions.reference_stamp_sec=10;
  stier::PlanningConstraints limits;limits.maximum_speed_mps=4;
  const auto path=core.plan(ego,road,predictions,limits);ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
  EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
  const auto at_two=std::lower_bound(path.begin(),path.end(),2.,
      [](const stier::PathPoint& q,double t){return q.time_sec<t;});
  ASSERT_NE(at_two,path.end());EXPECT_NEAR(at_two->speed_mps,4,1e-5);
  EXPECT_NEAR(path.back().speed_mps,4,1e-6);
  EXPECT_NEAR(path.back().time_sec,5-(ego.stamp_sec-10),1e-8);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,predictions,limits,path));
}

TEST(CorePathSpeedLimits, SmallMeasuredCruiseOverrunKeepsJerkLimitedRecoveryInsteadOfEmptyPath) {
  PathParameters p;p.trajectory_horizon_m=125;
  PathGenerationCore core(p);EgoState ego{{{0,0},0},10};
  ego.speed_mps=20.0136;ego.acceleration_mps2=.17;
  std::vector<ReferencePoint> reference{{{-10,0},-5,5},{{200,0},-5,5}};
  Predictions scene;scene.reference_stamp_sec=10;
  const auto path=core.plan(ego,reference,scene,PlanningConstraints{});
  ASSERT_GE(path.size(),2u);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
  EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
  EXPECT_LE(path.back().speed_mps,p.max_speed_mps+1e-8);
  const double unavoidable_peak=ego.speed_mps+ego.acceleration_mps2*ego.acceleration_mps2/(2*p.max_longitudinal_jerk_mps3);
  for(const auto& point:path) {
    EXPECT_LE(point.speed_mps,unavoidable_peak+1e-8);
    if(point.speed_mps>p.max_speed_mps+1e-8 || point.time_sec<ego.acceleration_mps2/p.max_longitudinal_jerk_mps3)
      EXPECT_LE(point.acceleration_mps2,std::max(0.,ego.acceleration_mps2-p.max_longitudinal_jerk_mps3*point.time_sec)+1e-8);
  }
  EXPECT_TRUE(core.checkPlannedPath(ego,reference,scene,PlanningConstraints{},path));
}

TEST(CorePathCentering, ReturnsFromInitialFootprintBoundaryOverhangWithoutMovingFartherOut) {
  PathParameters parameters;
  parameters.max_speed_mps=6;
  PathGenerationCore core(parameters);
  const std::vector<ReferencePoint> road{{{-20,0},-2,2},{{150,0},-2,2}};
  const auto boundary=PathGenerationCore::Reference(road).boundary(.25);
  auto region=std::make_shared<PathGenerationCore::NativeRoadRegion>();
  PathGenerationCore::RoadPolygon polygon;
  for(const auto& point:boundary)polygon.outer().emplace_back(point.x,point.y);
  boost::geometry::correct(polygon);region->polygons.push_back(polygon);region->prepare();
  core.active_native_region_=region;
  const EgoState ego{{{0,1.2},0},10};
  const auto scene=emptyScene();
  const auto path=core.plan(ego,road,scene,PlanningConstraints{});
  ASSERT_GE(path.size(),2u);
  EXPECT_NEAR(path.front().pose.position.y,1.2,1e-10);
  EXPECT_GT(path.back().progress_m,1);
  EXPECT_TRUE(geometry::polygonContainsBox(boundary,
      geometry::sweptEgoEnvelope(path.back().pose,path.back().pose,parameters.vehicle,parameters.road_margin_m)));
  for(const auto& point:path) EXPECT_LE(point.pose.position.y,1.2+1e-8);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,scene,PlanningConstraints{},path));
}

TEST(CorePathCentering, BoundaryRecoveryCannotBypassAnObstacleCoveringTheReturn) {
  PathParameters parameters;parameters.max_speed_mps=6;
  PathGenerationCore core(parameters);
  const std::vector<ReferencePoint> road{{{-20,0},-2,2},{{150,0},-2,2}};
  const auto boundary=PathGenerationCore::Reference(road).boundary(.25);
  auto region=std::make_shared<PathGenerationCore::NativeRoadRegion>();
  PathGenerationCore::RoadPolygon polygon;
  for(const auto& point:boundary)polygon.outer().emplace_back(point.x,point.y);
  boost::geometry::correct(polygon);region->polygons.push_back(polygon);region->prepare();
  core.active_native_region_=region;
  const EgoState ego{{{0,1.2},0},10};
  auto scene=emptyScene();
  PredictedObject obstacle;obstacle.id=1;obstacle.pose.position={2,1};
  obstacle.length_m=4;obstacle.width_m=5;
  scene.current.push_back(obstacle);
  for(auto& future:scene.future)future.push_back(obstacle);
  EXPECT_TRUE(core.plan(ego,road,scene,PlanningConstraints{}).empty());
}

TEST(CorePathCentering, RealStoppedFrontCornerOutsideRoadReturnsToAllowedRegion) {
  ros::Time::init();
  const std::string pkg=NP_PACKAGE_DIR;
  const auto map_reader=node_pipeline::ParamReader::fromFile(
      pkg+"/config/path_planner/parameters/shared_map.yaml",pkg);
  path_planner::SharedMap shared(path_planner::loadSharedMapParameters(map_reader));
  const auto checkpoints=path_planner::readCheckpoints(map_reader.asset("checkpoints_file"));
  path_planner::GlobalPath global(shared);
  const auto parameters=path_planner::loadCorePathPlannerParameters(
      node_pipeline::ParamReader::fromFile(pkg+"/config/path_planner/parameters/core_path_planner.yaml",pkg));
  PathGenerationCore core(parameters,shared);
  nav_msgs::Odometry ego;ego.header.stamp=ros::Time(10);ego.header.frame_id="map";
  ego.child_frame_id="base_link";
  ego.pose.pose.position.x=28.8849006354;ego.pose.pose.position.y=338.9962192554;
  const double yaw=-.48869670875;
  ego.pose.pose.orientation.z=std::sin(yaw/2);ego.pose.pose.orientation.w=std::cos(yaw/2);
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.brake=1;
  interfaces::Objects objects;objects.header=ego.header;
  path_planner::PredictedObjects predictions;predictions.fill(objects);
  sensor_msgs::Imu imu;imu.header=ego.header;imu.linear_acceleration.z=9.81;
  const path_planner::Checkpoints remaining(checkpoints.begin()+10,checkpoints.end());
  const auto route=std::get<5>(global.process(ego,status,objects,{},imu,remaining));
  ASSERT_TRUE(static_cast<bool>(route));
  const auto original=core.nativeRoadRegion(core.rearFootprintRoute(route));
  const Pose2 initial{{ego.pose.pose.position.x,ego.pose.pose.position.y},yaw};
  EXPECT_FALSE(original->containsBox(geometry::sweptEgoEnvelope(
      initial,initial,parameters.vehicle,parameters.road_margin_m)));
  const auto path=core.process(ego,status,objects,{},imu,route,predictions,ego.header.stamp);
  ASSERT_GE(path.points.size(),2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity,0);
  EXPECT_NEAR(path.points.front().pose.pose.position.x,ego.pose.pose.position.x,1e-10);
  EXPECT_GT(path.points.back().velocity,0);
  const auto& last=path.points.back().pose.pose;
  const double last_yaw=2*std::atan2(last.orientation.z,last.orientation.w);
  const Pose2 end{{last.position.x,last.position.y},last_yaw};
  EXPECT_TRUE(original->containsBox(geometry::sweptEgoEnvelope(
      end,end,parameters.vehicle,parameters.road_margin_m)));
}

TEST(CorePathCentering, LegacyTimedCheckerDoesNotActivateRoadReentry) {
  PathGenerationCore core;
  const std::vector<ReferencePoint> road{{{-20,0},-2,2},{{150,0},-2,2}};
  const auto boundary=PathGenerationCore::Reference(road).boundary(.25);
  auto region=std::make_shared<PathGenerationCore::NativeRoadRegion>();
  PathGenerationCore::RoadPolygon polygon;
  for(const auto& point:boundary)polygon.outer().emplace_back(point.x,point.y);
  boost::geometry::correct(polygon);region->polygons.push_back(polygon);region->prepare();
  core.active_native_region_=region;
  const EgoState ego{{{0,1.2},0},10};
  std::vector<PathPoint> hold(2);hold[0].pose=ego.pose;
  hold[1]=hold[0];hold[1].time_sec=5;
  EXPECT_FALSE(core.checkTimedPath(ego,road,emptyScene(),hold));
}
