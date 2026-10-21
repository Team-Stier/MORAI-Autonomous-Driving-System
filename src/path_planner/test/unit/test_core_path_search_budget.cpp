#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/core_path_planner.hpp>
#include <gtest/gtest.h>
namespace {
TEST(CorePathSearchBudget, DistantMapObjectDoesNotChangeTheFreeRoadSpeedPlan) {

  // 검증된 고주로 입력에 수 km 떨어진 이동 객체를 추가한다.
  // 이 객체는 도로·차체·예측 충돌과 분리되므로 기존 시간 경로가 유지돼야 한다.
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

  path_planner::CorePathPlanner empty_core(parameters,shared), distant_core(parameters,shared);
  ros::Time stamp;stamp.fromNSec(1792404138199318169ULL);
  const auto empty=empty_core.process(ego,status,objects,{},imu,route,predictions,stamp);
  ASSERT_GE(empty.points.size(),2u);
  // This vehicle remains kilometres outside every candidate and prediction sweep.
  interfaces::Object far;far.id=1;far.label="vehicle";
  far.position.x=far.bbox_pose.position.x=10000;
  far.position.y=far.bbox_pose.position.y=10000;
  far.bbox_pose.orientation.w=1;
  far.bbox_size.x=4.5;far.bbox_size.y=2.1;far.bbox_size.z=1.5;
  far.velocity_available=true;far.acceleration_available=true;far.velocity.x=1;
  objects.objects.push_back(far);
  for(std::size_t i=0;i<predictions.size();++i) {
    predictions[i]=objects;
    predictions[i].objects[0].position.x+=(i+1)*.5;
    predictions[i].objects[0].bbox_pose.position=predictions[i].objects[0].position;
  }
  const auto distant=distant_core.process(ego,status,objects,{},imu,route,predictions,stamp);
  EXPECT_TRUE(empty==distant) << "A remote object changed the free-road timed path";
}
}  // namespace
