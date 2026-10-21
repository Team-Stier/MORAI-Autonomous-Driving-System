#include <control/cores/morai_mpc.hpp>
#include <control/parameters/morai_mpc.hpp>
#include <gtest/gtest.h>
#include <cmath>
namespace {
control::autoware_ai_mpc::Parameters parameters() {
  ros::Time::init();const std::string root=NP_PACKAGE_DIR;
  return control::loadMoraiMpcParameters(node_pipeline::ParamReader::fromFile(root+"/config/control/parameters/morai_mpc.yaml",root));
}
interfaces::Path straightPath(double speed) {
  interfaces::Path p;p.header.frame_id="base_link";p.header.stamp=ros::Time(100,0);
  p.map_from_path_frame.header.frame_id="map";p.map_from_path_frame.child_frame_id="base_link";
  p.map_from_path_frame.header.stamp=p.header.stamp;p.map_from_path_frame.transform.rotation.w=1;
  for(int i=0;i<=240;++i){interfaces::PathPoint q;q.pose.header=p.header;q.pose.pose.position.x=i*.5;
    q.pose.pose.orientation.w=1;q.velocity=speed;q.time_from_start=ros::Duration(i*.5/std::max(.5,speed));p.points.push_back(q);}
  return p;
}
morai_msgs::CtrlCmd firstCommand(double speed,double yaw_rate) {
  control::MoraiMpc core(parameters());const auto path=straightPath(speed);
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=path.header.stamp;
  ego.child_frame_id="base_link";ego.pose.pose.orientation.w=1;ego.twist.twist.linear.x=speed;ego.twist.twist.angular.z=yaw_rate;
  interfaces::CompetitionVehicleStatus status;status.header=ego.header;status.velocity_x_mps=speed;
  return core.process(path,ego,status,ego.header.stamp);
}
TEST(MoraiMpcMeasuredLateral, CounteractsMeasuredRotationBeforePoseErrorAppears) {
  for(double speed:{6.,12.,25.}) {
    SCOPED_TRACE(speed);
    EXPECT_LT(firstCommand(speed,.12).steering,-1e-4);
    EXPECT_GT(firstCommand(speed,-.12).steering,1e-4);
    EXPECT_NEAR(firstCommand(speed,0).steering,0,1e-9);
  }
}
TEST(MoraiMpcMeasuredLateral, DoesNotDivideByStandstillNoise) {
  for(double speed:{0.,.03,.4}) {
    const auto left=firstCommand(speed,1e-4),right=firstCommand(speed,-1e-4);
    EXPECT_TRUE(std::isfinite(left.steering));EXPECT_NEAR(left.steering,0,1e-9);
    EXPECT_DOUBLE_EQ(left.steering,right.steering);
  }
}
TEST(MoraiMpcMeasuredLateral, PreservesPedalsAndSteeringBounds) {
  for(double speed:{6.,12.,25.}) {
    const auto left=firstCommand(speed,.12),right=firstCommand(speed,-.12),zero=firstCommand(speed,0);
    EXPECT_DOUBLE_EQ(left.accel,zero.accel);EXPECT_DOUBLE_EQ(right.accel,zero.accel);
    EXPECT_DOUBLE_EQ(left.brake,zero.brake);EXPECT_DOUBLE_EQ(right.brake,zero.brake);
    EXPECT_LE(std::abs(left.steering),120*.05/40+1e-8);EXPECT_LE(std::abs(right.steering),120*.05/40+1e-8);
    EXPECT_LE(std::abs(left.steering),1);EXPECT_LE(std::abs(right.steering),1);
  }
}
}
