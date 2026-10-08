#include <gtest/gtest.h>
#include <localization/cores/full_ekf.hpp>

TEST(FullEkf, PublishesBodyTwistAndDoesNotFuseDuplicateSamples) {
  localization::EkfParameters parameters;
  parameters.yaw_offset_rad = 0;
  localization::StateBank bank;
  localization::FullEkf core(parameters, bank);
  sensor_msgs::NavSatFix gps;
  gps.latitude = 37.0;
  gps.longitude = 127.0;
  gps.altitude = 10;
  gps.header.stamp = ros::Time(1, 0);
  sensor_msgs::Imu imu;
  imu.header.stamp = gps.header.stamp;
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp = gps.header.stamp;
  status.yaw_rad = 1.5707963267948966;
  status.velocity_x_mps = 3;
  status.angular_velocity_x_radps = 0.1;
  status.angular_velocity_y_radps = 0.2;
  status.angular_velocity_z_radps = 0.3;
  const auto first = core.process(gps, imu, status);
  const auto duplicate = core.process(gps, imu, status);
  EXPECT_NEAR(first.twist.twist.linear.x, 3, 1e-6);
  EXPECT_NEAR(first.twist.twist.linear.y, 0, 1e-6);
  EXPECT_NEAR(first.twist.twist.linear.z, 0, 1e-6);
  EXPECT_DOUBLE_EQ(first.pose.pose.position.x, duplicate.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(first.twist.twist.linear.x, duplicate.twist.twist.linear.x);
  EXPECT_EQ(first.child_frame_id, "base_link");
  EXPECT_TRUE(first.header.frame_id.empty());
  EXPECT_TRUE(first.header.stamp.isZero());
  EXPECT_FLOAT_EQ(first.twist.twist.angular.x, status.angular_velocity_x_radps);
  EXPECT_FLOAT_EQ(first.twist.twist.angular.y, status.angular_velocity_y_radps);
  EXPECT_FLOAT_EQ(first.twist.twist.angular.z, status.angular_velocity_z_radps);
  for (double value : first.pose.covariance) EXPECT_DOUBLE_EQ(value, 0);
  for (double value : first.twist.covariance) EXPECT_DOUBLE_EQ(value, 0);
}

TEST(FullEkf, EstimatesLateralAndVerticalVelocityFromGpsMotion) {
  localization::EkfParameters parameters;
  parameters.yaw_offset_rad = 0;
  parameters.gps_position_noise = 0.01;
  localization::StateBank bank;
  localization::FullEkf core(parameters, bank);
  sensor_msgs::NavSatFix gps;
  gps.latitude = 37.0;
  gps.longitude = 127.0;
  gps.altitude = 10;
  sensor_msgs::Imu imu;
  interfaces::CompetitionVehicleStatus status;
  nav_msgs::Odometry result;
  for (int i = 0; i < 201; ++i) {
    gps.latitude = 37.0 + i * 0.00000018;
    gps.altitude = 10 + i * 0.02;
    gps.header.stamp.fromSec(1 + i * 0.02);
    imu.header.stamp = gps.header.stamp;
    status.header.stamp = gps.header.stamp;
    result = core.process(gps, imu, status);
  }
  EXPECT_NEAR(result.twist.twist.linear.x, 0, 0.05);
  EXPECT_NEAR(result.twist.twist.linear.y, 1, 0.05);
  EXPECT_NEAR(result.twist.twist.linear.z, 1, 0.05);
  EXPECT_DOUBLE_EQ(result.pose.pose.position.z, 0);
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
