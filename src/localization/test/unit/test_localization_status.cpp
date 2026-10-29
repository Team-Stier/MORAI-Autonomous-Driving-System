#include <localization/cores/full_ekf.hpp>
#include <localization/cores/tunnel_wall.hpp>
#include <gtest/gtest.h>

TEST(LocalizationStatus, ReportsTheExecutedEstimatorAcrossModeTransitions) {
  localization::EkfState shared;
  localization::FullEkf gps(localization::EkfParameters{},shared);
  localization::TunnelWall tunnel(localization::TunnelWallParameters{},shared);
  sensor_msgs::NavSatFix fix;fix.latitude=37.4;fix.longitude=127.1;fix.altitude=30;
  sensor_msgs::Imu imu;imu.header.stamp=ros::Time(10);imu.linear_acceleration.z=9.81;
  interfaces::CompetitionVehicleStatus vehicle;vehicle.header.stamp=imu.header.stamp;vehicle.velocity_x_mps=5;
  fix.header.stamp=imu.header.stamp;
  auto fused=gps.process(fix,imu,vehicle);
  EXPECT_EQ(std::get<1>(fused).mode,interfaces::LocalizationStatus::GPS_FUSED);
  EXPECT_EQ(std::get<0>(fused).child_frame_id,"base_link");
  const auto initial=std::get<0>(fused).pose.pose.position;
  sensor_msgs::PointCloud2 cloud;cloud.header.stamp=imu.header.stamp;
  cloud.header.frame_id="base_link_level";cloud.height=1;cloud.point_step=16;
  for (const auto* name : {"x","y","z","time"}) {
    sensor_msgs::PointField field;field.name=name;field.offset=4*cloud.fields.size();
    field.datatype=sensor_msgs::PointField::FLOAT32;field.count=1;cloud.fields.push_back(field);
  }
  imu.header.stamp=ros::Time(10.02);vehicle.header.stamp=imu.header.stamp;
  auto inertial=tunnel.process(imu,vehicle,cloud);
  EXPECT_EQ(std::get<1>(inertial).mode,interfaces::LocalizationStatus::IMU_LIDAR);
  EXPECT_NEAR(std::get<0>(inertial).pose.pose.position.x,initial.x,.5);
  EXPECT_NEAR(std::get<0>(inertial).pose.pose.position.y,initial.y,.5);
  imu.header.stamp=ros::Time(10.04);fix.header.stamp=imu.header.stamp;vehicle.header.stamp=imu.header.stamp;
  EXPECT_EQ(std::get<1>(gps.process(fix,imu,vehicle)).mode,interfaces::LocalizationStatus::GPS_FUSED);
}
