#include <lidar_leveler/cores/pass_through.hpp>
#include <lidar_leveler/cores/pass_through_roi.hpp>
#include <lidar_leveler/cores/roll_pitch_leveler.hpp>
#include <lidar_leveler/cores/roll_pitch_roi_leveler.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cstring>
#include <cmath>

TEST(PassThrough, AppliesMountingTfAndPreservesOtherFields) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 7;
  cloud.header.stamp.fromNSec(1234567890);
  cloud.header.frame_id = "velodyne";
  cloud.height = 1;
  cloud.width = 2;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32,
    "ring", 1, sensor_msgs::PointField::UINT16);
  cloud.point_step = 32;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.assign(cloud.row_step, 0xa5);
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = 1; *y = 2; *z = 3;
  ++x; ++y; ++z;
  *x = -1; *y = 0; *z = 0.5;

  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(2);
  tf_static.transforms[0].header.frame_id = "base_link";
  tf_static.transforms[0].child_frame_id = "gps_link";
  auto& mount = tf_static.transforms[1];
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.translation.x = 10;
  mount.transform.translation.y = 20;
  mount.transform.translation.z = 30;
  mount.transform.rotation.z = std::sqrt(0.5);
  mount.transform.rotation.w = std::sqrt(0.5);

  lidar_leveler::PassThrough core;

  auto expected_cloud = cloud;
  sensor_msgs::PointCloud2Iterator<float> expected_x(expected_cloud, "x"),
    expected_y(expected_cloud, "y"), expected_z(expected_cloud, "z");
  *expected_x = 8; *expected_y = 21; *expected_z = 33;
  ++expected_x; ++expected_y; ++expected_z;
  *expected_x = 10; *expected_y = 19; *expected_z = 30.5;
  const auto expected = ros::serialization::serializeMessage(expected_cloud);
  const auto actual = ros::serialization::serializeMessage(
    core.process(cloud, interfaces::CompetitionVehicleStatus{}, tf_static));
  ASSERT_EQ(expected.num_bytes, actual.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected.buf.get(), actual.buf.get(), expected.num_bytes));
}

TEST(LevelCores, EveryRegisteredCoreUsesTheSameLeveledCoordinates) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "velodyne";
  cloud.height = 1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = 0;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = 2;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = 0;
  interfaces::CompetitionVehicleStatus competition;
  competition.roll_rad = std::acos(-1.0) / 2;
  tf2_msgs::TFMessage mounts;
  mounts.transforms.resize(1);
  mounts.transforms[0].header.frame_id = "base_link";
  mounts.transforms[0].child_frame_id = "velodyne";
  mounts.transforms[0].transform.rotation.w = 1;
  const sensor_msgs::PointCloud2 outputs[] = {
    lidar_leveler::PassThrough{}.process(cloud, competition, mounts),
    lidar_leveler::PassThroughRoi{}.process(cloud, competition, mounts),
    lidar_leveler::RollPitchLeveler{}.process(cloud, competition, mounts),
    lidar_leveler::RollPitchRoiLeveler{}.process(cloud, competition, mounts)};
  for (const auto& output : outputs) {
    ASSERT_EQ(1u, output.width);
    EXPECT_NEAR(0, *sensor_msgs::PointCloud2ConstIterator<float>(output, "y"), 1e-6);
    EXPECT_NEAR(2, *sensor_msgs::PointCloud2ConstIterator<float>(output, "z"), 1e-6);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
