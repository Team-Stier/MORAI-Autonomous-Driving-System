#include <lidar_leveler/core_set.hpp>
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

  lidar_leveler::CoreSet cores{lidar_leveler::PassThrough{}};
  lidar_leveler::Data data;
  data.get<0>().sample.message = std::make_shared<const sensor_msgs::PointCloud2>(cloud);
  const auto required = cores.requiredInputs(lidar_leveler::CoreId::PassThrough);
  EXPECT_FALSE(data.hasInputs(required));
  data.get<2>().sample.message = std::make_shared<const tf2_msgs::TFMessage>(tf_static);
  EXPECT_TRUE(data.hasInputs(required));

  auto expected_cloud = cloud;
  sensor_msgs::PointCloud2Iterator<float> expected_x(expected_cloud, "x"),
    expected_y(expected_cloud, "y"), expected_z(expected_cloud, "z");
  *expected_x = 8; *expected_y = 21; *expected_z = 33;
  ++expected_x; ++expected_y; ++expected_z;
  *expected_x = 10; *expected_y = 19; *expected_z = 30.5;
  const auto expected = ros::serialization::serializeMessage(expected_cloud);
  const auto actual = ros::serialization::serializeMessage(cores.process(lidar_leveler::CoreId::PassThrough, data));
  ASSERT_EQ(expected.num_bytes, actual.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected.buf.get(), actual.buf.get(), expected.num_bytes));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
