#include <lidar_leveler/cores/pass_through_roi.hpp>
#include <lidar_leveler/parameters/pass_through_roi.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cstring>

TEST(PassThroughRoi, DropsPointsOutsideDefaultHeightRange) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(2);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = 1; *y = 0; *z = -1000;
  ++x; ++y; ++z;
  *x = 1; *y = 0; *z = 1000;

  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(1);
  auto& mount = tf_static.transforms.front();
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.rotation.w = 1;

  lidar_leveler::PassThroughRoi core(lidar_leveler::PassThroughRoiParameters{});
  const auto output = core.process(cloud, tf_static);
  EXPECT_EQ(0u, output.width);
  EXPECT_EQ(1u, output.height);
  EXPECT_EQ(0u, output.row_step);
  EXPECT_TRUE(output.data.empty());
}

TEST(PassThroughRoi, AppliesTfBeforeInclusiveRoiAndPreservesPointRecords) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 17;
  cloud.header.stamp.fromNSec(1234567890);
  cloud.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32,
    "ring", 1, sensor_msgs::PointField::UINT16);
  cloud.width = 3;
  cloud.height = 2;
  cloud.point_step = 32;
  cloud.row_step = 104;
  cloud.is_dense = false;
  cloud.data.assign(cloud.height * cloud.row_step, 0xa5);
  const float input_z[] = {-2, -1, 0, 2, 3, 100};
  for (std::size_t i = 0; i < 6; ++i) {
    auto* record = cloud.data.data() + (i / 3) * cloud.row_step + (i % 3) * cloud.point_step;
    const float values[] = {static_cast<float>(i), static_cast<float>(i), input_z[i], 100.0f + i};
    const auto ring = static_cast<std::uint16_t>(i);
    std::memcpy(record, values, sizeof(values));
    std::memcpy(record + 16, &ring, sizeof(ring));
  }
  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(1);
  auto& mount = tf_static.transforms.front();
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.translation.x = 10;
  mount.transform.translation.z = 2;
  mount.transform.rotation.w = 1;

  lidar_leveler::PassThroughRoiParameters parameters;
  parameters.z_min_m = 1;
  parameters.z_max_m = 4;
  lidar_leveler::PassThroughRoi core(parameters);
  const auto output = core.process(cloud, tf_static);

  auto expected = cloud;
  expected.width = 3;
  expected.height = 1;
  expected.row_step = 96;
  expected.data.resize(96);
  const std::size_t retained[] = {1, 2, 3};
  const float expected_xyz[][3] = {{11, 1, 1}, {12, 2, 2}, {13, 3, 4}};
  for (std::size_t i = 0; i < 3; ++i) {
    const auto index = retained[i];
    auto* record = expected.data.data() + i * expected.point_step;
    std::memcpy(record, cloud.data.data() + (index / 3) * cloud.row_step + (index % 3) * cloud.point_step,
                cloud.point_step);
    std::memcpy(record, expected_xyz[i], sizeof(expected_xyz[i]));
  }
  const auto expected_bytes = ros::serialization::serializeMessage(expected);
  const auto actual_bytes = ros::serialization::serializeMessage(output);
  ASSERT_EQ(expected_bytes.num_bytes, actual_bytes.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected_bytes.buf.get(), actual_bytes.buf.get(), expected_bytes.num_bytes));
}

TEST(PassThroughRoi, ReturnsEmptyCloudForEmptyInput) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(1);
  auto& mount = tf_static.transforms.front();
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.rotation.w = 1;
  lidar_leveler::PassThroughRoi core(lidar_leveler::PassThroughRoiParameters{});
  const auto output = core.process(cloud, tf_static);
  EXPECT_EQ(0u, output.width);
  EXPECT_EQ(1u, output.height);
  EXPECT_EQ(0u, output.row_step);
  EXPECT_TRUE(output.data.empty());
}

TEST(PassThroughRoi, DecidesBoundsBeforeFloat32Rounding) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = 0; *y = 0; *z = 0;
  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(1);
  auto& mount = tf_static.transforms.front();
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.rotation.w = 1;
  struct Case {
    double translation_z_m;
    double z_min_m;
    double z_max_m;
    std::uint32_t expected_width;
  };
  const Case cases[] = {{-0.3, -0.3, 0.3, 1}, {0.3, -0.3, 0.3, 1},
                       {-0.500000001, -0.5, 4.5, 0}, {4.500000001, -0.5, 4.5, 0}};
  for (const auto& test_case : cases) {
    SCOPED_TRACE(test_case.translation_z_m);
    mount.transform.translation.z = test_case.translation_z_m;
    lidar_leveler::PassThroughRoiParameters parameters;
    parameters.z_min_m = test_case.z_min_m;
    parameters.z_max_m = test_case.z_max_m;
    lidar_leveler::PassThroughRoi core(parameters);
    const auto output = core.process(cloud, tf_static);
    EXPECT_EQ(test_case.expected_width, output.width);
  }
}

TEST(PassThroughRoi, LoaderReadsOrderedBounds) {
  const auto parameters = lidar_leveler::loadPassThroughRoiParameters(
      node_pipeline::ParamReader::fromString("{z_min_m: -0.5, z_max_m: 4.5}"));
  EXPECT_DOUBLE_EQ(-0.5, parameters.z_min_m);
  EXPECT_DOUBLE_EQ(4.5, parameters.z_max_m);
  EXPECT_THROW(lidar_leveler::loadPassThroughRoiParameters(
      node_pipeline::ParamReader::fromString("{z_min_m: 1.0, z_max_m: 0.0}")), std::invalid_argument);
}
