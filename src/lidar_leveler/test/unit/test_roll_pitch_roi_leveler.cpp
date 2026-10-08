#include <lidar_leveler/cores/roll_pitch_leveler.hpp>
#include <lidar_leveler/cores/roll_pitch_roi_leveler.hpp>
#include <lidar_leveler/parameters/roll_pitch_roi_leveler.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>
#include <cmath>

namespace {
sensor_msgs::PointCloud2 cloudOf(const std::vector<std::array<float, 3>>& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& p : points) {
    *x = p[0]; *y = p[1]; *z = p[2];
    ++x; ++y; ++z;
  }
  return cloud;
}
tf2_msgs::TFMessage mountAt(double z_m) {
  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(1);
  auto& mount = tf_static.transforms.front();
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.translation.z = z_m;
  mount.transform.rotation.w = 1;
  return tf_static;
}
interfaces::CompetitionVehicleStatus pitchOf(double pitch_rad) {
  interfaces::CompetitionVehicleStatus status;
  status.pitch_rad = pitch_rad;
  return status;
}
}  // namespace

TEST(RollPitchRoiLeveler, LevelsLikeRollPitchLevelerThenKeepsHeightRange) {
  const auto cloud = cloudOf({{{10, 0, 0}}, {{10, 0, 3}}, {{2, 0, -2}}});
  const auto tf_static = mountAt(1.5);
  const auto status = pitchOf(0.1);
  lidar_leveler::RollPitchLeveler leveler;
  const auto leveled = leveler.process(cloud, status, tf_static);
  lidar_leveler::RollPitchRoiLevelerParameters parameters;
  parameters.z_min_m = -0.5;
  parameters.z_max_m = 2.0;
  lidar_leveler::RollPitchRoiLeveler core(parameters);
  const auto output = core.process(cloud, status, tf_static);
  sensor_msgs::PointCloud2ConstIterator<float> all_z(leveled, "z");
  std::vector<float> expected;
  for (; all_z != all_z.end(); ++all_z)
    if (*all_z >= parameters.z_min_m && *all_z <= parameters.z_max_m) expected.push_back(*all_z);
  ASSERT_EQ(expected.size(), output.width);
  EXPECT_EQ(1u, output.height);
  sensor_msgs::PointCloud2ConstIterator<float> z(output, "z");
  for (const float value : expected) {
    EXPECT_FLOAT_EQ(value, *z);
    ++z;
  }
}

TEST(RollPitchRoiLeveler, LoaderRejectsInvertedBounds) {
  EXPECT_THROW(lidar_leveler::loadRollPitchRoiLevelerParameters(
      node_pipeline::ParamReader::fromString("{z_min_m: 2.0, z_max_m: -2.0}")), std::invalid_argument);
}
