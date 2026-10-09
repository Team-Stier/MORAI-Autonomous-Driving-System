#include <object_detector/parameters/dbscan.hpp>
#include <object_detector/cores/objects_pass_through.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>

namespace {
sensor_msgs::PointCloud2 cube(std::size_t count) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "base_link_level";
  cloud.header.stamp.fromNSec(1234567890);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(count);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (std::size_t i = 0; i < count; ++i, ++x, ++y, ++z) {
    *x = 4.0f + 0.08f * (i & 1);
    *y = 0.08f * ((i >> 1) & 1);
    *z = 0.6f + 0.08f * ((i >> 2) & 1);
  }
  return cloud;
}

TEST(DbscanParameters, C5RequiresEightNeighborsIncludingSelfAndPreservesInput) {
  const auto reader = node_pipeline::ParamReader::fromFile(
    std::string(NP_PACKAGE_DIR) + "/config/object_detector/parameters/dbscan.yaml", NP_PACKAGE_DIR);
  const auto parameters = object_detector::loadDbscanParameters(reader);
  EXPECT_DOUBLE_EQ(0.075, parameters.voxel_leaf_size);
  EXPECT_DOUBLE_EQ(0.16, parameters.dbscan_eps);
  EXPECT_EQ(8, parameters.dbscan_min_points);
  EXPECT_EQ(6, parameters.min_cluster_size);
  EXPECT_EQ(1000, parameters.max_cluster_size);
  EXPECT_DOUBLE_EQ(0.0, parameters.roi_x_min);
  EXPECT_DOUBLE_EQ(20.0, parameters.roi_x_max);
  EXPECT_DOUBLE_EQ(-5.0, parameters.roi_y_min);
  EXPECT_DOUBLE_EQ(5.0, parameters.roi_y_max);
  EXPECT_DOUBLE_EQ(0.2, parameters.roi_z_min);
  EXPECT_DOUBLE_EQ(2.0, parameters.roi_z_max);
  object_detector::Dbscan core(parameters);
  auto cloud = cube(8);
  const auto original = cloud;
  const auto objects = core.process(cloud);
  ASSERT_EQ(1u, objects.objects.size());
  const auto& box = objects.objects.front();
  EXPECT_NEAR(4.04, box.position.x, 1e-5);
  EXPECT_NEAR(0.04, box.position.y, 1e-5);
  EXPECT_NEAR(0.64, box.position.z, 1e-5);
  EXPECT_NEAR(0.08, box.bbox_size.x, 1e-5);
  EXPECT_NEAR(0.08, box.bbox_size.y, 1e-5);
  EXPECT_NEAR(0.08, box.bbox_size.z, 1e-5);
  EXPECT_FALSE(box.velocity_available);
  EXPECT_FALSE(box.acceleration_available);
  EXPECT_EQ(original, cloud);
  EXPECT_TRUE(core.process(cube(7)).objects.empty());
  EXPECT_EQ(objects, object_detector::ObjectsPassThrough{}.process(objects));
  EXPECT_TRUE(objects.header.stamp.isZero());
  EXPECT_TRUE(objects.header.frame_id.empty());
}

TEST(DbscanParameters, TemporaryFixedRoiExcludesPointsBeyondCurrentBounds) {
  const auto reader = node_pipeline::ParamReader::fromFile(
    std::string(NP_PACKAGE_DIR) + "/config/object_detector/parameters/dbscan.yaml", NP_PACKAGE_DIR);
  object_detector::Dbscan core(object_detector::loadDbscanParameters(reader));
  for (const auto axis : {"x", "y", "z"}) {
    auto cloud = cube(8);
    sensor_msgs::PointCloud2Iterator<float> value(cloud, axis);
    const float shift = std::string(axis) == "x" ? 21.0f : (std::string(axis) == "y" ? 6.0f : 2.1f);
    for (; value != value.end(); ++value) *value += shift;
    EXPECT_TRUE(core.process(cloud).objects.empty());
  }
}

TEST(DbscanParameters, InvalidParametersCannotSilentlyFallBackToDefaults) {
  EXPECT_THROW(object_detector::loadDbscanParameters(
    node_pipeline::ParamReader::fromString("{}")), std::invalid_argument);
  EXPECT_THROW(object_detector::loadDbscanParameters(
    node_pipeline::ParamReader::fromString("roi_x_min: wrong_type")), std::invalid_argument);
}

TEST(DbscanParameters, RejectsInvalidGeometryAndUnrepresentablePointCounts) {
  const std::string valid = "roi_x_min: 0.0\nroi_x_max: 20.0\nroi_y_min: -5.0\n"
    "roi_y_max: 5.0\nroi_z_min: 0.2\nroi_z_max: 2.0\nvoxel_leaf_size: 0.075\n"
    "dbscan_eps: 0.16\ndbscan_min_points: 8\nmin_cluster_size: 6\nmax_cluster_size: 1000\n";
  const std::vector<std::pair<std::string, std::string>> cases{
    {"roi_x_min: 0.0", "roi_x_min: 21.0"},
    {"voxel_leaf_size: 0.075", "voxel_leaf_size: 0.0"},
    {"dbscan_eps: 0.16", "dbscan_eps: .nan"},
    {"dbscan_min_points: 8", "dbscan_min_points: 0"},
    {"dbscan_min_points: 8", "dbscan_min_points: 2147483648"},
    {"max_cluster_size: 1000", "max_cluster_size: 5"}};
  for (const auto& value : cases) {
    auto yaml = valid;
    yaml.replace(yaml.find(value.first), value.first.size(), value.second);
    EXPECT_THROW(object_detector::loadDbscanParameters(
      node_pipeline::ParamReader::fromString(yaml)), std::invalid_argument) << value.second;
  }
}
}  // namespace
