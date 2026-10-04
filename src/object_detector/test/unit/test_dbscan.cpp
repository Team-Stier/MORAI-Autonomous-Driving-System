// First acceptance tests for the planned DBSCAN core.
// Not registered in CMake yet: dbscan.hpp and ROS/PCL build verification are pending.
#include <object_detector/cores/dbscan.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <vector>

namespace {
using Center = std::array<float, 3>;

sensor_msgs::PointCloud2 makeCloud(const std::vector<Center>& centers) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp.fromSec(12.0);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(27 * centers.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& center : centers) {
    for (int ix = -1; ix <= 1; ++ix) {
      for (int iy = -1; iy <= 1; ++iy) {
        for (int iz = -1; iz <= 1; ++iz, ++x, ++y, ++z) {
          *x = center[0] + 0.1f * ix;
          *y = center[1] + 0.1f * iy;
          *z = center[2] + 0.1f * iz;
        }
      }
    }
  }
  return cloud;
}

object_detector::DbscanParameters testParameters() {
  object_detector::DbscanParameters p;
  // Synthetic test settings, not tuned competition defaults.
  p.roi_x_min = 0.0;
  p.roi_x_max = 20.0;
  p.roi_y_min = -5.0;
  p.roi_y_max = 5.0;
  p.roi_z_min = 0.2;
  p.roi_z_max = 2.0;
  p.voxel_leaf_size = 0.04;
  p.dbscan_eps = 0.18;
  p.dbscan_min_points = 4;  // Includes the query point itself.
  p.min_cluster_size = 5;
  p.max_cluster_size = 1000;
  return p;
}

void expectBox(const interfaces::Object& object, const Center& center) {
  EXPECT_NEAR(center[0], object.bbox_pose.position.x, 1e-4);
  EXPECT_NEAR(center[1], object.bbox_pose.position.y, 1e-4);
  EXPECT_NEAR(center[2], object.bbox_pose.position.z, 1e-4);
  EXPECT_NEAR(0.2, object.bbox_size.x, 1e-4);
  EXPECT_NEAR(0.2, object.bbox_size.y, 1e-4);
  EXPECT_NEAR(0.2, object.bbox_size.z, 1e-4);
  EXPECT_NEAR(object.bbox_pose.position.x, object.position.x, 1e-6);
  EXPECT_NEAR(object.bbox_pose.position.y, object.position.y, 1e-6);
  EXPECT_NEAR(object.bbox_pose.position.z, object.position.z, 1e-6);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);
}
}  // namespace

TEST(Dbscan, KeepsOneObjectAsOneCluster) {
  const Center center{{4.0f, 0.0f, 0.6f}};
  object_detector::Dbscan core(testParameters());
  const auto result = core.process(makeCloud({center}));
  ASSERT_EQ(1u, result.objects.size());
  expectBox(result.objects.front(), center);
}

TEST(Dbscan, SeparatesThreeDistantObjects) {
  const std::vector<Center> centers{
    {{4.0f, -2.0f, 0.6f}}, {{7.0f, 0.0f, 0.6f}}, {{10.0f, 2.0f, 0.6f}}};
  object_detector::Dbscan core(testParameters());
  auto result = core.process(makeCloud(centers));
  ASSERT_EQ(3u, result.objects.size());
  // Cluster enumeration order is not an object identity contract.
  std::sort(result.objects.begin(), result.objects.end(),
    [](const interfaces::Object& a, const interfaces::Object& b) {
      return a.position.x < b.position.x;
    });
  for (std::size_t i = 0; i < centers.size(); ++i) expectBox(result.objects[i], centers[i]);
}
