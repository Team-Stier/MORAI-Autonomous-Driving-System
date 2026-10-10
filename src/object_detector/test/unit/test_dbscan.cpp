#include <object_detector/cores/dbscan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <initializer_list>
#include <tuple>
#include <vector>

namespace {
sensor_msgs::PointCloud2 cloud(std::initializer_list<float> xs) {
  pcl::PointCloud<pcl::PointXYZ> points;
  for (const auto x : xs) points.push_back(pcl::PointXYZ(x, 0.0f, 1.0f));
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(points, message);
  return message;
}
}  // namespace

TEST(Dbscan, SeparatesClustersAndExcludesNoise) {
  object_detector::Dbscan core({0.001, 0.11, 3, 3, 3});
  interfaces::CompetitionVehicleStatus competition;
  competition.header.frame_id = "base_link";
  competition.header.stamp.fromNSec(1234567890);
  competition.velocity_x_mps = 4.5;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms.front().header.frame_id = "map";
  tf.transforms.front().header.stamp.fromNSec(1234567890);
  tf.transforms.front().child_frame_id = "base_link_level";
  tf.transforms.front().transform.translation.x = 123;
  const auto result = core.process(cloud({0.0f, 0.05f, 0.1f, 5.0f, 5.05f, 5.1f, 9.0f}), competition, tf);
  const auto& clusters = std::get<0>(result);
  ASSERT_EQ(2u, clusters.size());
  std::vector<float> centers;
  for (const auto& cluster : clusters) {
    ASSERT_EQ(3u, cluster.size());
    float center = 0.0f;
    for (const auto& point : cluster) center += point.x / 3.0f;
    centers.push_back(center);
  }
  std::sort(centers.begin(), centers.end());
  EXPECT_NEAR(0.05f, centers[0], 1e-5);
  EXPECT_NEAR(5.05f, centers[1], 1e-5);
  EXPECT_EQ(competition.header, std::get<1>(result).header);
  EXPECT_EQ(competition.velocity_x_mps, std::get<1>(result).velocity_x_mps);
  EXPECT_EQ(tf.transforms, std::get<2>(result).transforms);
}

TEST(Dbscan, IncludesSelfAndPromotesEarlierNoiseToBorderWithoutDuplicates) {
  object_detector::Dbscan core({0.001, 0.1, 3, 3, 3});
  const auto result = core.process(cloud({0.0f, 0.09f, 0.18f}), {}, {});
  const auto& clusters = std::get<0>(result);
  ASSERT_EQ(1u, clusters.size());
  std::vector<float> xs;
  for (const auto& point : clusters.front()) xs.push_back(point.x);
  std::sort(xs.begin(), xs.end());
  EXPECT_EQ((std::vector<float>{0.0f, 0.09f, 0.18f}), xs);
}

TEST(Dbscan, FiltersClusterSizesBeforeReturningClouds) {
  object_detector::Dbscan core({0.001, 0.4, 2, 4, 5});
  const auto result = core.process(cloud({0.0f, 0.05f, 0.1f,
                                        5.0f, 5.05f, 5.1f, 5.15f,
                                        9.0f, 9.05f, 9.1f, 9.15f, 9.2f, 9.25f}), {}, {});
  const auto& clusters = std::get<0>(result);
  ASSERT_EQ(1u, clusters.size());
  ASSERT_EQ(4u, clusters.front().size());
  for (const auto& point : clusters.front()) {
    EXPECT_GE(point.x, 5.0f);
    EXPECT_LE(point.x, 5.15f);
  }
}

TEST(Dbscan, DownsamplesBeforeDensitySearchAndAcceptsEmptyCloud) {
  object_detector::Dbscan core({0.01, 0.1, 3, 1, 100});
  const auto result = core.process(cloud({1.0f, 1.001f, 2.0f, 2.001f}), {}, {});
  EXPECT_TRUE(std::get<0>(result).empty());
  object_detector::Dbscan sparse_core({0.01, 0.1, 1, 1, 100});
  const auto sparse = sparse_core.process(cloud({1.0f, 1.001f, 2.0f, 2.001f}), {}, {});
  ASSERT_EQ(2u, std::get<0>(sparse).size());
  for (const auto& cluster : std::get<0>(sparse)) EXPECT_EQ(1u, cluster.size());
  const auto empty = core.process(cloud({}), {}, {});
  EXPECT_TRUE(std::get<0>(empty).empty());
}
