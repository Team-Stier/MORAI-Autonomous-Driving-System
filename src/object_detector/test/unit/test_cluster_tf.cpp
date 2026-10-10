#include <object_detector/cores/cluster_tf.hpp>
#include <object_detector/cores/obb.hpp>
#include <object_detector/cores/tracking.hpp>
#include <object_detector/validators/cluster_transform.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace pkg = object_detector;
namespace {
tf2_msgs::TFMessage transforms(double x = 100, double yaw = 1.5707963267948966) {
  tf2_msgs::TFMessage message;
  geometry_msgs::TransformStamped body;
  body.header.frame_id = "map";
  body.header.stamp = ros::Time(100);
  body.child_frame_id = "base_link";
  body.transform.rotation.x = 0.5;
  body.transform.rotation.w = std::sqrt(0.75);
  message.transforms.push_back(body);
  auto level = body;
  level.child_frame_id = "base_link_level";
  level.transform.translation.x = x;
  level.transform.translation.y = 200;
  level.transform.translation.z = 5;
  level.transform.rotation.x = 0;
  level.transform.rotation.z = std::sin(yaw/2);
  level.transform.rotation.w = std::cos(yaw/2);
  message.transforms.push_back(level);
  return message;
}
}

TEST(ClusterTf, MapsEveryPointInClusterOrderAndPreservesInput) {
  pkg::Clusters clusters(2);
  clusters[0].push_back(pcl::PointXYZ(10, -3, 2));
  clusters[0].push_back(pcl::PointXYZ(12, -2, 3));
  clusters[1].push_back(pcl::PointXYZ(0, 1, -1));
  clusters[0].header.seq = 19;
  clusters[0].header.stamp = 123456;
  clusters[0].is_dense = false;
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp = ros::Time(99);
  status.velocity_x_mps = 7;
  const auto tf = transforms();
  const auto result = pkg::ClusterTf().process(clusters, status, tf);
  const auto& output = std::get<0>(result);
  ASSERT_EQ(2u, output.size());
  ASSERT_EQ(2u, output[0].size());
  ASSERT_EQ(1u, output[1].size());
  EXPECT_FLOAT_EQ(103, output[0][0].x);
  EXPECT_FLOAT_EQ(210, output[0][0].y);
  EXPECT_FLOAT_EQ(7, output[0][0].z);
  EXPECT_FLOAT_EQ(102, output[0][1].x);
  EXPECT_FLOAT_EQ(212, output[0][1].y);
  EXPECT_FLOAT_EQ(8, output[0][1].z);
  EXPECT_FLOAT_EQ(99, output[1][0].x);
  EXPECT_FLOAT_EQ(200, output[1][0].y);
  EXPECT_FLOAT_EQ(4, output[1][0].z);
  EXPECT_EQ(clusters[0].header.seq, output[0].header.seq);
  EXPECT_EQ(clusters[0].header.stamp, output[0].header.stamp);
  EXPECT_EQ(clusters[0].width, output[0].width);
  EXPECT_EQ(clusters[0].height, output[0].height);
  EXPECT_EQ(clusters[0].is_dense, output[0].is_dense);
  EXPECT_FLOAT_EQ(10, clusters[0][0].x);
  EXPECT_EQ(status.header, std::get<1>(result).header);
  EXPECT_EQ(status.velocity_x_mps, std::get<1>(result).velocity_x_mps);
  EXPECT_TRUE(std::get<0>(pkg::ClusterTf().process({}, status, tf)).empty());
}

TEST(ClusterTf, MapClusterFittingProducesGlobalBoxWithoutChangingDimensions) {
  pkg::Cloud cloud;
  for (const auto x : {8.0f, 12.0f})
    for (const auto y : {-4.0f, -2.0f})
      for (const auto z : {1.0f, 3.0f}) cloud.push_back(pcl::PointXYZ(x, y, z));
  const auto result = pkg::ClusterTf().process({cloud}, {}, transforms());
  const auto object = std::get<0>(pkg::Obb().process(std::get<0>(result), {})).objects.front();
  EXPECT_NEAR(103, object.position.x, 1e-6);
  EXPECT_NEAR(210, object.position.y, 1e-6);
  EXPECT_NEAR(7, object.position.z, 1e-6);
  EXPECT_EQ(object.position, object.bbox_pose.position);
  EXPECT_DOUBLE_EQ(4, object.bbox_size.x);
  EXPECT_DOUBLE_EQ(2, object.bbox_size.y);
  EXPECT_DOUBLE_EQ(2, object.bbox_size.z);
  EXPECT_NEAR(std::sqrt(0.5), std::abs(object.bbox_pose.orientation.z), 1e-6);
  EXPECT_NEAR(std::sqrt(0.5), object.bbox_pose.orientation.w, 1e-6);
}

TEST(ClusterTf, MovingTurningTransformKeepsStaticMapObjectAndTrack) {
  pkg::ClusterTf conversion;
  pkg::Obb fitting;
  pkg::Tracking tracking;
  std::uint32_t id = 0;
  for (int i = 0; i <= 20; ++i) {
    const double time = i * 0.1, yaw = time * 0.4;
    auto tf = transforms(5*time, yaw);
    for (auto& transform : tf.transforms) transform.header.stamp += ros::Duration(time);
    pkg::Cloud cloud;
    cloud.push_back(pcl::PointXYZ(std::cos(yaw) * (20-5*time) + std::sin(yaw)*3,
                                -std::sin(yaw) * (20-5*time) + std::cos(yaw)*3, 2));
    const auto clusters = std::get<0>(conversion.process({cloud}, {}, tf));
    const auto boxes = std::get<0>(fitting.process(clusters, {}));
    const auto object = tracking.process(boxes, tf.transforms.front().header.stamp).objects.front();
    if (i == 0) id = object.id;
    EXPECT_EQ(id, object.id);
    EXPECT_NEAR(20, object.position.x, 2e-6);
    EXPECT_NEAR(203, object.position.y, 2e-6);
    EXPECT_NEAR(7, object.position.z, 2e-6);
    if (i == 20) {
      EXPECT_TRUE(object.velocity_available);
      EXPECT_TRUE(object.acceleration_available);
      EXPECT_NEAR(0, object.velocity.x, 1e-5);
      EXPECT_NEAR(0, object.velocity.y, 1e-5);
      EXPECT_NEAR(0, object.velocity.z, 1e-5);
    }
  }
}

TEST(ClusterTransform, RejectsMissingOrInvalidRequiredTransform) {
  const pkg::ClusterTransform validator;
  auto tf = transforms();
  std::vector<node_pipeline::Kind> issues;
  validator(tf, {}, issues);
  EXPECT_TRUE(issues.empty());
  for (int invalid = 0; invalid < 6; ++invalid) {
    tf = transforms();
    if (invalid == 0) tf.transforms.pop_back();
    if (invalid == 1) tf.transforms.back().header.frame_id = "odom";
    if (invalid == 2) tf.transforms.back().transform.rotation.w = 0;
    if (invalid == 3) tf.transforms.back().transform.translation.x = std::numeric_limits<double>::infinity();
    if (invalid == 4) tf.transforms.back().header.stamp = ros::Time(99);
    if (invalid == 5) tf.transforms.push_back(tf.transforms.back());
    issues.clear();
    validator(tf, {}, issues);
    EXPECT_FALSE(issues.empty()) << invalid;
  }
}
