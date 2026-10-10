#include <object_detector/cores/obb.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

namespace pkg = object_detector;

TEST(Obb, FitsRotatedRectangleAndPreservesObjectSemantics) {
  pkg::Cloud cloud;
  const double yaw_rad = 0.4;
  const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
  for (const auto x : {-2.0, 2.0})
    for (const auto y : {-1.0, 1.0})
      for (const auto z : {1.0, 3.0})
        cloud.push_back(pcl::PointXYZ(10.0+c*x-s*y, -3.0+s*x+c*y, z));
  pkg::Obb core;

  const auto output = std::get<0>(core.process({cloud}, {}));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects.front();
  EXPECT_NEAR(10.0, object.position.x, 1e-6);
  EXPECT_NEAR(-3.0, object.position.y, 1e-6);
  EXPECT_DOUBLE_EQ(2.0, object.position.z);
  EXPECT_EQ(object.position, object.bbox_pose.position);
  EXPECT_NEAR(4.0, object.bbox_size.x, 1e-6);
  EXPECT_NEAR(2.0, object.bbox_size.y, 1e-6);
  EXPECT_DOUBLE_EQ(2.0, object.bbox_size.z);
  EXPECT_DOUBLE_EQ(0.0, object.bbox_pose.orientation.x);
  EXPECT_DOUBLE_EQ(0.0, object.bbox_pose.orientation.y);
  EXPECT_NEAR(std::sin(yaw_rad/2.0), object.bbox_pose.orientation.z, 1e-6);
  EXPECT_NEAR(std::cos(yaw_rad/2.0), object.bbox_pose.orientation.w, 1e-6);
  EXPECT_EQ("unknown", object.label);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);
  EXPECT_EQ(geometry_msgs::Vector3{}, object.velocity);
  EXPECT_EQ(geometry_msgs::Vector3{}, object.acceleration);
  EXPECT_TRUE(output.header.frame_id.empty());
  EXPECT_TRUE(output.header.stamp.isZero());
}

TEST(Obb, FitsCollinearAndSingleXyClusters) {
  pkg::Cloud line_cloud, single_xy_cloud;
  for (const auto& point : {pcl::PointXYZ(0, 0, 1), pcl::PointXYZ(2, 2, 3),
                            pcl::PointXYZ(4, 4, 2)})
    line_cloud.push_back(point);
  single_xy_cloud.push_back(pcl::PointXYZ(7, -2, -1));
  single_xy_cloud.push_back(pcl::PointXYZ(7, -2, 4));
  pkg::Obb core;

  const auto output = std::get<0>(core.process({line_cloud, single_xy_cloud}, {}));
  ASSERT_EQ(2u, output.objects.size());
  const auto& line = output.objects[0];
  EXPECT_DOUBLE_EQ(2.0, line.position.x);
  EXPECT_DOUBLE_EQ(2.0, line.position.y);
  EXPECT_DOUBLE_EQ(2.0, line.position.z);
  EXPECT_NEAR(std::sqrt(32.0), line.bbox_size.x, 1e-12);
  EXPECT_DOUBLE_EQ(0.0, line.bbox_size.y);
  EXPECT_DOUBLE_EQ(2.0, line.bbox_size.z);
  const double yaw_rad = std::atan2(4.0, 4.0);
  EXPECT_NEAR(std::sin(yaw_rad/2.0), line.bbox_pose.orientation.z, 1e-12);
  EXPECT_NEAR(std::cos(yaw_rad/2.0), line.bbox_pose.orientation.w, 1e-12);
  const auto& point = output.objects[1];
  EXPECT_DOUBLE_EQ(7.0, point.position.x);
  EXPECT_DOUBLE_EQ(-2.0, point.position.y);
  EXPECT_DOUBLE_EQ(1.5, point.position.z);
  EXPECT_DOUBLE_EQ(0.0, point.bbox_size.x);
  EXPECT_DOUBLE_EQ(0.0, point.bbox_size.y);
  EXPECT_DOUBLE_EQ(5.0, point.bbox_size.z);
  EXPECT_DOUBLE_EQ(0.0, point.bbox_pose.orientation.z);
  EXPECT_DOUBLE_EQ(1.0, point.bbox_pose.orientation.w);
}

TEST(Obb, ReturnsEmptyObjectsAndPassesCompetitionThrough) {
  interfaces::CompetitionVehicleStatus competition;
  competition.header.seq = 19;
  competition.header.stamp.fromNSec(9876543210);
  competition.header.frame_id = "base_link";
  competition.gear = 4;
  competition.velocity_x_mps = 5.5f;
  competition.link_id = "test_link";
  pkg::Obb core;

  const auto result = core.process({}, competition);
  EXPECT_TRUE(std::get<0>(result).objects.empty());
  const auto expected_status = ros::serialization::serializeMessage(competition);
  const auto actual_status = ros::serialization::serializeMessage(std::get<1>(result));
  ASSERT_EQ(expected_status.num_bytes, actual_status.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected_status.buf.get(), actual_status.buf.get(), expected_status.num_bytes));
}
