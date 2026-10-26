#include <object_detector/cores/bounded_obb.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

namespace pkg = object_detector;
namespace {
pkg::Cloud box(double length, double width, double height, double yaw = 0,
               double center_x = 0, double center_y = 0) {
  pkg::Cloud cloud;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto x : {-length / 2, length / 2})
    for (const auto y : {-width / 2, width / 2})
      for (const auto z : {0.0, height})
        cloud.push_back(pcl::PointXYZ(center_x + c*x - s*y, center_y + s*x + c*y, z));
  return cloud;
}
}  // namespace

TEST(BoundedObb, RemovesLongThinStructureAndRetainsBoxPedestrianCarAndBus) {
  pkg::BoundedObb core;
  const auto output = std::get<0>(core.process({
      box(36.9, 2.32, 1.0), box(3, 2, 2), box(0.6, 0.5, 1.8),
      box(4.8, 2, 1.7), box(12, 2.5, 3.2)}, {}));
  ASSERT_EQ(4u, output.objects.size());
  EXPECT_NEAR(3, output.objects[0].bbox_size.x, 1e-6);
  EXPECT_NEAR(2, output.objects[0].bbox_size.y, 1e-6);
  EXPECT_NEAR(2, output.objects[0].bbox_size.z, 1e-6);
  EXPECT_NEAR(0.6, output.objects[1].bbox_size.x, 1e-6);
  EXPECT_NEAR(4.8, output.objects[2].bbox_size.x, 1e-6);
  EXPECT_NEAR(12, output.objects[3].bbox_size.x, 1e-6);
}

TEST(BoundedObb, FilteringIsInvariantUnderMapTranslationAndYaw) {
  pkg::BoundedObb core;
  for (const auto yaw : {0.0, 0.7, 1.5707963267948966, -2.1}) {
    const auto output = std::get<0>(core.process({
        box(36.9, 2.32, 1, yaw, 100, -200), box(4.8, 2, 1.7, yaw, 110, -190)}, {}));
    ASSERT_EQ(1u, output.objects.size());
    EXPECT_NEAR(4.8, output.objects[0].bbox_size.x, 2e-5);
    EXPECT_NEAR(2, output.objects[0].bbox_size.y, 2e-5);
    EXPECT_NEAR(110, output.objects[0].position.x, 2e-5);
    EXPECT_NEAR(-190, output.objects[0].position.y, 2e-5);
  }
}

TEST(BoundedObb, RequiresBothExcessiveLengthAndAspectRatio) {
  pkg::BoundedObb core;
  const auto output = std::get<0>(core.process({
      box(18, 1, 1), box(20, 5, 1), box(20, 6, 1), box(20, 0, 1)}, {}));
  ASSERT_EQ(3u, output.objects.size());
  EXPECT_NEAR(18, output.objects[0].bbox_size.x, 1e-6);
  EXPECT_NEAR(5, output.objects[1].bbox_size.y, 1e-6);
  EXPECT_NEAR(6, output.objects[2].bbox_size.y, 1e-6);
}

TEST(BoundedObb, PreservesObjectSemanticsAndCompetitionPayload) {
  pkg::BoundedObb core;
  interfaces::CompetitionVehicleStatus competition;
  competition.header.seq = 19;
  competition.header.stamp.fromNSec(9876543210);
  competition.header.frame_id = "base_link";
  competition.gear = 4;
  competition.velocity_x_mps = 5.5f;
  competition.link_id = "test_link";
  const auto result = core.process({box(3, 2, 2, 0.4, 10, -3)}, competition);
  ASSERT_EQ(1u, std::get<0>(result).objects.size());
  const auto& object = std::get<0>(result).objects.front();
  EXPECT_EQ(object.position, object.bbox_pose.position);
  EXPECT_EQ("unknown", object.label);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);
  EXPECT_TRUE(std::get<0>(result).header.frame_id.empty());
  EXPECT_TRUE(std::get<0>(result).header.stamp.isZero());
  const auto expected_status = ros::serialization::serializeMessage(competition);
  const auto actual_status = ros::serialization::serializeMessage(std::get<1>(result));
  ASSERT_EQ(expected_status.num_bytes, actual_status.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected_status.buf.get(), actual_status.buf.get(), expected_status.num_bytes));
  EXPECT_TRUE(std::get<0>(core.process({}, competition)).objects.empty());
}
