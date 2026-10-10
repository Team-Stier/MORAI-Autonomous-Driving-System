#include <tf_publisher/cores/ego_tf.hpp>
#include <tf_publisher/parameters/sensor_mounts.hpp>
#include <gtest/gtest.h>
#include <cmath>

TEST(EgoTf, PublishesYawOnlyLeveledFrameAlongsideTheBodyFrame) {
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 3;
  ego.pose.pose.position.y = 4;
  ego.pose.pose.position.z = 5;
  const double roll = 0.4, pitch = -0.3, yaw = 0.8;
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  auto& q = ego.pose.pose.orientation;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  const auto message = tf_publisher::EgoTf{}.process(ego);
  ASSERT_EQ(2u, message.transforms.size());
  EXPECT_EQ("base_link", message.transforms[0].child_frame_id);
  EXPECT_EQ(ego.pose.pose.orientation, message.transforms[0].transform.rotation);
  const auto& level = message.transforms[1];
  EXPECT_EQ("base_link_level", level.child_frame_id);
  EXPECT_DOUBLE_EQ(3, level.transform.translation.x);
  EXPECT_DOUBLE_EQ(4, level.transform.translation.y);
  EXPECT_DOUBLE_EQ(5, level.transform.translation.z);
  EXPECT_DOUBLE_EQ(0, level.transform.rotation.x);
  EXPECT_DOUBLE_EQ(0, level.transform.rotation.y);
  EXPECT_NEAR(sy, level.transform.rotation.z, 1e-12);
  EXPECT_NEAR(cy, level.transform.rotation.w, 1e-12);
}

TEST(EgoTf, CopiesEgoPoseIntoBaseLinkTransform) {
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 1;
  ego.pose.pose.position.y = 2;
  ego.pose.pose.orientation.z = 1;
  tf_publisher::EgoTf core;
  const auto message = core.process(ego);
  ASSERT_EQ(2u, message.transforms.size());
  EXPECT_EQ("base_link", message.transforms[0].child_frame_id);
  EXPECT_DOUBLE_EQ(2, message.transforms[0].transform.translation.y);
  EXPECT_DOUBLE_EQ(1, message.transforms[0].transform.rotation.z);
  EXPECT_TRUE(message.transforms[0].header.frame_id.empty());
}

TEST(SensorMounts, LoadsConfiguredMountsInOrderAndRejectsInvalidRotation) {
  const auto parameters = tf_publisher::loadSensorMountsParameters(node_pipeline::ParamReader::fromString(
      "mounts:\n  gps_link: {translation: [0, 0, 1.2], rotation: [0, 0, 0, 1]}\n"
      "  velodyne: {translation: [1.5, 0, 1.403], rotation: [0, 0, 0, 1]}\n"));
  tf_publisher::SensorMounts core(parameters);
  const auto message = core.process();
  ASSERT_EQ(2u, message.transforms.size());
  EXPECT_EQ("gps_link", message.transforms[0].child_frame_id);
  EXPECT_EQ("velodyne", message.transforms[1].child_frame_id);
  EXPECT_DOUBLE_EQ(1.403, message.transforms[1].transform.translation.z);
  EXPECT_THROW(tf_publisher::loadSensorMountsParameters(node_pipeline::ParamReader::fromString(
      "mounts:\n  gps_link: {translation: [0, 0, 1], rotation: [0, 0, 0, 2]}\n")), std::invalid_argument);
  EXPECT_THROW(tf_publisher::loadSensorMountsParameters(node_pipeline::ParamReader::fromString(
      "mounts:\n  base_link: {translation: [0, 0, 1], rotation: [0, 0, 0, 1]}\n")), std::invalid_argument);
}
