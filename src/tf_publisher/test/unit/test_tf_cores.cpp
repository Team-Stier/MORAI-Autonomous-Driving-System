#include <tf_publisher/cores/ego_tf.hpp>
#include <tf_publisher/parameters/sensor_mounts.hpp>
#include <gtest/gtest.h>

TEST(EgoTf, CopiesEgoPoseIntoBaseLinkTransform) {
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 1;
  ego.pose.pose.position.y = 2;
  ego.pose.pose.orientation.z = 1;
  tf_publisher::EgoTf core;
  const auto message = core.process(ego);
  ASSERT_EQ(1u, message.transforms.size());
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
