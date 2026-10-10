#include <traffic_light/cores/unknown.hpp>
#include <gtest/gtest.h>

TEST(Unknown, EmptyImageReturnsUnknown) {
  sensor_msgs::CompressedImage image;
  traffic_light::Unknown core;

  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(image).state);
}

TEST(Unknown, ImageReturnsUnknownAndLeavesHeaderForPipeline) {
  sensor_msgs::CompressedImage image;
  image.format = "jpeg";
  image.data = {1, 2, 3};
  image.header.seq = 42;
  image.header.stamp = ros::Time(10, 0);
  image.header.frame_id = "camera_7_optical_frame";
  traffic_light::Unknown core;

  const auto light = core.process(image);

  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, light.state);
  EXPECT_EQ(0u, light.header.seq);
  EXPECT_TRUE(light.header.stamp.isZero());
  EXPECT_TRUE(light.header.frame_id.empty());
}
