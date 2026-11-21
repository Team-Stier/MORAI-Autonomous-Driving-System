#include <traffic_light/parameters/yolo_traffic_light_default.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>

TEST(YoloTrafficLightDefault, NativeCudaRedFrameRequiresThreeVotes) {
  const char* enabled = std::getenv("NP_TEST_CUDA");
  if (!enabled || std::strcmp(enabled, "1") != 0) {
    GTEST_SKIP() << "Set NP_TEST_CUDA=1 on a CUDA-capable host; enabled runs never fall back";
  }
  const auto parameters = traffic_light::loadYoloTrafficLightDefaultParameters(
      node_pipeline::ParamReader::fromFile(
          NP_PACKAGE_DIR "/config/traffic_light/parameters/yolo_traffic_light_default.yaml", NP_PACKAGE_DIR));
  ASSERT_EQ(3u, parameters.detection.minimum_votes);
  std::ifstream input(NP_PACKAGE_DIR "/test/data/yolo_red.jpg", std::ios::binary);
  ASSERT_TRUE(input.good());
  sensor_msgs::CompressedImage frame;
  frame.format = "jpeg";
  frame.header.stamp = ros::Time(1, 0);
  frame.header.frame_id = "camera_7_optical_frame";
  frame.data.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  ASSERT_FALSE(frame.data.empty());

  traffic_light::YoloTrafficLightDefault core(parameters);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(frame).state);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(frame).state);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(frame).state);
}
