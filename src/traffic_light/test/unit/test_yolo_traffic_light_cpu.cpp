#include <traffic_light/parameters/yolo_traffic_light_cpu.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <iterator>

TEST(YoloTrafficLightCpu, NativeCpuRedFrameRequiresThreeVotes) {
  const auto parameters = traffic_light::loadYoloTrafficLightCpuParameters(
      node_pipeline::ParamReader::fromFile(
          NP_PACKAGE_DIR "/config/traffic_light/parameters/yolo_traffic_light_cpu.yaml", NP_PACKAGE_DIR));
  ASSERT_EQ(3u, parameters.detection.minimum_votes);
  std::ifstream input(NP_PACKAGE_DIR "/test/data/yolo_red.jpg", std::ios::binary);
  ASSERT_TRUE(input.good());
  sensor_msgs::CompressedImage frame;
  frame.format = "jpeg";
  frame.data.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  ASSERT_FALSE(frame.data.empty());

  traffic_light::YoloTrafficLightCpu core(parameters);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(frame).state);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(frame).state);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(frame).state);
}
