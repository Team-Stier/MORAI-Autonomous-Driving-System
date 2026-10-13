#include <visualizer/cores/traffic_signal.hpp>
#include <gtest/gtest.h>
#include <array>
#include <cstdint>

TEST(TrafficSignal, EnumCombinationsHaveDistinctLabelsAndPriorityColors) {
  struct Expected {
    uint8_t state;
    const char* text;
    float red;
    float green;
    float blue;
  };
  const std::array<Expected, 17> cases{{
      {interfaces::TrafficLight::UNKNOWN, "Traffic light: UNKNOWN", 0.6f, 0.6f, 0.6f},
      {interfaces::TrafficLight::OFF, "Traffic light: OFF", 0.6f, 0.6f, 0.6f},
      {interfaces::TrafficLight::RED, "Traffic light: RED", 1, 0, 0},
      {interfaces::TrafficLight::YELLOW, "Traffic light: YELLOW", 1, 1, 0},
      {interfaces::TrafficLight::GREEN, "Traffic light: GREEN", 0, 1, 0},
      {interfaces::TrafficLight::LEFT, "Traffic light: LEFT", 0, 1, 0},
      {interfaces::TrafficLight::RED_YELLOW, "Traffic light: RED + YELLOW", 1, 0, 0},
      {interfaces::TrafficLight::RED_GREEN, "Traffic light: RED + GREEN", 1, 0, 0},
      {interfaces::TrafficLight::RED_LEFT, "Traffic light: RED + LEFT", 1, 0, 0},
      {interfaces::TrafficLight::YELLOW_GREEN, "Traffic light: YELLOW + GREEN", 1, 1, 0},
      {interfaces::TrafficLight::YELLOW_LEFT, "Traffic light: YELLOW + LEFT", 1, 1, 0},
      {interfaces::TrafficLight::GREEN_LEFT, "Traffic light: GREEN + LEFT", 0, 1, 0},
      {interfaces::TrafficLight::RED_YELLOW_GREEN, "Traffic light: RED + YELLOW + GREEN", 1, 0, 0},
      {interfaces::TrafficLight::RED_YELLOW_LEFT, "Traffic light: RED + YELLOW + LEFT", 1, 0, 0},
      {interfaces::TrafficLight::RED_GREEN_LEFT, "Traffic light: RED + GREEN + LEFT", 1, 0, 0},
      {interfaces::TrafficLight::YELLOW_GREEN_LEFT, "Traffic light: YELLOW + GREEN + LEFT", 1, 1, 0},
      {interfaces::TrafficLight::RED_YELLOW_GREEN_LEFT, "Traffic light: RED + YELLOW + GREEN + LEFT", 1, 0, 0},
  }};
  visualizer::TrafficSignal core;
  interfaces::TrafficLight message;
  for (const auto& expected : cases) {
    SCOPED_TRACE(static_cast<unsigned>(expected.state));
    message.state = expected.state;
    const auto marker = core.process(message);
    EXPECT_EQ(expected.text, marker.text);
    EXPECT_FLOAT_EQ(expected.red, marker.color.r);
    EXPECT_FLOAT_EQ(expected.green, marker.color.g);
    EXPECT_FLOAT_EQ(expected.blue, marker.color.b);
    EXPECT_FLOAT_EQ(1, marker.color.a);
  }
}

TEST(TrafficSignal, TextFollowsVehicleAndLeavesHeaderForPipeline) {
  interfaces::TrafficLight message;
  message.header.frame_id = "camera_7_optical_frame";
  message.header.stamp = ros::Time(123, 456);
  message.state = interfaces::TrafficLight::RED_LEFT;
  const auto marker = visualizer::TrafficSignal().process(message);

  EXPECT_EQ("traffic_light", marker.ns);
  EXPECT_EQ(0, marker.id);
  EXPECT_EQ(visualization_msgs::Marker::TEXT_VIEW_FACING, marker.type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, marker.action);
  EXPECT_DOUBLE_EQ(8, marker.pose.position.x);
  EXPECT_DOUBLE_EQ(0, marker.pose.position.y);
  EXPECT_DOUBLE_EQ(3, marker.pose.position.z);
  EXPECT_DOUBLE_EQ(0, marker.pose.orientation.x);
  EXPECT_DOUBLE_EQ(0, marker.pose.orientation.y);
  EXPECT_DOUBLE_EQ(0, marker.pose.orientation.z);
  EXPECT_DOUBLE_EQ(1, marker.pose.orientation.w);
  EXPECT_DOUBLE_EQ(1, marker.scale.z);
  EXPECT_TRUE(marker.frame_locked);
  EXPECT_DOUBLE_EQ(1, marker.lifetime.toSec());
  EXPECT_TRUE(marker.header.frame_id.empty());
  EXPECT_TRUE(marker.header.stamp.isZero());
  EXPECT_TRUE(marker.points.empty());
}
