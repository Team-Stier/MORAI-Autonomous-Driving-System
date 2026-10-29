#include <gtest/gtest.h>
#include <visualizer/cores/estop_signal.hpp>

TEST(EstopSignal, OnIsRedText) {
  visualizer::EstopSignal core;
  std_msgs::Bool message;
  message.data = true;
  const auto marker = core.process(message);
  EXPECT_EQ(marker.text, "E-stop: ON");
  EXPECT_FLOAT_EQ(marker.color.r, 1.0f);
  EXPECT_FLOAT_EQ(marker.color.g, 0.0f);
  EXPECT_EQ(marker.type, visualization_msgs::Marker::TEXT_VIEW_FACING);
}

TEST(EstopSignal, OffIsGreyText) {
  visualizer::EstopSignal core;
  const auto marker = core.process(std_msgs::Bool{});
  EXPECT_EQ(marker.text, "E-stop: OFF");
  EXPECT_FLOAT_EQ(marker.color.r, 0.6f);
}
