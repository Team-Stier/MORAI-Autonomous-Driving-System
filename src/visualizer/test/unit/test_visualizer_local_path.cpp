#include <visualizer/cores/local_path.hpp>
#include <gtest/gtest.h>
#include <array>

TEST(LocalPath, UsesBaseLinkPointsAndClampsSpeedColors) {
  interfaces::Path input;
  input.header.frame_id = "base_link";
  const std::array<double, 8> speeds{{-5, 0, 12.5, 25, 50, 75, 100, 125}};
  const std::array<std::array<double, 3>, 8> colors{{
      {{1, 0, 0}}, {{1, 0, 0}}, {{1, 0.5, 0}}, {{1, 1, 0}},
      {{0, 1, 0}}, {{0, 1, 1}}, {{0, 0, 1}}, {{0, 0, 1}}}};
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    interfaces::PathPoint point;
    point.pose.header = input.header;
    point.pose.pose.position.x = 5 + i;
    point.pose.pose.position.y = -2;
    point.pose.pose.position.z = 1.5;
    point.velocity = speeds[i] / 3.6;
    input.points.push_back(point);
  }
  visualizer::LocalPath core;
  const auto output = core.process(input);
  ASSERT_EQ(speeds.size(), output.points.size());
  ASSERT_EQ(speeds.size(), output.colors.size());
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    EXPECT_DOUBLE_EQ(5 + i, output.points[i].x);
    EXPECT_DOUBLE_EQ(-2, output.points[i].y);
    EXPECT_DOUBLE_EQ(0.1, output.points[i].z);
    EXPECT_NEAR(colors[i][0], output.colors[i].r, 1e-6);
    EXPECT_NEAR(colors[i][1], output.colors[i].g, 1e-6);
    EXPECT_NEAR(colors[i][2], output.colors[i].b, 1e-6);
    EXPECT_FLOAT_EQ(1, output.colors[i].a);
  }
  EXPECT_TRUE(output.frame_locked);
}

TEST(LocalPath, PreservesHeightWhenNotFlattenedAndClearsEmptyPath) {
  visualizer::LocalPath core({false});
  interfaces::Path input;
  interfaces::PathPoint point;
  point.pose.pose.position.z = 1.5;
  input.points.push_back(point);
  EXPECT_DOUBLE_EQ(1.6, core.process(input).points.front().z);
  input.points.clear();
  const auto output = core.process(input);
  EXPECT_EQ(visualization_msgs::Marker::DELETE, output.action);
  EXPECT_TRUE(output.points.empty());
  EXPECT_TRUE(output.colors.empty());
}
