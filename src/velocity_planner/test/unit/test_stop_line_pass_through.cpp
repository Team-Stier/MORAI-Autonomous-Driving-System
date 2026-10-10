#include <velocity_planner/cores/stop_line_pass_through.hpp>
#include <gtest/gtest.h>

TEST(StopLinePassThrough, KeepsTheMapPathAndTfOutsideSignalMissions) {
  velocity_planner::StopLineState state({});
  velocity_planner::StopLinePassThrough core(state);
  interfaces::Path path;
  path.header.frame_id = "map";
  path.points.resize(1);
  path.points.front().velocity = 10;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  const auto output = core.process(path, tf);
  EXPECT_EQ(path, output.first);
  EXPECT_EQ(tf, output.second);
}

TEST(StopLinePassThrough, KeepsAnExistingStopWhenTheSectionChanges) {
  velocity_planner::StopLineState state({});
  state.stopping = true;
  state.green_frames = 2;
  velocity_planner::StopLinePassThrough core(state);
  interfaces::Path path;
  path.points.resize(2);
  for (auto& point : path.points) {
    point.velocity = 10;
    point.acceleration = 1;
  }
  const auto output = core.process(path, {});
  for (const auto& point : output.first.points) {
    EXPECT_DOUBLE_EQ(0, point.velocity);
    EXPECT_DOUBLE_EQ(0, point.acceleration);
  }
  EXPECT_TRUE(state.stopping);
  EXPECT_EQ(0, state.green_frames);
}
