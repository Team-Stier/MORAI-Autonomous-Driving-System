#include <velocity_planner/cores/stop_line_pass_through.hpp>
#include <velocity_planner/cores/stop_line_cutoff_straight.hpp>
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

TEST(StopLinePassThrough, ReleasesTheSharedStopAtTimeoutAfterTheSectionChanges) {
  velocity_planner::StopLineStateParameters parameters;
  parameters.stop_lines = {{1, {{{25, -1}}, {{25, 1}}}}};
  velocity_planner::StopLineState state(parameters);
  velocity_planner::StopLineCutoffStraight straight(state);
  velocity_planner::StopLinePassThrough pass(state);
  interfaces::Path path;
  path.header.stamp.fromSec(10);
  path.points.resize(2);
  path.points.back().pose.pose.position.x = 40;
  for (auto& point : path.points) {
    point.velocity = 10;
    point.acceleration = 1;
  }
  interfaces::TrafficLight red;
  red.state = interfaces::TrafficLight::RED;
  red.header.stamp.fromSec(1);
  interfaces::CompetitionVehicleStatus competition;
  straight.process(path, red, competition, {});
  ASSERT_TRUE(state.stopping);
  path.header.stamp.fromSec(109.999);
  EXPECT_DOUBLE_EQ(0, pass.process(path, {}).first.points.front().velocity);
  EXPECT_TRUE(state.stopping);
  path.header.stamp.fromSec(110);
  EXPECT_EQ(path, pass.process(path, {}).first);
  EXPECT_FALSE(state.stopping);
  EXPECT_EQ(1, state.released_stop_line_id);
  path.header.stamp.fromSec(111);
  EXPECT_EQ(path, straight.process(path, red, competition, {}).first);
  EXPECT_FALSE(state.stopping);
}
