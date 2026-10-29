#include <gtest/gtest.h>
#include <velocity_planner/cores/stop_line_pass_through.hpp>

namespace {
velocity_planner::StopLineStateParameters stopLines() {
  velocity_planner::StopLineStateParameters parameters;
  parameters.intersection_speed_mps = 0.0;
  parameters.stop_lines.push_back({7294, {{{50.0, -3.0}}, {{50.0, 3.0}}}, false});
  return parameters;
}

interfaces::Path path(double origin_x_m, double stamp_sec) {
  interfaces::Path message;
  message.header.stamp = ros::Time(stamp_sec);
  for (int i = 0; i <= 20; ++i) {
    interfaces::PathPoint point;
    point.pose.pose.position.x = origin_x_m + i;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = 8.0;
    message.points.push_back(point);
  }
  return message;
}

void latch(velocity_planner::StopLineState& state) {
  state.stopping = true;
  state.stop_line_id = 7294;
  state.stopping_started_sec = 100.0;
}
}  // namespace

TEST(StopLinePassThrough, HoldsLatchedStopWhileDriving) {
  velocity_planner::StopLineState state(stopLines());
  latch(state);
  velocity_planner::StopLinePassThrough core(state);
  core.process(path(30.0, 100.0), {});
  const auto output = core.process(path(30.3, 100.033), {}).first;
  EXPECT_TRUE(state.stopping);
  EXPECT_DOUBLE_EQ(output.points[5].velocity, 0.0);
}

TEST(StopLinePassThrough, JumpReleasesLatchedStop) {
  velocity_planner::StopLineState state(stopLines());
  latch(state);
  velocity_planner::StopLinePassThrough core(state);
  core.process(path(30.0, 100.0), {});
  // Teleport back to the start, 230 m behind, on the next call.
  const auto output = core.process(path(-200.0, 100.033), {}).first;
  EXPECT_FALSE(state.stopping);
  EXPECT_EQ(state.stop_line_id, 0);
  EXPECT_DOUBLE_EQ(output.points[5].velocity, 8.0);
}

TEST(StopLinePassThrough, BackwardStampReleasesLatchedStop) {
  velocity_planner::StopLineState state(stopLines());
  latch(state);
  state.last_light_stamp_ns = 99900000000ULL;
  velocity_planner::StopLinePassThrough core(state);
  core.process(path(30.0, 100.0), {});
  core.process(path(30.0, 50.0), {});
  EXPECT_FALSE(state.stopping);
  EXPECT_EQ(state.last_light_stamp_ns, 0ULL);
}
