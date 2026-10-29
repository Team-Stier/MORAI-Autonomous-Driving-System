#include <gtest/gtest.h>
#include <velocity_planner/banks/stop_line_state.hpp>
#include <velocity_planner/libraries/localization_jump.hpp>

namespace {
tf2_msgs::TFMessage at(double x, double y) {
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.rotation.w = 1.0;
  tf2_msgs::TFMessage message;
  message.transforms.push_back(transform);
  return message;
}

velocity_planner::StopLineState stopped() {
  velocity_planner::StopLineState state{velocity_planner::StopLineStateParameters{}};
  state.stopping = true;
  state.stop_line_id = 4188;
  state.green_frames = 2;
  state.last_light_stamp_ns = 123;
  return state;
}
}  // namespace

TEST(LocalizationJump, DrivingKeepsTheStop) {
  auto state = stopped();
  EXPECT_FALSE(velocity_planner::releaseStopAfterJump(state, at(0.0, 0.0), 10.0));
  EXPECT_FALSE(velocity_planner::releaseStopAfterJump(state, at(0.5, 0.0), 10.033));
  EXPECT_TRUE(state.stopping);
  EXPECT_EQ(state.stop_line_id, 4188);
}

TEST(LocalizationJump, CheckpointReturnReleasesTheStop) {
  auto state = stopped();
  velocity_planner::releaseStopAfterJump(state, at(0.0, 0.0), 10.0);
  EXPECT_TRUE(velocity_planner::releaseStopAfterJump(state, at(-60.0, -120.0), 10.033));
  EXPECT_FALSE(state.stopping);
  EXPECT_EQ(state.stop_line_id, 0);
  EXPECT_EQ(state.green_frames, 0);
  EXPECT_EQ(state.last_light_stamp_ns, 0u);
}

TEST(LocalizationJump, RewoundClockReleasesTheStop) {
  auto state = stopped();
  velocity_planner::releaseStopAfterJump(state, at(0.0, 0.0), 10.0);
  EXPECT_TRUE(velocity_planner::releaseStopAfterJump(state, at(0.0, 0.0), 5.0));
  EXPECT_FALSE(state.stopping);
}
