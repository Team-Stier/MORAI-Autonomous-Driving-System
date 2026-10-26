#include <velocity_planner/cores/scored_stop_line_straight.hpp>
#include <velocity_planner/cores/scored_stop_line_left.hpp>
#include <velocity_planner/cores/scored_stop_line_pass_through.hpp>
#include <velocity_planner/cores/stop_line_cutoff_straight.hpp>
#include <velocity_planner/cores/stop_line_cutoff_left.hpp>
#include <velocity_planner/cores/stop_line_pass_through.hpp>
#include <gtest/gtest.h>
#include <cmath>

namespace {
interfaces::Path path(double length = 40) {
  interfaces::Path result;
  result.header.frame_id = "map";
  result.header.stamp.fromSec(10);
  for (double x = 0; x <= length; x += 1) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = x;
    point.pose.pose.orientation.w = 1;
    point.velocity = 10;
    point.acceleration = 0;
    result.points.push_back(point);
  }
  return result;
}
velocity_planner::StopLineStateParameters parameters(std::int64_t id = 7294) {
  velocity_planner::StopLineStateParameters result;
  result.stop_lines = {{id, {{{25, -1}}, {{25, 1}}}}};
  return result;
}
interfaces::TrafficLight light(std::uint8_t state, double stamp) {
  interfaces::TrafficLight result;
  result.state = state;
  result.header.stamp.fromSec(stamp);
  result.header.frame_id = "camera_7_optical_frame";
  return result;
}
interfaces::CompetitionVehicleStatus status(double speed = 5) {
  interfaces::CompetitionVehicleStatus result;
  result.signed_velocity_mps = speed;
  return result;
}
}  // namespace

TEST(ScoredStopLine, AllEnumsRequireDirectionalConfirmation) {
  const bool straight_go[] = {false, false, false, false, true, false, false, true, false,
                             true, false, true, true, false, true, true, true};
  const bool left_go[] = {false, false, false, false, false, true, false, false, true,
                         false, true, true, false, true, true, true, true};
  for (std::uint8_t value = 0; value <= 16; ++value) {
    velocity_planner::StopLineState straight_state(parameters()), left_state(parameters(4356));
    velocity_planner::ScoredStopLineStraight straight(straight_state);
    velocity_planner::ScoredStopLineLeft left(left_state);
    for (int stamp = 1; stamp <= 3; ++stamp) {
      straight.process(path(), light(value, stamp), status(), {});
      left.process(path(), light(value, stamp), status(), {});
      EXPECT_EQ(!(straight_go[value] && stamp == 3), straight_state.stopping) << int(value);
      EXPECT_EQ(!(left_go[value] && stamp == 3), left_state.stopping) << int(value);
    }
  }
}

TEST(ScoredStopLine, BrakingProfileKeepsApproachAndStopsAheadOfFrontAxle) {
  auto params = parameters();
  params.x_m = 0;
  velocity_planner::StopLineState state(params);
  velocity_planner::ScoredStopLineStraight core(state);
  auto input = path();
  input.points[19].velocity = 2;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  const auto output = core.process(input, light(0, 1), status(), tf);
  EXPECT_DOUBLE_EQ(10, output.first.points.front().velocity);
  EXPECT_DOUBLE_EQ(2, output.first.points[19].velocity);
  EXPECT_NEAR(std::sqrt(3.0), output.first.points[21].velocity, 1e-12);
  EXPECT_DOUBLE_EQ(-3, output.first.points[21].acceleration);
  EXPECT_DOUBLE_EQ(0, output.first.points[22].velocity);
  EXPECT_DOUBLE_EQ(0, output.first.points[22].acceleration);
  EXPECT_EQ(tf, output.second);
  EXPECT_EQ(input.header, output.first.header);
  for (std::size_t i = 0; i < input.points.size(); ++i)
    EXPECT_EQ(input.points[i].pose, output.first.points[i].pose);
}

TEST(ScoredStopLine, UnknownStopsAtEachOfTheFourJudgedIds) {
  for (const auto id : {7294, 6254, 4188, 4356}) {
    velocity_planner::StopLineState state(parameters(id));
    velocity_planner::ScoredStopLineStraight core(state);
    const auto output = core.process(path(), light(0, 1), status(), {});
    EXPECT_TRUE(state.stopping) << id;
    EXPECT_EQ(id, state.stop_line_id);
    EXPECT_DOUBLE_EQ(0, output.first.points.back().velocity);
  }
}

TEST(ScoredStopLine, RepeatedStampCannotReleaseAndThirdFreshGoReleasesImmediately) {
  velocity_planner::StopLineState state(parameters());
  velocity_planner::ScoredStopLineStraight core(state);
  core.process(path(), light(2, 1), status(), {});
  for (int i = 0; i < 10; ++i) core.process(path(), light(4, 2), status(), {});
  EXPECT_EQ(1, state.green_frames);
  core.process(path(), light(4, 1.5), status(), {});
  EXPECT_EQ(1, state.green_frames);
  core.process(path(), light(4, 3), status(), {});
  EXPECT_TRUE(state.stopping);
  const auto output = core.process(path(), light(4, 4), status(), {});
  EXPECT_FALSE(state.stopping);
  EXPECT_EQ(path(), output.first);
  for (int stamp = 5; stamp < 20; ++stamp) core.process(path(), light(4, stamp), status(), {});
  EXPECT_EQ(3, state.green_frames);
}

TEST(ScoredStopLine, UnknownAndOffResetVotesAndTimeoutNeverReleases) {
  for (const std::uint8_t value : {0, 1, 2}) {
    velocity_planner::StopLineState state(parameters());
    velocity_planner::ScoredStopLineStraight core(state);
    core.process(path(), light(2, 1), status(), {});
    core.process(path(), light(4, 2), status(), {});
    core.process(path(), light(4, 3), status(), {});
    auto later = path();
    later.header.stamp.fromSec(500);
    const auto output = core.process(later, light(value, 4), status(), {});
    EXPECT_TRUE(state.stopping);
    EXPECT_EQ(0, state.green_frames);
    EXPECT_EQ(0, state.released_stop_line_id);
    EXPECT_DOUBLE_EQ(0, output.first.points.back().velocity);
  }
}

TEST(ScoredStopLine, YellowOnlyCreatesStopWhenBrakingIsPossible) {
  for (const double speed : {5.0, 20.0}) {
    velocity_planner::StopLineState state(parameters());
    velocity_planner::ScoredStopLineStraight core(state);
    core.process(path(), light(3, 1), status(speed), {});
    EXPECT_EQ(speed == 5, state.stopping);
  }
}

TEST(ScoredStopLine, LostContactAndPassThroughKeepScoredStopWithoutTimeout) {
  velocity_planner::StopLineState state(parameters(4356));
  velocity_planner::ScoredStopLineLeft core(state);
  velocity_planner::ScoredStopLinePassThrough pass(state);
  core.process(path(), light(2, 1), status(), {});
  auto short_path = path(10);
  short_path.header.stamp.fromSec(500);
  const auto missing = core.process(short_path, light(0, 2), status(), {});
  for (const auto& point : missing.first.points) EXPECT_DOUBLE_EQ(0, point.velocity);
  for (int stamp = 3; stamp <= 4; ++stamp) {
    const auto output = pass.process(short_path, light(5, stamp), {});
    EXPECT_TRUE(state.stopping);
    for (const auto& point : output.first.points) EXPECT_DOUBLE_EQ(0, point.velocity);
  }
  EXPECT_EQ(short_path, pass.process(short_path, light(5, 5), {}).first);
  EXPECT_FALSE(state.stopping);
}

TEST(ScoredStopLine, NewScoredTargetDoesNotInheritConfirmedGo) {
  auto params = parameters();
  params.stop_lines.push_back({6254, {{{60, -1}}, {{60, 1}}}});
  velocity_planner::StopLineState state(params);
  velocity_planner::ScoredStopLineStraight core(state);
  for (int stamp = 1; stamp <= 3; ++stamp) core.process(path(), light(4, stamp), status(), {});
  EXPECT_FALSE(state.stopping);
  auto next = path();
  for (auto& point : next.points) point.pose.pose.position.x += 35;
  core.process(next, light(4, 4), status(), {});
  EXPECT_TRUE(state.stopping);
  EXPECT_EQ(6254, state.stop_line_id);
  EXPECT_EQ(1, state.green_frames);
}

TEST(ScoredStopLine, NonScoredResultsAndSharedStateMatchLegacy) {
  for (std::uint8_t value = 0; value <= 16; ++value) {
    velocity_planner::StopLineState old_straight(parameters(123)), new_straight(parameters(123));
    velocity_planner::StopLineState old_left(parameters(123)), new_left(parameters(123));
    velocity_planner::StopLineCutoffStraight legacy_straight(old_straight);
    velocity_planner::ScoredStopLineStraight scored_straight(new_straight);
    velocity_planner::StopLineCutoffLeft legacy_left(old_left);
    velocity_planner::ScoredStopLineLeft scored_left(new_left);
    for (int stamp = 1; stamp <= 3; ++stamp) {
      EXPECT_EQ(legacy_straight.process(path(), light(value, stamp), status(), {}),
                scored_straight.process(path(), light(value, stamp), status(), {}));
      EXPECT_EQ(legacy_left.process(path(), light(value, stamp), status(), {}),
                scored_left.process(path(), light(value, stamp), status(), {}));
      EXPECT_EQ(old_straight.stopping, new_straight.stopping);
      EXPECT_EQ(old_left.stopping, new_left.stopping);
    }
  }
  velocity_planner::StopLineState old_state(parameters(123)), new_state(parameters(123));
  old_state.stopping = new_state.stopping = true;
  old_state.stop_line_id = new_state.stop_line_id = 123;
  velocity_planner::StopLinePassThrough legacy(old_state);
  velocity_planner::ScoredStopLinePassThrough scored(new_state);
  auto later = path();
  later.header.stamp.fromSec(500);
  EXPECT_EQ(legacy.process(later, {}), scored.process(later, light(0, 1), {}));
  EXPECT_FALSE(new_state.stopping);
}
