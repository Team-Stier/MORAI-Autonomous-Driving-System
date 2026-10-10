#include <velocity_planner/cores/stop_line_cutoff_straight.hpp>
#include <velocity_planner/cores/stop_line_cutoff_left.hpp>
#include <velocity_planner/validators/stop_line_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace {
interfaces::Path makePath(double length_m = 40.0) {
  interfaces::Path path;
  path.header.frame_id = "map";
  path.header.stamp.fromSec(10);
  for (double s = 0; s <= length_m; s += 5) {
    interfaces::PathPoint point;
    point.pose.header = path.header;
    point.pose.pose.position.x = s;
    point.pose.pose.orientation.w = 1;
    point.velocity = 10;
    point.acceleration = 0;
    path.points.push_back(point);
  }
  return path;
}

velocity_planner::StopLineStateParameters makeParameters() {
  velocity_planner::StopLineStateParameters parameters;
  parameters.x_m = 5;
  parameters.y_m = 10;
  parameters.z_mps2 = 3;
  parameters.n_frames = 3;
  parameters.stop_lines = {{2, {{{35, -1}}, {{35, 1}}}},
                           {1, {{{25, -1}}, {{25, 0}}, {{25, 1}}}}};
  return parameters;
}

interfaces::TrafficLight makeLight(std::uint8_t signal, double stamp_sec) {
  interfaces::TrafficLight light;
  light.header.frame_id = "camera_7_optical_frame";
  light.header.stamp.fromSec(stamp_sec);
  light.state = signal;
  return light;
}

interfaces::CompetitionVehicleStatus makeStatus(double speed_mps = 5) {
  interfaces::CompetitionVehicleStatus competition;
  competition.signed_velocity_mps = speed_mps;
  return competition;
}
}  // namespace

TEST(StopLineCutoff, FirstContactAndLinearProfilePreserveGeometryAndTf) {
  auto path = makePath();
  path.points[2].velocity = 12;
  velocity_planner::StopLineState state(makeParameters());
  velocity_planner::StopLineCutoffStraight core(state);
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms[0].transform.translation.x = 123;
  const auto output = core.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), tf);
  ASSERT_EQ(path.points.size(), output.first.points.size());
  EXPECT_EQ(1, state.stop_line_id);
  EXPECT_DOUBLE_EQ(10, output.first.points[0].velocity);
  EXPECT_DOUBLE_EQ(12, output.first.points[2].velocity);
  EXPECT_DOUBLE_EQ(6, output.first.points[3].velocity);
  EXPECT_DOUBLE_EQ(-7.2, output.first.points[3].acceleration);
  for (std::size_t i = 4; i < path.points.size(); ++i)
    EXPECT_DOUBLE_EQ(0, output.first.points[i].velocity);
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    EXPECT_EQ(path.points[i].pose, output.first.points[i].pose);
    EXPECT_DOUBLE_EQ(10, path.points[0].velocity);
  }
  EXPECT_EQ(path.header, output.first.header);
  EXPECT_EQ(tf, output.second);
}

TEST(StopLineCutoff, ShortApproachOverwritesEveryPointWithZero) {
  auto parameters = makeParameters();
  parameters.y_m = 21;
  velocity_planner::StopLineState state(parameters);
  velocity_planner::StopLineCutoffStraight core(state);
  const auto output = core.process(makePath(), makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  for (const auto& point : output.first.points) {
    EXPECT_DOUBLE_EQ(0, point.velocity);
    EXPECT_DOUBLE_EQ(0, point.acceleration);
  }
}

TEST(StopLineCutoff, YellowStopsOnlyWhenBrakingBeforeOffsetIsPossible) {
  for (const double speed : {5.0, 12.0}) {
    velocity_planner::StopLineState state(makeParameters());
    velocity_planner::StopLineCutoffStraight core(state);
    const auto output = core.process(makePath(), makeLight(interfaces::TrafficLight::YELLOW, 1), makeStatus(speed), {});
    EXPECT_EQ(speed == 5, state.stopping);
    EXPECT_DOUBLE_EQ(speed == 5 ? 0 : 10, output.first.points.back().velocity);
  }
}

TEST(StopLineCutoff, UnknownAndOffKeepStopAndResetGreenConfirmation) {
  velocity_planner::StopLineState state(makeParameters());
  velocity_planner::StopLineCutoffStraight core(state);
  const auto path = makePath();
  core.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  core.process(path, makeLight(interfaces::TrafficLight::GREEN, 2), makeStatus(), {});
  core.process(path, makeLight(interfaces::TrafficLight::GREEN, 3), makeStatus(), {});
  EXPECT_EQ(2, state.green_frames);
  for (const auto signal : {interfaces::TrafficLight::UNKNOWN, interfaces::TrafficLight::OFF}) {
    const auto output = core.process(path, makeLight(signal, 4 + signal), makeStatus(), {});
    EXPECT_TRUE(state.stopping);
    EXPECT_EQ(0, state.green_frames);
    EXPECT_DOUBLE_EQ(0, output.first.points.back().velocity);
  }
  for (int i = 0; i < 3; ++i) {
    const auto output = core.process(path, makeLight(interfaces::TrafficLight::GREEN, 6 + i), makeStatus(), {});
    EXPECT_EQ(i < 2, state.stopping);
    EXPECT_DOUBLE_EQ(i < 2 ? 0 : 10, output.first.points.back().velocity);
  }
}

TEST(StopLineCutoff, DuplicateAndOlderImageStampsDoNotReleaseStop) {
  velocity_planner::StopLineState state(makeParameters());
  velocity_planner::StopLineCutoffStraight core(state);
  const auto path = makePath();
  core.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  for (int i = 0; i < 10; ++i)
    core.process(path, makeLight(interfaces::TrafficLight::GREEN, 2), makeStatus(), {});
  core.process(path, makeLight(interfaces::TrafficLight::GREEN, 1.5), makeStatus(), {});
  EXPECT_TRUE(state.stopping);
  EXPECT_EQ(1, state.green_frames);
}

TEST(StopLineCutoff, DirectionalMappingsAreExplicitForAllSignalEnums) {
  const bool straight_stops[] = {false, false, true, true, false, true, true, false, true,
                                false, true, false, false, true, false, false, false};
  const bool left_stops[] = {false, false, true, true, true, false, true, true, false,
                            true, false, false, true, false, false, false, false};
  for (std::uint8_t signal = 0; signal <= 16; ++signal) {
    velocity_planner::StopLineState straight_state(makeParameters()), left_state(makeParameters());
    velocity_planner::StopLineCutoffStraight straight(straight_state);
    velocity_planner::StopLineCutoffLeft left(left_state);
    const auto path = makePath();
    straight.process(path, makeLight(signal, 1), makeStatus(), {});
    left.process(path, makeLight(signal, 1), makeStatus(), {});
    EXPECT_EQ(straight_stops[signal], straight_state.stopping) << static_cast<int>(signal);
    EXPECT_EQ(left_stops[signal], left_state.stopping) << static_cast<int>(signal);
  }
}

TEST(StopLineCutoff, SharedStopSurvivesChangingDirectionalCore) {
  velocity_planner::StopLineState state(makeParameters());
  velocity_planner::StopLineCutoffStraight straight(state);
  velocity_planner::StopLineCutoffLeft left(state);
  const auto path = makePath();
  straight.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  left.process(path, makeLight(interfaces::TrafficLight::UNKNOWN, 2), makeStatus(), {});
  EXPECT_TRUE(state.stopping);
  for (int i = 0; i < 3; ++i)
    left.process(path, makeLight(interfaces::TrafficLight::RED_LEFT, 3 + i), makeStatus(), {});
  EXPECT_FALSE(state.stopping);
}

TEST(StopLineCutoff, CollinearContactAndLostStopLineDoNotReleaseStop) {
  auto parameters = makeParameters();
  parameters.stop_lines = {{1, {{{15, 0}}, {{20, 0}}}}};
  velocity_planner::StopLineState state(parameters);
  velocity_planner::StopLineCutoffStraight core(state);
  auto path = makePath();
  core.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  EXPECT_TRUE(state.stopping);
  for (auto& point : path.points) point.pose.pose.position.y = 2;
  const auto output = core.process(path, makeLight(interfaces::TrafficLight::OFF, 2), makeStatus(), {});
  for (const auto& point : output.first.points) EXPECT_DOUBLE_EQ(0, point.velocity);
}

TEST(StopLineCutoff, DirectionChangeDoesNotMixGreenConfirmationFrames) {
  velocity_planner::StopLineState state(makeParameters());
  velocity_planner::StopLineCutoffStraight straight(state);
  velocity_planner::StopLineCutoffLeft left(state);
  const auto path = makePath();
  straight.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  straight.process(path, makeLight(interfaces::TrafficLight::GREEN, 2), makeStatus(), {});
  straight.process(path, makeLight(interfaces::TrafficLight::GREEN, 3), makeStatus(), {});
  left.process(path, makeLight(interfaces::TrafficLight::LEFT, 4), makeStatus(), {});
  EXPECT_TRUE(state.stopping);
  EXPECT_EQ(1, state.green_frames);
  left.process(path, makeLight(interfaces::TrafficLight::LEFT, 5), makeStatus(), {});
  EXPECT_TRUE(state.stopping);
  left.process(path, makeLight(interfaces::TrafficLight::LEFT, 6), makeStatus(), {});
  EXPECT_FALSE(state.stopping);
}

TEST(StopLineCutoff, MissingGeometricContactLeavesUnlatchedPathUnchanged) {
  velocity_planner::StopLineState state(makeParameters());
  velocity_planner::StopLineCutoffStraight core(state);
  auto path = makePath();
  for (auto& point : path.points) point.pose.pose.position.y = 2;
  const auto output = core.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  EXPECT_EQ(path, output.first);
  EXPECT_FALSE(state.stopping);
}

TEST(StopLineCutoff, InterpolatesTheOriginalSpeedAtProfileStart) {
  auto parameters = makeParameters();
  parameters.y_m = 12;
  velocity_planner::StopLineState state(parameters);
  velocity_planner::StopLineCutoffStraight core(state);
  auto path = makePath();
  path.points[2].velocity = 20;
  const auto output = core.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  EXPECT_DOUBLE_EQ(10, output.first.points[1].velocity);
  EXPECT_NEAR(16.0 * 10 / 12, output.first.points[2].velocity, 1e-12);
  EXPECT_NEAR(16.0 * 5 / 12, output.first.points[3].velocity, 1e-12);
}

TEST(StopLineCutoff, UsesPathArcLengthOnACurve) {
  auto parameters = makeParameters();
  parameters.y_m = 25;
  parameters.stop_lines = {{1, {{{25, 9}}, {{25, 11}}}}};
  velocity_planner::StopLineState state(parameters);
  velocity_planner::StopLineCutoffStraight core(state);
  auto path = makePath(20);
  const std::array<std::array<double, 2>, 5> positions{{
      {{0, 0}}, {{0, 10}}, {{10, 10}}, {{20, 10}}, {{30, 10}}}};
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    path.points[i].pose.pose.position.x = positions[i][0];
    path.points[i].pose.pose.position.y = positions[i][1];
  }
  const auto output = core.process(path, makeLight(interfaces::TrafficLight::RED, 1), makeStatus(), {});
  EXPECT_DOUBLE_EQ(10, output.first.points[0].velocity);
  EXPECT_DOUBLE_EQ(8, output.first.points[1].velocity);
  EXPECT_DOUBLE_EQ(4, output.first.points[2].velocity);
  EXPECT_DOUBLE_EQ(0, output.first.points[3].velocity);
}

TEST(StopLineInputs, DeclaresPathContentPrerequisites) {
  velocity_planner::StopLineInputs validator;
  node_pipeline::HealthLimits limits;
  limits.frame = "map";
  std::vector<node_pipeline::Kind> issues;
  const auto path = makePath();
  validator(path, limits, issues);
  EXPECT_TRUE(issues.empty());
  auto invalid = path;
  invalid.points.resize(1);
  validator(invalid, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::Missing, issues.front());
  invalid = path;
  invalid.points.front().velocity = std::numeric_limits<double>::quiet_NaN();
  issues.clear();
  validator(invalid, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::NaN, issues.front());
  invalid = path;
  invalid.points.front().pose.header.frame_id = "base_link";
  issues.clear();
  validator(invalid, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::FrameMismatch, issues.front());
}
