#include <gtest/gtest.h>
#include <velocity_planner/cores/integrated_stop_dynamic.hpp>
#include <cmath>

namespace {
velocity_planner::StopLineStateParameters stopLines() {
  velocity_planner::StopLineStateParameters parameters;
  parameters.x_m = 5.0;
  parameters.y_m = 100.0;
  parameters.z_mps2 = 3.0;
  parameters.stop_response_time_sec = 0.35;
  parameters.stop_position_tolerance_m = 0.2;
  parameters.intersection_speed_mps = 0.0;
  parameters.stop_lines.push_back({7294, {{{50.0, -3.0}}, {{50.0, 3.0}}}, false});
  return parameters;
}

// Spatial path along +x from the rear axle at x=0, 1 m spacing, 40 km/h; stop line at x=50.
interfaces::Path approach() {
  interfaces::Path path;
  path.header.stamp = ros::Time(100.0);
  for (int i = 0; i <= 100; ++i) {
    interfaces::PathPoint point;
    point.pose.pose.position.x = i;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = 11.1;
    path.points.push_back(point);
  }
  return path;
}

interfaces::TrafficLight light(std::uint8_t state, double stamp_sec) {
  interfaces::TrafficLight message;
  message.state = state;
  message.header.stamp = ros::Time(stamp_sec);
  return message;
}

double creep(double detection_m, double offset_m) {
  const double response = 3.0 * 0.35, braking = 2 * 3.0 * (detection_m - offset_m);
  return braking / (std::sqrt(response * response + braking) + response);
}

velocity_planner::IntegratedStopDynamicParameters blind() {
  velocity_planner::IntegratedStopDynamicParameters parameters;
  parameters.blind_detection_distance_m = 9.0;
  parameters.blind_deceleration_mps2 = 2.5;
  return parameters;
}
}  // namespace

TEST(IntegratedStopDynamic, LimitsUnobservedApproach) {
  velocity_planner::StopLineState state(stopLines());
  velocity_planner::IntegratedStopDynamic core(blind(), state);
  const auto output = core.process(approach(), light(interfaces::TrafficLight::UNKNOWN, 0.0), {}, {}).first;
  const double creep_mps = creep(9.0, 5.0);
  EXPECT_DOUBLE_EQ(output.points[0].velocity, 11.1);
  EXPECT_NEAR(output.points[30].velocity, std::sqrt(creep_mps * creep_mps + 2 * 2.5 * 11.0), 1e-9);
  EXPECT_NEAR(output.points[45].velocity, creep_mps, 1e-9);
  EXPECT_DOUBLE_EQ(output.points[60].velocity, 11.1);
  EXPECT_FALSE(state.stopping);
}

TEST(IntegratedStopDynamic, ObservedGreenLiftsLimitForThatLine) {
  velocity_planner::StopLineState state(stopLines());
  velocity_planner::IntegratedStopDynamic core(blind(), state);
  auto output = core.process(approach(), light(interfaces::TrafficLight::GREEN, 99.9), {}, {}).first;
  EXPECT_DOUBLE_EQ(output.points[45].velocity, 11.1);
  // The light leaves the camera view near the line; the observed GO still holds.
  output = core.process(approach(), light(interfaces::TrafficLight::UNKNOWN, 0.0), {}, {}).first;
  EXPECT_DOUBLE_EQ(output.points[45].velocity, 11.1);
}

TEST(IntegratedStopDynamic, RedStillStopsAtTarget) {
  velocity_planner::StopLineState state(stopLines());
  velocity_planner::IntegratedStopDynamic core(blind(), state);
  const auto output = core.process(approach(), light(interfaces::TrafficLight::RED, 99.9), {}, {}).first;
  EXPECT_TRUE(state.stopping);
  EXPECT_DOUBLE_EQ(output.points[45].velocity, 0.0);
  EXPECT_DOUBLE_EQ(output.points[60].velocity, 0.0);
}

TEST(IntegratedStopDynamic, DisabledKeepsApproachSpeed) {
  velocity_planner::StopLineState state(stopLines());
  auto parameters = blind();
  parameters.blind_detection_distance_m = 0.0;
  velocity_planner::IntegratedStopDynamic core(parameters, state);
  const auto output = core.process(approach(), light(interfaces::TrafficLight::UNKNOWN, 0.0), {}, {}).first;
  EXPECT_DOUBLE_EQ(output.points[45].velocity, 11.1);
}

TEST(IntegratedStopDynamic, JumpReleasesLatchedStop) {
  velocity_planner::StopLineState state(stopLines());
  velocity_planner::IntegratedStopDynamic core(blind(), state);
  core.process(approach(), light(interfaces::TrafficLight::RED, 99.9), {}, {});
  ASSERT_TRUE(state.stopping);
  // Checkpoint return 150 m behind: the latched line is no longer on the path.
  auto back = approach();
  back.header.stamp = ros::Time(100.033);
  for (auto& point : back.points) point.pose.pose.position.x -= 150.0;
  const auto output = core.process(back, light(interfaces::TrafficLight::UNKNOWN, 0.0), {}, {}).first;
  EXPECT_FALSE(state.stopping);
  EXPECT_DOUBLE_EQ(output.points[0].velocity, 11.1);
}

TEST(IntegratedStopDynamic, JumpClearsObservedGo) {
  velocity_planner::StopLineState state(stopLines());
  velocity_planner::IntegratedStopDynamic core(blind(), state);
  core.process(approach(), light(interfaces::TrafficLight::GREEN, 99.9), {}, {});
  // Checkpoint return 20 m behind on the same approach: the GO seen before it no longer lifts the limit.
  auto back = approach();
  back.header.stamp = ros::Time(100.033);
  for (auto& point : back.points) point.pose.pose.position.x -= 20.0;
  const auto output = core.process(back, light(interfaces::TrafficLight::UNKNOWN, 0.0), {}, {}).first;
  EXPECT_NEAR(output.points[65].velocity, creep(9.0, 5.0), 1e-9);
}

TEST(IntegratedStopDynamic, BackwardStampRecountsGoFrames) {
  velocity_planner::StopLineState state(stopLines());
  velocity_planner::IntegratedStopDynamic core(blind(), state);
  core.process(approach(), light(interfaces::TrafficLight::RED, 99.9), {}, {});
  // The clock rewinds: the stop latches again and GO images on the new clock confirm the release.
  auto rewound = approach();
  rewound.header.stamp = ros::Time(50.0);
  core.process(rewound, light(interfaces::TrafficLight::RED, 49.9), {}, {});
  ASSERT_TRUE(state.stopping);
  for (int frame = 1; frame <= 3; ++frame) {
    rewound.header.stamp = ros::Time(50.0 + frame / 30.0);
    core.process(rewound, light(interfaces::TrafficLight::GREEN, 49.9 + frame / 10.0), {}, {});
  }
  EXPECT_FALSE(state.stopping);
}

namespace {
// The approach() path started start_x metres further along +x, so the line at x=50 is 50 - start_x ahead.
interfaces::Path approachFrom(double start_x) {
  interfaces::Path path = approach();
  for (auto& point : path.points) point.pose.pose.position.x += start_x;
  return path;
}

interfaces::CompetitionVehicleStatus moving(double speed_mps) {
  interfaces::CompetitionVehicleStatus status;
  status.signed_velocity_mps = speed_mps;
  return status;
}
}  // namespace

TEST(IntegratedStopDynamic, CloseYellowStopsWhenFrontAxleCanStop) {
  velocity_planner::StopLineState state(stopLines());
  auto parameters = blind();
  parameters.yellow_stop_deceleration_mps2 = 4.5;
  velocity_planner::IntegratedStopDynamic core(parameters, state);
  // Line 12 m ahead at 7.1 m/s: 7.1^2 / 9 + 7.1 * 0.35 = 8.09 <= 12 - 3 - 0.5.
  core.process(approachFrom(38.0), light(interfaces::TrafficLight::YELLOW, 99.9), moving(7.1), {});
  EXPECT_TRUE(state.stopping);
}

TEST(IntegratedStopDynamic, CloseYellowPassesWithoutFrontAxleRule) {
  velocity_planner::StopLineState state(stopLines());
  velocity_planner::IntegratedStopDynamic core(blind(), state);
  // Comfortable stop needs 7.1^2 / 6 = 8.4 m, more than the 12 m line minus the rear-axle offset.
  core.process(approachFrom(38.0), light(interfaces::TrafficLight::YELLOW, 99.9), moving(7.1), {});
  EXPECT_FALSE(state.stopping);
}

TEST(IntegratedStopDynamic, YellowTooCloseForFrontAxlePasses) {
  velocity_planner::StopLineState state(stopLines());
  auto parameters = blind();
  parameters.yellow_stop_deceleration_mps2 = 4.5;
  velocity_planner::IntegratedStopDynamic core(parameters, state);
  // Line 9 m ahead: 8.09 m > 9 - 3 - 0.5.
  core.process(approachFrom(41.0), light(interfaces::TrafficLight::YELLOW, 99.9), moving(7.1), {});
  EXPECT_FALSE(state.stopping);
}
