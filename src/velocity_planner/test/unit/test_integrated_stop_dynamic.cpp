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
