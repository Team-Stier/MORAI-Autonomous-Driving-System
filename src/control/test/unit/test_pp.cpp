#include <control/cores/pp.hpp>
#include <gtest/gtest.h>
#include <array>
#include <initializer_list>

namespace {

interfaces::Path path(std::initializer_list<std::array<double, 3>> points) {
  interfaces::Path result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  for (const auto& coordinates : points) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = coordinates[0];
    point.pose.pose.position.y = coordinates[1];
    point.pose.pose.orientation.w = 1;
    point.velocity = coordinates[2];
    result.points.push_back(point);
  }
  return result;
}

nav_msgs::Odometry ego() {
  nav_msgs::Odometry result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "map";
  result.child_frame_id = "base_link";
  result.pose.pose.orientation.w = 1;
  return result;
}

interfaces::CompetitionVehicleStatus status(double speed_mps) {
  interfaces::CompetitionVehicleStatus result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  result.velocity_x_mps = speed_mps;
  return result;
}

control::PpParameters geometryParameters() {
  control::PpParameters parameters;
  parameters.lookahead_min_m = 5;
  parameters.steering_rate_limit_radps = 100;
  parameters.speed_ki = 0;
  parameters.speed_kd = 0;
  return parameters;
}

TEST(Pp, UsesForwardGeometryAndNormalizesSteeringToFortyDegrees) {
  control::Pp straight(geometryParameters());
  control::Pp left(geometryParameters());
  control::Pp right(geometryParameters());
  control::Pp sharp(geometryParameters());
  EXPECT_DOUBLE_EQ(straight.process(path({{-8, 0, 1}, {0, 0, 1}, {20, 0, 1}}),
                                    ego(), status(1)).steering, 0);
  // The 5 m circle meets the left segment at (4, 3): atan(18 / 25) / 40 degrees.
  EXPECT_NEAR(left.process(path({{-8, -6, 1}, {0, 0, 1}, {8, 6, 1}}),
                           ego(), status(1)).steering, 0.8938471813609188, 1e-9);
  EXPECT_NEAR(right.process(path({{-8, 6, 1}, {0, 0, 1}, {8, -6, 1}}),
                            ego(), status(1)).steering, -0.8938471813609188, 1e-9);
  EXPECT_DOUBLE_EQ(sharp.process(path({{0, 0, 1}, {3, 4, 1}, {6, 8, 1}}),
                                 ego(), status(1)).steering, 1);
}

TEST(Pp, InterpolatesSparseSegmentsAtTheSpeedDependentLookahead) {
  auto parameters = geometryParameters();
  parameters.lookahead_min_m = 2;
  control::Pp core(parameters);
  // At 10 m/s the radius is 6 m; this ray gives atan(1 / sqrt(5)) / 40 degrees.
  const auto command = core.process(path({{0, 0, 10}, {20, 10, 10}}), ego(), status(10));
  EXPECT_NEAR(command.steering, 0.6023710638027675, 1e-9);
}

TEST(Pp, UsesTheActualDistanceWhenThePathEndsBeforeLookahead) {
  control::Pp core(geometryParameters());
  // The endpoint is sqrt(10) m away: atan(6 / 10) / 40 degrees.
  const auto command = core.process(path({{0, 0, 1}, {3, 1, 1}}), ego(), status(1));
  EXPECT_NEAR(command.steering, 0.7740939133018381, 1e-9);
}

TEST(Pp, SelectsTheForwardExitInsteadOfThePastEntryToLookahead) {
  control::Pp core(geometryParameters());
  // The forward exit is (4.8, -1.4); the preceding entry turns left instead.
  const auto command = core.process(path({{1, 6, 1}, {4, -1, 1}, {8, -3, 1}}), ego(), status(1));
  EXPECT_NEAR(command.steering, -0.4643087127601826, 1e-9);
}

TEST(Pp, TracksTheNearestInjectedSpeedUsingExclusivePedals) {
  auto parameters = geometryParameters();
  parameters.acceleration_gain_mps2 = 2;
  parameters.braking_gain_mps2 = 4;
  control::Pp core(parameters);
  auto input = path({{-10, 0, 13}, {0, 0, 6}, {20, 0, 13}});
  auto odometry = ego();
  odometry.twist.twist.linear.x = 100;
  const auto accelerating = core.process(input, odometry, status(5));
  EXPECT_EQ(accelerating.longlCmdType, 1);
  EXPECT_NEAR(accelerating.accel, 0.6, 1e-12);
  EXPECT_DOUBLE_EQ(accelerating.brake, 0);

  input.points[1].velocity = 4;
  const auto braking = core.process(input, odometry, status(5));
  EXPECT_DOUBLE_EQ(braking.accel, 0);
  EXPECT_NEAR(braking.brake, 0.3, 1e-12);
}

TEST(Pp, DifferentiatesMeasuredSpeedWithoutATargetStepKick) {
  auto parameters = geometryParameters();
  parameters.speed_kp = 0;
  parameters.speed_kd = 1;
  parameters.acceleration_gain_mps2 = 1;
  parameters.braking_gain_mps2 = 1;
  control::Pp core(parameters);
  core.process(path({{0, 0, 5}, {20, 0, 5}}), ego(), status(5));

  const auto target_step = core.process(path({{0, 0, 10}, {20, 0, 10}}), ego(), status(5));
  EXPECT_DOUBLE_EQ(target_step.accel, 0);
  EXPECT_DOUBLE_EQ(target_step.brake, 0);
  const auto speed_rise = core.process(path({{0, 0, 10}, {20, 0, 10}}), ego(), status(6));
  EXPECT_DOUBLE_EQ(speed_rise.accel, 0);
  EXPECT_DOUBLE_EQ(speed_rise.brake, 0.6);
}

TEST(Pp, HoldsAStoppedVehicleAndClearsTheSpeedIntegral) {
  auto parameters = geometryParameters();
  parameters.speed_kp = 0;
  parameters.speed_ki = 1;
  parameters.acceleration_gain_mps2 = 1;
  control::Pp core(parameters);
  morai_msgs::CtrlCmd accumulated;
  for (int i = 0; i < 6; ++i)
    accumulated = core.process(path({{0, 0, 6}, {20, 0, 6}}), ego(), status(5));
  EXPECT_NEAR(accumulated.accel, 0.3, 1e-12);

  const auto stopped = core.process(path({{0, 0, 0}, {20, 0, 0}}), ego(), status(0));
  EXPECT_DOUBLE_EQ(stopped.accel, 0);
  EXPECT_DOUBLE_EQ(stopped.brake, 0.2);
  const auto resumed = core.process(path({{0, 0, 5}, {20, 0, 5}}), ego(), status(5));
  EXPECT_DOUBLE_EQ(resumed.accel, 0);
  EXPECT_DOUBLE_EQ(resumed.brake, 0);
}

TEST(Pp, PreventsIntegralWindupAndBoundsUnsaturatedIntegralAction) {
  auto parameters = geometryParameters();
  parameters.speed_kp = 1;
  parameters.speed_ki = 1;
  parameters.acceleration_gain_mps2 = 1;
  control::Pp saturated(parameters);
  morai_msgs::CtrlCmd capped;
  for (int i = 0; i < 40; ++i)
    capped = saturated.process(path({{0, 0, 15}, {20, 0, 15}}), ego(), status(5));
  EXPECT_DOUBLE_EQ(capped.accel, 0.7);
  EXPECT_DOUBLE_EQ(capped.brake, 0);
  const auto recovered = saturated.process(path({{0, 0, 5}, {20, 0, 5}}), ego(), status(5));
  EXPECT_DOUBLE_EQ(recovered.accel, 0);
  EXPECT_DOUBLE_EQ(recovered.brake, 0);

  parameters.speed_kp = 0;
  parameters.acceleration_gain_mps2 = 10;
  parameters.integral_limit_mps_sec = 0.2;
  control::Pp bounded(parameters);
  morai_msgs::CtrlCmd command;
  for (int i = 0; i < 20; ++i)
    command = bounded.process(path({{0, 0, 6}, {20, 0, 6}}), ego(), status(5));
  EXPECT_NEAR(command.accel, 0.02, 1e-12);
  EXPECT_DOUBLE_EQ(command.brake, 0);
}

TEST(Pp, LimitsSteeringChangesAtTheConfiguredControlPeriod) {
  auto parameters = geometryParameters();
  parameters.steering_rate_limit_radps = 1.0471975512;
  control::Pp core(parameters);
  const auto left = path({{0, 0, 1}, {3, 4, 1}, {6, 8, 1}});
  EXPECT_NEAR(core.process(left, ego(), status(1)).steering, 0.075, 1e-9);
  EXPECT_NEAR(core.process(left, ego(), status(1)).steering, 0.15, 1e-9);
  const auto right = path({{0, 0, 1}, {3, -4, 1}, {6, -8, 1}});
  EXPECT_NEAR(core.process(right, ego(), status(1)).steering, 0.075, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
