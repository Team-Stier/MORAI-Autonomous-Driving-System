#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "morai_path_tracking/controllers/longitudinal/pid_controller.hpp"

namespace morai_path_tracking {
namespace {

TEST(LongitudinalPid, ProducesAccelBelowTarget) {
  LongitudinalPid pid(PidConfig{});
  const auto command = pid.update(3.0, 0.0, 0.1);
  EXPECT_GT(command.accel, 0.0);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, ProducesBrakeAboveTarget) {
  LongitudinalPid pid(PidConfig{});
  const auto command = pid.update(3.0, 4.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_GT(command.brake, 0.0);
}

TEST(LongitudinalPid, ClampsAccelAndBrakeOutputs) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.maximum_accel = 0.4;
  config.maximum_brake = 0.6;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.4, pid.update(2.0, 0.0, 0.1).accel);
  EXPECT_DOUBLE_EQ(0.6, pid.update(0.0, 2.0, 0.1).brake);
}

TEST(LongitudinalPid, KeepsAccelAndBrakeMutuallyExclusive) {
  LongitudinalPid pid(PidConfig{});
  const auto accelerating = pid.update(3.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, accelerating.brake);
  const auto braking = pid.update(0.0, 3.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, braking.accel);
}

TEST(LongitudinalPid, LimitsSignedCommandChangeByElapsedTime) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.maximum_accel = 1.0;
  config.maximum_brake = 1.0;
  config.command_rate_limit_per_sec = 1.0;
  LongitudinalPid pid(config);

  const auto first = pid.update(10.0, 0.0, 0.1);
  const auto second = pid.update(10.0, 0.0, 0.1);

  EXPECT_NEAR(0.1, first.accel, 1.0e-12);
  EXPECT_DOUBLE_EQ(0.0, first.brake);
  EXPECT_NEAR(0.2, second.accel, 1.0e-12);
  EXPECT_DOUBLE_EQ(0.0, second.brake);
}

TEST(LongitudinalPid, ReversesFromAccelToBrakeThroughCoast) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.maximum_accel = 1.0;
  config.maximum_brake = 1.0;
  config.command_rate_limit_per_sec = 1.0;
  LongitudinalPid pid(config);
  ASSERT_NEAR(0.2, pid.update(10.0, 0.0, 0.2).accel, 1.0e-12);

  const auto still_accel = pid.update(0.0, 10.0, 0.1);
  const auto coast = pid.update(0.0, 10.0, 0.1);
  const auto braking = pid.update(0.0, 10.0, 0.1);

  EXPECT_NEAR(0.1, still_accel.accel, 1.0e-12);
  EXPECT_DOUBLE_EQ(0.0, still_accel.brake);
  EXPECT_NEAR(0.0, coast.accel, 1.0e-12);
  EXPECT_NEAR(0.0, coast.brake, 1.0e-12);
  EXPECT_DOUBLE_EQ(0.0, braking.accel);
  EXPECT_NEAR(0.1, braking.brake, 1.0e-12);
}

TEST(LongitudinalPid, StopsAtCoastWhenRateStepWouldCrossZero) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.maximum_accel = 1.0;
  config.maximum_brake = 1.0;
  config.command_rate_limit_per_sec = 1.0;
  LongitudinalPid pid(config);
  ASSERT_NEAR(0.15, pid.update(10.0, 0.0, 0.15).accel, 1.0e-12);

  const auto coast = pid.update(0.0, 10.0, 0.2);

  EXPECT_DOUBLE_EQ(0.0, coast.accel);
  EXPECT_DOUBLE_EQ(0.0, coast.brake);
}

TEST(LongitudinalPid, AppliesDeadband) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.error_deadband_mps = 0.05;
  LongitudinalPid pid(config);
  const auto command = pid.update(1.04, 1.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, SuppressesDerivativeInsideDeadband) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 0.0;
  config.kd = 1.0;
  config.error_deadband_mps = 0.05;
  config.maximum_accel = 20.0;
  config.maximum_brake = 20.0;
  LongitudinalPid pid(config);
  pid.update(1.0, 0.0, 0.1);

  const auto command = pid.update(1.0, 0.96, 0.1);

  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, UnwindsIntegralTowardZeroInsideDeadband) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 1.0;
  config.kd = 0.0;
  config.integral_unwind_rate_per_sec = 0.5;
  config.maximum_accel = 2.0;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(1.0, pid.update(1.0, 0.0, 1.0).accel);

  EXPECT_DOUBLE_EQ(0.75, pid.update(1.0, 0.98, 0.5).accel);
  EXPECT_DOUBLE_EQ(0.50, pid.update(1.0, 0.98, 0.5).accel);
  EXPECT_DOUBLE_EQ(0.25, pid.update(1.0, 0.98, 0.5).accel);
  EXPECT_DOUBLE_EQ(0.00, pid.update(1.0, 0.98, 0.5).accel);
}

TEST(LongitudinalPid, AppliesUnwindPolicyAtInclusiveDeadbandBoundary) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 1.0;
  config.kd = 1.0;
  config.integral_unwind_rate_per_sec = 0.5;
  config.error_deadband_mps = 0.125;
  config.maximum_accel = 20.0;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(1.0, pid.update(1.0, 0.0, 1.0).accel);

  const auto command = pid.update(1.0, 0.875, 0.5);

  EXPECT_DOUBLE_EQ(0.75, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, UsesLastDeadbandMeasurementWhenLeavingDeadband) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 0.0;
  config.kd = 1.0;
  config.error_deadband_mps = 0.05;
  config.maximum_accel = 10.0;
  config.maximum_brake = 10.0;
  LongitudinalPid pid(config);
  pid.update(1.0, 0.0, 1.0);
  pid.update(1.0, 0.96, 0.1);

  const auto command = pid.update(1.0, 0.8, 0.1);

  EXPECT_DOUBLE_EQ(1.6, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, UsesDerivativeOfMeasurement) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 0.0;
  config.kd = 1.0;
  config.maximum_brake = 20.0;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.0, pid.update(3.0, 0.0, 0.1).brake);
  EXPECT_DOUBLE_EQ(10.0, pid.update(3.0, 1.0, 0.1).brake);
}

TEST(LongitudinalPid, ClampsIntegralState) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 1.0;
  config.kd = 0.0;
  config.integral_limit = 0.2;
  config.maximum_accel = 2.0;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.2, pid.update(1.0, 0.0, 1.0).accel);
  EXPECT_DOUBLE_EQ(0.2, pid.update(1.0, 0.0, 1.0).accel);
}

TEST(LongitudinalPid, PreventsIntegralWindupIntoSaturation) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 1.0;
  config.kd = 0.0;
  config.integral_limit = 10.0;
  config.maximum_accel = 0.5;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.5, pid.update(1.0, 0.0, 1.0).accel);
  EXPECT_DOUBLE_EQ(0.0, pid.update(0.0, 0.0, 1.0).accel);
}

TEST(LongitudinalPid, AddsTargetSpeedAccelFeedforward) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.accel_feedforward_gain_per_mps = 0.008;
  LongitudinalPid pid(config);

  EXPECT_NEAR(0.08, pid.update(10.0, 10.0, 0.1).accel, 1.0e-12);
  EXPECT_DOUBLE_EQ(0.0, pid.update(0.0, 0.0, 0.1).accel);
}

TEST(LongitudinalPid, CutsFeedforwardAndBrakesOutsideOverspeedDeadband) {
  PidConfig config;
  config.kp = 0.12;
  config.ki = 0.0;
  config.kd = 0.0;
  config.error_deadband_mps = 0.1;
  config.accel_feedforward_gain_per_mps = 0.008;
  LongitudinalPid pid(config);

  const auto command = pid.update(12.6, 13.2, 0.1);

  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_GT(command.brake, 0.07);
}

TEST(LongitudinalPid, CoastsInsteadOfBrakingForSmallOverspeed) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.accel_feedforward_gain_per_mps = 0.1;
  config.coast_overspeed_threshold_mps = 0.05;
  config.brake_overspeed_threshold_mps = 0.50;
  LongitudinalPid pid(config);

  const auto command = pid.update(10.0, 10.2, 0.1);

  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
  EXPECT_EQ(LongitudinalState::kCoast, command.state);
}

TEST(LongitudinalPid, BrakesAtConfiguredOverspeedThreshold) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.coast_overspeed_threshold_mps = 0.05;
  config.brake_overspeed_threshold_mps = 0.50;
  LongitudinalPid pid(config);

  const auto command = pid.update(10.0, 10.5, 0.1);

  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_GT(command.brake, 0.0);
  EXPECT_EQ(LongitudinalState::kBrake, command.state);
}

TEST(LongitudinalPid, HardSpeedGuardImmediatelyOverridesAcceleration) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.maximum_accel = 1.0;
  config.maximum_brake = 0.6;
  config.command_rate_limit_per_sec = 0.1;
  config.hard_brake_activation_speed_mps = 59.0 / 3.6;
  config.minimum_hard_brake_command = 0.25;
  LongitudinalPid pid(config);

  const auto command = pid.update(30.0, 59.1 / 3.6, 0.1);

  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_GE(command.brake, 0.25);
  EXPECT_EQ(LongitudinalState::kHardSpeedBrake, command.state);
}

TEST(LongitudinalPid, NeverAcceleratesWhileOverspeedDuringDeceleration) {
  PidConfig config;
  config.kp = 0.12;
  config.ki = 0.0;
  config.kd = 0.02;
  config.error_deadband_mps = 0.1;
  config.accel_feedforward_gain_per_mps = 0.008;
  LongitudinalPid pid(config);
  (void)pid.update(10.0, 14.0, 0.1);

  const auto command = pid.update(10.0, 11.0, 0.1);

  EXPECT_DOUBLE_EQ(0.0, command.accel);
}

TEST(LongitudinalPid, ResetClearsIntegralAndDerivativeHistory) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 1.0;
  config.kd = 1.0;
  config.maximum_accel = 10.0;
  LongitudinalPid pid(config);
  ASSERT_GT(pid.update(1.0, 0.0, 1.0).accel, 0.0);
  pid.reset();
  const auto command = pid.update(0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, ResetClearsCommandRateLimiterHistory) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.maximum_accel = 1.0;
  config.command_rate_limit_per_sec = 1.0;
  LongitudinalPid pid(config);
  EXPECT_NEAR(0.1, pid.update(10.0, 0.0, 0.1).accel, 1.0e-12);
  EXPECT_NEAR(0.2, pid.update(10.0, 0.0, 0.1).accel, 1.0e-12);

  pid.reset();

  EXPECT_NEAR(0.1, pid.update(10.0, 0.0, 0.1).accel, 1.0e-12);
}

TEST(LongitudinalPid, RejectsInvalidUpdateInputs) {
  LongitudinalPid pid(PidConfig{});
  EXPECT_THROW(pid.update(1.0, 0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(pid.update(1.0, 0.0, -0.1), std::invalid_argument);
  EXPECT_THROW(pid.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1),
               std::invalid_argument);
  EXPECT_THROW(pid.update(1.0, std::numeric_limits<double>::infinity(), 0.1),
               std::invalid_argument);
}

TEST(LongitudinalPid, InvalidUpdateDoesNotMutateIntegralState) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 1.0;
  config.kd = 0.0;
  config.integral_limit = 10.0;
  config.maximum_accel = 10.0;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(1.0, pid.update(1.0, 0.0, 1.0).accel);

  EXPECT_THROW(pid.update(1.0, std::numeric_limits<double>::infinity(), 0.1),
               std::invalid_argument);

  EXPECT_DOUBLE_EQ(2.0, pid.update(1.0, 0.0, 1.0).accel);
}

TEST(LongitudinalPid, RejectsInvalidConfiguration) {
  PidConfig config;
  config.kp = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.ki = std::numeric_limits<double>::infinity();
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.integral_limit = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.maximum_brake = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.integral_unwind_rate_per_sec = 0.0;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.integral_unwind_rate_per_sec = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.integral_unwind_rate_per_sec =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.accel_feedforward_gain_per_mps = -0.001;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.accel_feedforward_gain_per_mps =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.command_rate_limit_per_sec = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.command_rate_limit_per_sec =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);
}

TEST(LongitudinalPid, RejectsInvalidCoastAndHardSpeedConfiguration) {
  PidConfig config;
  config.coast_overspeed_threshold_mps = 0.5;
  config.brake_overspeed_threshold_mps = 0.5;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.hard_brake_activation_speed_mps = 0.0;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.minimum_hard_brake_command = config.maximum_brake + 0.01;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
