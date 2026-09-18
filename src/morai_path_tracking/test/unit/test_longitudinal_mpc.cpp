#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "morai_path_tracking/controllers/longitudinal/mpc/longitudinal_mpc.hpp"

namespace morai_path_tracking {
namespace {

LongitudinalMpcConfig testConfig(const std::string& solver) {
  LongitudinalMpcConfig config;
  config.solver = solver;
  config.horizon_steps = 16;
  config.optimizer_iterations = 6;
  return config;
}

TEST(LongitudinalMpcTest, BothSolversAccelerateBelowTarget) {
  for (const std::string solver : {"osqp", "projected_gradient", "coordinate_search"}) {
    LongitudinalMpc controller(testConfig(solver));
    const LongitudinalMpcResult result = controller.update(10.0, 2.0, 0.05);
    ASSERT_TRUE(result.valid) << solver << ": " << result.error;
    EXPECT_GT(result.accel, 0.0) << solver;
    EXPECT_DOUBLE_EQ(result.brake, 0.0);
    EXPECT_LE(result.accel, testConfig(solver).maximum_accel_command);
  }
}

TEST(LongitudinalMpcTest, BrakesAboveTargetWithoutMixedPedals) {
  LongitudinalMpc controller(testConfig("projected_gradient"));
  const LongitudinalMpcResult result = controller.update(5.0, 9.0, 0.05);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_DOUBLE_EQ(result.accel, 0.0);
  EXPECT_GT(result.brake, 0.0);
}

TEST(LongitudinalMpcTest, HardGuardOverridesRateLimitNearSixtyKph) {
  LongitudinalMpcConfig config = testConfig("osqp");
  LongitudinalMpc controller(config);
  const LongitudinalMpcResult result =
      controller.update(config.maximum_speed_mps, config.maximum_speed_mps,
                        0.01);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_TRUE(result.hard_speed_guard_active);
  EXPECT_DOUBLE_EQ(result.accel, 0.0);
  EXPECT_GE(result.brake, config.minimum_hard_brake_command);
}

TEST(LongitudinalMpcTest, NormalCommandChangeIsRateLimited) {
  LongitudinalMpcConfig config = testConfig("projected_gradient");
  config.command_rate_limit_per_sec = 0.5;
  LongitudinalMpc controller(config);
  const LongitudinalMpcResult result = controller.update(12.0, 1.0, 0.02);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_LE(std::abs(result.signed_command), 0.5 * 0.02 + 1.0e-12);
}

TEST(LongitudinalMpcTest, RejectsUnknownSolver) {
  LongitudinalMpcConfig config;
  config.solver = "unknown";
  EXPECT_THROW(LongitudinalMpc controller(config), std::invalid_argument);
}

TEST(LongitudinalMpcTest, AcceptsSignedCompetitionVelocityAsSpeedMagnitude) {
  LongitudinalMpc controller(testConfig("osqp"));
  const LongitudinalMpcResult result = controller.update(5.0, -0.01, 0.05);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.accel, 0.0);
}

TEST(LongitudinalMpcTest, ClosedLoopApproachesTargetWithoutExceedingSixtyKph) {
  LongitudinalMpcConfig config = testConfig("osqp");
  config.horizon_steps = 24;
  config.optimizer_iterations = 8;
  LongitudinalMpc controller(config);
  double speed = 0.0;
  double acceleration = 0.0;
  double maximum_speed = 0.0;
  const double dt = 0.05;
  const double target = 59.0 / 3.6;
  for (int step = 0; step < 800; ++step) {
    const LongitudinalMpcResult result = controller.update(target, speed, dt);
    ASSERT_TRUE(result.valid) << result.error;
    const double requested_acceleration =
        result.signed_command >= 0.0
            ? config.acceleration_gain_mps2 * result.signed_command
            : config.braking_gain_mps2 * result.signed_command;
    const double alpha =
        1.0 - std::exp(-dt / config.actuator_time_constant_sec);
    acceleration += alpha * (requested_acceleration - acceleration);
    speed = std::max(0.0, speed + acceleration * dt);
    maximum_speed = std::max(maximum_speed, speed);
  }
  EXPECT_GT(speed * 3.6, 58.8);
  EXPECT_LE(maximum_speed * 3.6, 60.0);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
