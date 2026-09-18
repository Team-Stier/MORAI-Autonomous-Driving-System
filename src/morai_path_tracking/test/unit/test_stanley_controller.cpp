#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "morai_path_tracking/controllers/lateral/stanley_controller.hpp"

namespace morai_path_tracking {
namespace {

constexpr double kPi = 3.14159265358979323846;

StanleyConfig competitionConfig() {
  StanleyConfig config;
  config.wheelbase_m = 3.0;
  config.gain = 2.0;
  config.softening_speed_mps = 2.0;
  config.minimum_control_speed_mps = 1.0;
  config.heading_window_m = 4.0;
  config.maximum_steering_angle_rad = 40.0 * kPi / 180.0;
  config.maximum_steering_rate_rad_per_sec = 90.0 * kPi / 180.0;
  return config;
}

std::vector<Point2d> leftCircularPath(double radius_m,
                                      double maximum_angle_rad) {
  std::vector<Point2d> path;
  for (double angle_rad = -0.20; angle_rad <= maximum_angle_rad;
       angle_rad += 0.01) {
    path.push_back(
        {3.0 + radius_m * std::sin(angle_rad),
         radius_m * (1.0 - std::cos(angle_rad))});
  }
  return path;
}

TEST(StanleyController, StraightCenteredPathHasZeroError) {
  const StanleyResult result = StanleyController(competitionConfig())
                                   .calculate({{0.0, 0.0}, {10.0, 0.0}},
                                              5.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.0, result.cross_track_error_m, 1.0e-12);
  EXPECT_NEAR(0.0, result.heading_error_rad, 1.0e-12);
  EXPECT_NEAR(0.0, result.steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(3.0, result.target.x, 1.0e-12);
  EXPECT_NEAR(0.0, result.target.y, 1.0e-12);
}

TEST(StanleyController, PathOnLeftProducesPositiveSteering) {
  const StanleyResult result = StanleyController(competitionConfig())
                                   .calculate({{0.0, 1.0}, {10.0, 1.0}},
                                              5.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.cross_track_error_m, 0.0);
  EXPECT_GT(result.steering_angle_rad, 0.0);
}

TEST(StanleyController, PathOnRightProducesNegativeSteering) {
  const StanleyResult result = StanleyController(competitionConfig())
                                   .calculate({{0.0, -1.0}, {10.0, -1.0}},
                                              5.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_LT(result.cross_track_error_m, 0.0);
  EXPECT_LT(result.steering_angle_rad, 0.0);
}

TEST(StanleyController,
     OneMetreErrorAtFiftyFiveKphAddsAboutSixPointSixDegrees) {
  const StanleyResult result =
      StanleyController(competitionConfig())
          .calculate({{0.0, 1.0}, {20.0, 1.0}}, 55.0 / 3.6, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(6.6 * kPi / 180.0, result.steering_angle_rad, 0.002);
}

TEST(StanleyController, UsesPathHeadingRelativeToVehicle) {
  const StanleyResult result =
      StanleyController(competitionConfig())
          .calculate({{0.0, 0.0}, {10.0, 10.0}}, 5.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.25 * kPi, result.heading_error_rad, 1.0e-12);
}

TEST(StanleyController, SmoothsShortSegmentHeadingOverSpatialWindow) {
  const std::vector<Point2d> path{
      {0.0, 0.0}, {2.8, 0.0}, {2.9, 0.1},
      {3.1, -0.1}, {3.2, 0.0}, {6.0, 0.0}};

  const StanleyResult result =
      StanleyController(competitionConfig()).calculate(path, 15.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(3.0, result.target.x, 1.0e-12);
  EXPECT_NEAR(0.0, result.target.y, 1.0e-12);
  EXPECT_NEAR(0.0, result.heading_error_rad, 1.0e-12);
  EXPECT_NEAR(0.0, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController,
     CircularPathUsesCurvatureFeedforwardWithoutSteadyTrackingError) {
  StanleyConfig config = competitionConfig();
  config.heading_error_gain = 0.7;
  config.curvature_feedforward_gain = 1.0;
  config.curvature_preview_distance_m = 8.0;
  config.yaw_rate_damping_gain_sec = 0.1;
  const double radius_m = 30.0;
  const double speed_mps = 10.0;

  const StanleyResult result =
      StanleyController(config).calculate(
          leftCircularPath(radius_m, 0.8), speed_mps, speed_mps / radius_m,
          0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(1.0 / radius_m, result.reference_curvature_m_inv, 2.0e-4);
  EXPECT_NEAR(speed_mps / radius_m, result.reference_yaw_rate_radps, 2.0e-3);
  EXPECT_NEAR(0.0, result.yaw_rate_error_radps, 2.0e-3);
  EXPECT_NEAR(std::atan(config.wheelbase_m / radius_m),
              result.steering_angle_rad, 1.0e-3);
}

TEST(StanleyController, PositiveYawRateOnStraightPathProducesCounterSteering) {
  StanleyConfig config = competitionConfig();
  config.heading_error_gain = 0.7;
  config.curvature_feedforward_gain = 1.0;
  config.curvature_preview_distance_m = 8.0;
  config.yaw_rate_damping_gain_sec = 0.1;

  const StanleyResult result =
      StanleyController(config).calculate(
          {{0.0, 0.0}, {20.0, 0.0}}, 10.0, 0.4, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.0, result.reference_curvature_m_inv, 1.0e-12);
  EXPECT_NEAR(0.4, result.yaw_rate_error_radps, 1.0e-12);
  EXPECT_NEAR(-0.04, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController,
     NonlinearYawDampingStrengthensOnlyWhenMeasuredRotationIsLarge) {
  StanleyConfig config = competitionConfig();
  config.heading_error_gain = 0.5;
  config.curvature_feedforward_gain = 1.0;
  config.curvature_preview_distance_m = 8.0;
  config.yaw_rate_damping_gain_sec = 0.1;
  config.yaw_rate_damping_nonlinear_gain_sec2 = 0.5;

  const StanleyResult result =
      StanleyController(config).calculate(
          {{0.0, 0.0}, {20.0, 0.0}}, 10.0, 0.4, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.3, result.applied_yaw_rate_damping_gain_sec, 1.0e-12);
  EXPECT_NEAR(-0.12, result.yaw_rate_damping_steering_rad, 1.0e-12);
  EXPECT_NEAR(-0.12, result.requested_steering_angle_rad, 1.0e-12);
  EXPECT_NEAR(-0.12, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController, RightCircularPathProducesNegativeReferenceCurvature) {
  StanleyConfig config = competitionConfig();
  config.curvature_feedforward_gain = 1.0;
  config.curvature_preview_distance_m = 8.0;

  std::vector<Point2d> path = leftCircularPath(25.0, 0.8);
  for (Point2d& point : path) {
    point.y = -point.y;
  }
  const StanleyResult result =
      StanleyController(config).calculate(path, 8.0, 0.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(-1.0 / 25.0, result.reference_curvature_m_inv, 2.0e-4);
  EXPECT_LT(result.steering_angle_rad, 0.0);
}

TEST(StanleyController, ClampsRequestedSteeringToPhysicalLimit) {
  const StanleyResult result =
      StanleyController(competitionConfig())
          .calculate({{0.0, 10.0}, {10.0, 10.0}}, 0.0, 0.0, 1.0);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(40.0 * kPi / 180.0, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController, LimitsSteeringChangeAtFiftyHertz) {
  const StanleyResult result =
      StanleyController(competitionConfig())
          .calculate({{0.0, 10.0}, {10.0, 10.0}}, 0.0, 0.0, 0.02);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(1.8 * kPi / 180.0, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController, RateLimitStartsFromPreviousCommand) {
  const double previous_steering = -10.0 * kPi / 180.0;
  const StanleyResult result =
      StanleyController(competitionConfig())
          .calculate({{0.0, 10.0}, {10.0, 10.0}}, 0.0,
                     previous_steering, 0.02);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(-8.2 * kPi / 180.0, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController, AlwaysKeepsRateLimitedOutputWithinPhysicalLimit) {
  const StanleyResult result =
      StanleyController(competitionConfig())
          .calculate({{0.0, 0.0}, {10.0, 0.0}}, 5.0,
                     60.0 * kPi / 180.0, 0.02);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(40.0 * kPi / 180.0, result.steering_angle_rad, 1.0e-12);
}

TEST(StanleyController, ZeroSpeedProducesFiniteSteering) {
  const StanleyResult result = StanleyController(competitionConfig())
                                   .calculate({{0.0, 1.0}, {10.0, 1.0}},
                                              0.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_TRUE(std::isfinite(result.steering_angle_rad));
}

TEST(StanleyController, RejectsPathWithoutValidSegment) {
  StanleyController controller(competitionConfig());
  EXPECT_FALSE(controller.calculate({{0.0, 0.0}}, 5.0, 0.0, 0.02).valid);
  EXPECT_FALSE(
      controller.calculate({{0.0, 0.0}, {0.0, 0.0}}, 5.0, 0.0, 0.02)
          .valid);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      controller.calculate({{nan, 0.0}, {5.0, 0.0}}, 5.0, 0.0, 0.02)
          .valid);
}

TEST(StanleyController, RejectsInvalidRuntimeInputs) {
  StanleyController controller(competitionConfig());
  const std::vector<Point2d> path{{0.0, 0.0}, {10.0, 0.0}};

  EXPECT_FALSE(controller.calculate(path, -0.1, 0.0, 0.02).valid);
  EXPECT_FALSE(
      controller
          .calculate(path, std::numeric_limits<double>::infinity(), 0.0, 0.02)
          .valid);
  EXPECT_FALSE(controller.calculate(path, 1.0, 0.0, 0.0).valid);
  EXPECT_FALSE(controller.calculate(path, 1.0, 0.0, -0.02).valid);
  EXPECT_FALSE(
      controller
          .calculate(path, 1.0,
                     std::numeric_limits<double>::quiet_NaN(), 0.02)
          .valid);
  EXPECT_FALSE(
      controller
          .calculate(path, 1.0,
                     std::numeric_limits<double>::quiet_NaN(), 0.0, 0.02)
          .valid);
}

TEST(StanleyController, RejectsInvalidConfiguration) {
  const std::vector<Point2d> path{{0.0, 0.0}, {10.0, 0.0}};

  StanleyConfig config = competitionConfig();
  config.wheelbase_m = 0.0;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.gain = 0.0;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.softening_speed_mps = 0.0;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.minimum_control_speed_mps = 0.0;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.maximum_steering_angle_rad =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.maximum_steering_rate_rad_per_sec = -1.0;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.heading_window_m = 0.0;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.heading_window_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.heading_error_gain = -0.1;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.curvature_feedforward_gain =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.curvature_preview_distance_m = 0.0;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.yaw_rate_damping_gain_sec = -0.1;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);

  config = competitionConfig();
  config.yaw_rate_damping_nonlinear_gain_sec2 = -0.1;
  EXPECT_FALSE(StanleyController(config).calculate(path, 1.0, 0.0, 0.02).valid);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
