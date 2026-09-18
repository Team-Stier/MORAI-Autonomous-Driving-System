#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "morai_path_tracking/planning/wheel_corridor.hpp"

namespace morai_path_tracking {
namespace {

WheelCorridorConfig competitionCorridor() {
  WheelCorridorConfig config;
  config.lane_half_width_m = 1.30;
  config.vehicle_width_m = 1.892;
  config.wheelbase_m = 3.0;
  return config;
}

TEST(WheelCorridor, ReportsCenteredStraightWheelClearance) {
  const std::vector<Point2d> path = {
      {-5.0, 0.0}, {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};

  const WheelCorridorResult result =
      estimateWheelCorridor(path, competitionCorridor());

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.946, result.maximum_wheel_offset_m, 1.0e-12);
  EXPECT_NEAR(0.354, result.minimum_clearance_m, 1.0e-12);
}

TEST(WheelCorridor, IncludesRearAndFrontOuterWheelsForLateralOffset) {
  const std::vector<Point2d> path = {
      {-5.0, 0.2}, {0.0, 0.2}, {5.0, 0.2}, {10.0, 0.2}};

  const WheelCorridorResult result =
      estimateWheelCorridor(path, competitionCorridor());

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(1.146, result.maximum_wheel_offset_m, 1.0e-12);
  EXPECT_NEAR(0.154, result.minimum_clearance_m, 1.0e-12);
}

TEST(WheelCorridor, HeadingMisalignmentConsumesFrontWheelMargin) {
  const double yaw_rad = 0.1;
  const std::vector<Point2d> path = {
      {-5.0 * std::cos(yaw_rad), -5.0 * std::sin(yaw_rad)},
      {0.0, 0.0},
      {5.0 * std::cos(yaw_rad), 5.0 * std::sin(yaw_rad)},
      {10.0 * std::cos(yaw_rad), 10.0 * std::sin(yaw_rad)}};

  const WheelCorridorResult result =
      estimateWheelCorridor(path, competitionCorridor());

  ASSERT_TRUE(result.valid) << result.error;
  // 3*sin(0.1) + 0.946*cos(0.1): the front outer wheel is critical.
  EXPECT_NEAR(1.240774190293, result.maximum_wheel_offset_m, 1.0e-9);
  EXPECT_NEAR(0.059225809707, result.minimum_clearance_m, 1.0e-9);
}

TEST(WheelCorridor, RejectsInvalidDimensionsAndDegeneratePath) {
  WheelCorridorConfig config = competitionCorridor();
  config.lane_half_width_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(estimateWheelCorridor({{0.0, 0.0}, {1.0, 0.0}}, config)
                   .valid);

  config = competitionCorridor();
  config.vehicle_width_m = 0.0;
  EXPECT_FALSE(estimateWheelCorridor({{0.0, 0.0}, {1.0, 0.0}}, config)
                   .valid);

  config = competitionCorridor();
  EXPECT_FALSE(estimateWheelCorridor({{0.0, 0.0}, {0.0, 0.0}}, config)
                   .valid);
}

TEST(WheelCorridor, ClearanceSpeedLimitIsContinuousAndBounded) {
  LaneClearanceSpeedConfig config;
  config.recovery_start_m = 0.18;
  config.recovery_full_m = 0.05;
  config.minimum_speed_mps = 10.0 / 3.6;

  const LaneClearanceSpeedLimit comfortable =
      computeLaneClearanceSpeedLimit(0.30, 58.0 / 3.6, config);
  const LaneClearanceSpeedLimit midpoint =
      computeLaneClearanceSpeedLimit(0.115, 58.0 / 3.6, config);
  const LaneClearanceSpeedLimit critical =
      computeLaneClearanceSpeedLimit(0.05, 58.0 / 3.6, config);

  ASSERT_TRUE(comfortable.valid) << comfortable.error;
  ASSERT_TRUE(midpoint.valid) << midpoint.error;
  ASSERT_TRUE(critical.valid) << critical.error;
  EXPECT_NEAR(0.0, comfortable.urgency, 1.0e-12);
  EXPECT_NEAR(58.0 / 3.6, comfortable.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(0.5, midpoint.urgency, 1.0e-12);
  EXPECT_NEAR(34.0 / 3.6, midpoint.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(1.0, critical.urgency, 1.0e-12);
  EXPECT_NEAR(10.0 / 3.6, critical.speed_limit_mps, 1.0e-12);
}

TEST(WheelCorridor, PositiveClearanceDoesNotUndercutCurveMinimumSpeed) {
  LaneClearanceSpeedConfig config;
  config.recovery_start_m = 0.18;
  config.recovery_full_m = 0.0;
  config.minimum_speed_mps = 10.0 / 3.6;

  const LaneClearanceSpeedLimit positive =
      computeLaneClearanceSpeedLimit(0.048, 59.0 / 3.6, config);
  const LaneClearanceSpeedLimit contact =
      computeLaneClearanceSpeedLimit(0.0, 59.0 / 3.6, config);

  ASSERT_TRUE(positive.valid) << positive.error;
  ASSERT_TRUE(contact.valid) << contact.error;
  EXPECT_GT(positive.speed_limit_mps, 15.0 / 3.6);
  EXPECT_NEAR(contact.speed_limit_mps, 10.0 / 3.6, 1.0e-12);
}

TEST(WheelCorridor, ClearanceSpeedLimitRejectsInvertedThresholds) {
  LaneClearanceSpeedConfig config;
  config.recovery_start_m = 0.05;
  config.recovery_full_m = 0.18;
  config.minimum_speed_mps = 10.0 / 3.6;

  EXPECT_FALSE(
      computeLaneClearanceSpeedLimit(0.1, 58.0 / 3.6, config).valid);
}

TEST(WheelCorridor, HeadingErrorSpeedLimitActsBeforeLargeHeadingLag) {
  HeadingErrorSpeedConfig config;
  config.recovery_start_rad = 4.0 * M_PI / 180.0;
  config.recovery_full_rad = 10.0 * M_PI / 180.0;
  config.minimum_speed_mps = 10.0 / 3.6;

  const HeadingErrorSpeedLimit aligned = computeHeadingErrorSpeedLimit(
      2.0 * M_PI / 180.0, 20.0 / 3.6, config);
  const HeadingErrorSpeedLimit midpoint = computeHeadingErrorSpeedLimit(
      7.0 * M_PI / 180.0, 20.0 / 3.6, config);
  const HeadingErrorSpeedLimit lagging = computeHeadingErrorSpeedLimit(
      -12.0 * M_PI / 180.0, 20.0 / 3.6, config);

  ASSERT_TRUE(aligned.valid) << aligned.error;
  ASSERT_TRUE(midpoint.valid) << midpoint.error;
  ASSERT_TRUE(lagging.valid) << lagging.error;
  EXPECT_NEAR(0.0, aligned.urgency, 1.0e-12);
  EXPECT_NEAR(20.0 / 3.6, aligned.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(0.5, midpoint.urgency, 1.0e-12);
  EXPECT_NEAR(15.0 / 3.6, midpoint.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(1.0, lagging.urgency, 1.0e-12);
  EXPECT_NEAR(10.0 / 3.6, lagging.speed_limit_mps, 1.0e-12);
}

TEST(WheelCorridor, HeadingErrorSpeedLimitRejectsInvalidThresholds) {
  HeadingErrorSpeedConfig config;
  config.recovery_start_rad = 0.2;
  config.recovery_full_rad = 0.1;
  config.minimum_speed_mps = 10.0 / 3.6;

  EXPECT_FALSE(computeHeadingErrorSpeedLimit(0.1, 20.0 / 3.6, config)
                   .valid);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
