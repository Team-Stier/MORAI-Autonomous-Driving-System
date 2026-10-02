#include <cmath>
#include <deque>
#include <vector>

#include <gtest/gtest.h>

#include "control/tracking/controllers/lateral/mpc/mpc_lateral_controller.hpp"
#include "control/tracking/controllers/lateral/mpc/mpc_controller.hpp"

namespace control { namespace tracking {
namespace {

morai_mpc::MpcConfig testConfig() {
  morai_mpc::MpcConfig config;
  config.dt = 0.05;
  config.horizon = 16;
  config.optimizer_iterations = 4;
  config.resample_ds = 0.5;
  config.use_previous_control_rate_change = true;
  return config;
}

TEST(MpcLateralControllerTest, StraightCenteredPathProducesFiniteCommand) {
  MpcLateralController controller(testConfig());
  const std::vector<morai_mpc::Point2d> path{
      {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};

  const MpcLateralResult result = controller.calculate(path, 5.0, 0.0, 0.0, 0.05);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(result.steering_angle_rad, 0.0, 1.0e-9);
  EXPECT_TRUE(std::isfinite(result.solver_cost));
}

TEST(MpcLateralControllerTest, SweptPathCompensationIsZeroOnStraight) {
  morai_mpc::MpcConfig config = testConfig();
  config.swept_path_compensation_enabled = true;
  const std::vector<morai_mpc::Point2d> path{
      {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};

  const SweptPathCompensationResult result =
      compensateSweptWheelPath(path, config);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(result.maximum_offset_m, 0.0, 1.0e-12);
  ASSERT_EQ(result.path.size(), path.size());
  for (std::size_t index = 0U; index < path.size(); ++index) {
    EXPECT_NEAR(result.path[index].x, path[index].x, 1.0e-12);
    EXPECT_NEAR(result.path[index].y, path[index].y, 1.0e-12);
  }
}

TEST(MpcLateralControllerTest, SweptPathCompensationMovesTightArcInward) {
  morai_mpc::MpcConfig config = testConfig();
  config.swept_path_compensation_enabled = true;
  config.swept_path_activation_start_curvature = 0.005;
  config.swept_path_activation_full_curvature = 0.020;
  config.swept_path_curvature_smoothing_window = 1;
  const double radius_m = 9.25;
  std::vector<morai_mpc::Point2d> path;
  for (int index = 0; index <= 12; ++index) {
    const double angle = 0.04 * static_cast<double>(index);
    path.push_back({radius_m * std::sin(angle),
                    radius_m * (1.0 - std::cos(angle))});
  }

  const SweptPathCompensationResult result =
      compensateSweptWheelPath(path, config);

  ASSERT_TRUE(result.valid) << result.error;
  const double expected_offset =
      config.wheelbase * config.wheelbase /
      (4.0 * radius_m + 2.0 * config.swept_path_vehicle_width);
  EXPECT_NEAR(result.maximum_offset_m, expected_offset, 2.0e-3);
  EXPECT_GT(result.path[6].y, path[6].y);
}

TEST(MpcLateralControllerTest, SweptPathCompensationMirrorsRightArc) {
  morai_mpc::MpcConfig config = testConfig();
  config.swept_path_compensation_enabled = true;
  config.swept_path_curvature_smoothing_window = 1;
  std::vector<morai_mpc::Point2d> path;
  for (int index = 0; index <= 12; ++index) {
    const double angle = 0.04 * static_cast<double>(index);
    path.push_back({9.25 * std::sin(angle),
                    -9.25 * (1.0 - std::cos(angle))});
  }

  const SweptPathCompensationResult result =
      compensateSweptWheelPath(path, config);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.maximum_offset_m, 0.20);
  EXPECT_LT(result.path[6].y, path[6].y);
}

TEST(MpcLateralControllerTest, FiltersGpsCteStepOnlyOnHighSpeedStraight) {
  morai_mpc::MpcConfig config = testConfig();
  config.straight_cte_filter_time_constant = 0.10;
  config.straight_cte_filter_minimum_speed = 45.0 / 3.6;
  config.straight_cte_filter_maximum_curvature = 0.003;
  MpcLateralController controller(config);
  ASSERT_TRUE(controller.calculate(
      {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}},
      15.0, 0.0, 0.0, 0.033).valid);

  const MpcLateralResult filtered = controller.calculate(
      {{0.0, 0.15}, {5.0, 0.15}, {10.0, 0.15}, {20.0, 0.15}},
      15.0, 0.0, 0.0, 0.033);

  ASSERT_TRUE(filtered.valid) << filtered.error;
  EXPECT_NEAR(filtered.raw_cross_track_error_m, -0.15, 1.0e-9);
  EXPECT_LT(std::abs(filtered.cross_track_error_m),
            std::abs(filtered.raw_cross_track_error_m));
}

TEST(MpcLateralControllerTest, DoesNotFilterCteAtLowSpeed) {
  morai_mpc::MpcConfig config = testConfig();
  config.straight_cte_filter_time_constant = 0.10;
  config.straight_cte_filter_minimum_speed = 45.0 / 3.6;
  MpcLateralController controller(config);
  ASSERT_TRUE(controller.calculate(
      {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}},
      5.0, 0.0, 0.0, 0.033).valid);

  const MpcLateralResult unfiltered = controller.calculate(
      {{0.0, 0.15}, {5.0, 0.15}, {10.0, 0.15}, {20.0, 0.15}},
      5.0, 0.0, 0.0, 0.033);

  ASSERT_TRUE(unfiltered.valid) << unfiltered.error;
  EXPECT_NEAR(unfiltered.cross_track_error_m,
              unfiltered.raw_cross_track_error_m, 1.0e-12);
}

TEST(MpcLateralControllerTest, RejectsImpossibleHighSpeedStraightCteJump) {
  morai_mpc::MpcConfig config = testConfig();
  config.straight_cte_filter_time_constant = 0.0;
  config.straight_cte_maximum_rate = 1.0;
  config.straight_cte_filter_minimum_speed = 45.0 / 3.6;
  config.straight_cte_filter_maximum_curvature = 0.003;
  MpcLateralController controller(config);
  ASSERT_TRUE(controller.calculate(
      {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}},
      15.0, 0.0, 0.0, 0.033).valid);

  const MpcLateralResult limited = controller.calculate(
      {{0.0, 0.13}, {5.0, 0.13}, {10.0, 0.13}, {20.0, 0.13}},
      15.0, 0.0, 0.0, 0.033);

  ASSERT_TRUE(limited.valid) << limited.error;
  EXPECT_NEAR(limited.raw_cross_track_error_m, -0.13, 1.0e-9);
  EXPECT_NEAR(limited.cross_track_error_m, -0.033, 1.0e-9);
}

TEST(MpcLateralControllerTest, CteJumpLimiterReleasesBeforeCurve) {
  morai_mpc::MpcConfig config = testConfig();
  config.straight_cte_maximum_rate = 1.0;
  config.straight_cte_filter_minimum_speed = 45.0 / 3.6;
  config.straight_cte_filter_maximum_curvature = 0.003;
  MpcLateralController controller(config);
  ASSERT_TRUE(controller.calculate(
      {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}},
      15.0, 0.0, 0.0, 0.033).valid);

  const MpcLateralResult curve = controller.calculate(
      {{0.0, 0.13}, {2.0, 0.18}, {4.0, 0.35}, {6.0, 0.70},
       {8.0, 1.25}, {10.0, 2.0}},
      15.0, 0.0, 0.0, 0.033);

  ASSERT_TRUE(curve.valid) << curve.error;
  EXPECT_NEAR(curve.cross_track_error_m,
              curve.raw_cross_track_error_m, 1.0e-12);
}

TEST(MpcLateralControllerTest,
     SpatialStraightFitDoesNotKeepStaleCteSign) {
  morai_mpc::MpcConfig config = testConfig();
  config.solver = "osqp";
  config.straight_cte_maximum_rate = 0.0;
  config.straight_spatial_fit_enabled = true;
  config.straight_spatial_fit_start_speed = 1.0;
  config.straight_spatial_fit_full_speed = 2.0;
  config.straight_spatial_fit_full_curvature = 0.001;
  config.straight_spatial_fit_off_curvature = 0.006;
  config.straight_spatial_fit_base_preview = 4.0;
  config.straight_spatial_fit_maximum_preview = 10.0;
  MpcLateralController controller(config);

  const MpcLateralResult right_path = controller.calculate(
      {{0.0, -0.10}, {5.0, -0.10}, {10.0, -0.10}, {20.0, -0.10}},
      15.0, 0.0, 0.0, 0.033);
  const MpcLateralResult left_path = controller.calculate(
      {{0.0, 0.10}, {5.0, 0.10}, {10.0, 0.10}, {20.0, 0.10}},
      15.0, 0.0, 0.0, 0.033);

  ASSERT_TRUE(right_path.valid) << right_path.error;
  ASSERT_TRUE(left_path.valid) << left_path.error;
  EXPECT_GT(right_path.cross_track_error_m, 0.09);
  EXPECT_LT(left_path.cross_track_error_m, -0.09);
  EXPECT_NEAR(right_path.cross_track_error_m,
              right_path.raw_cross_track_error_m, 1.0e-9);
  EXPECT_NEAR(left_path.cross_track_error_m,
              left_path.raw_cross_track_error_m, 1.0e-9);
}

TEST(MpcLateralControllerTest,
     SpatialQuadraticFitEstimatesCurveTangentWithoutChordBias) {
  morai_mpc::MpcConfig raw_config = testConfig();
  raw_config.solver = "osqp";
  morai_mpc::MpcConfig fitted_config = raw_config;
  fitted_config.straight_spatial_fit_enabled = true;
  fitted_config.straight_spatial_fit_start_speed = 1.0;
  fitted_config.straight_spatial_fit_full_speed = 2.0;
  fitted_config.straight_spatial_fit_full_curvature = 0.001;
  fitted_config.straight_spatial_fit_off_curvature = 0.060;
  fitted_config.straight_spatial_fit_base_preview = 8.0;
  fitted_config.straight_spatial_fit_maximum_preview = 8.0;
  std::vector<morai_mpc::Point2d> curve;
  for (int index = 0; index <= 80; ++index) {
    const double angle = 0.01 * static_cast<double>(index);
    curve.push_back({30.0 * std::sin(angle),
                     30.0 * (1.0 - std::cos(angle))});
  }
  MpcLateralController raw_controller(raw_config);
  MpcLateralController fitted_controller(fitted_config);

  const MpcLateralResult raw = raw_controller.calculate(
      curve, 15.0, 0.0, 0.0, 0.05);
  const MpcLateralResult fitted = fitted_controller.calculate(
      curve, 15.0, 0.0, 0.0, 0.05);

  ASSERT_TRUE(raw.valid) << raw.error;
  ASSERT_TRUE(fitted.valid) << fitted.error;
  EXPECT_LT(std::abs(fitted.heading_error_rad),
            std::abs(raw.heading_error_rad) * 0.6);
}

TEST(MpcLateralControllerTest, SmoothsSteeringRateOnlyOnHighSpeedStraight) {
  morai_mpc::MpcConfig config = testConfig();
  config.straight_steering_rate_filter_time_constant = 0.12;
  config.straight_steering_rate_filter_minimum_speed = 45.0 / 3.6;
  config.straight_steering_rate_filter_maximum_curvature = 0.003;
  MpcLateralController controller(config);
  ASSERT_TRUE(controller.calculate(
      {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}},
      15.0, 0.0, 0.0, 0.033).valid);

  const MpcLateralResult filtered = controller.calculate(
      {{0.0, 0.2}, {5.0, 0.2}, {10.0, 0.2}, {20.0, 0.2}},
      15.0, 0.0, 0.0, 0.033);

  ASSERT_TRUE(filtered.valid) << filtered.error;
  EXPECT_LT(std::abs(filtered.steering_rate_rad_per_sec),
            std::abs(filtered.raw_steering_rate_rad_per_sec));
}

TEST(MpcLateralControllerTest, BypassesSteeringRateFilterOnCurve) {
  morai_mpc::MpcConfig config = testConfig();
  config.straight_steering_rate_filter_time_constant = 0.12;
  config.straight_steering_rate_filter_minimum_speed = 45.0 / 3.6;
  config.straight_steering_rate_filter_maximum_curvature = 0.003;
  MpcLateralController controller(config);
  ASSERT_TRUE(controller.calculate(
      {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}},
      15.0, 0.0, 0.0, 0.033).valid);

  const MpcLateralResult curve = controller.calculate(
      {{0.0, 0.0}, {2.0, 0.1}, {4.0, 0.4}, {6.0, 0.9}, {8.0, 1.6}},
      15.0, 0.0, 0.0, 0.033);

  ASSERT_TRUE(curve.valid) << curve.error;
  EXPECT_NEAR(curve.steering_rate_rad_per_sec,
              curve.raw_steering_rate_rad_per_sec, 1.0e-12);
}

TEST(MpcLateralControllerTest, IntegratesRateUsingActualControlPeriod) {
  morai_mpc::MpcConfig config = testConfig();
  config.optimizer_initial_step = morai_mpc::deg2rad(20.0);
  MpcLateralController controller(config);
  const std::vector<morai_mpc::Point2d> path{
      {0.0, 1.0}, {5.0, 1.0}, {10.0, 1.0}, {20.0, 1.0}};

  const MpcLateralResult result = controller.calculate(path, 5.0, 0.0, 0.0, 0.02);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(result.steering_angle_rad,
              result.steering_rate_rad_per_sec * 0.02, 1.0e-12);
  EXPECT_LE(std::abs(result.steering_angle_rad), config.max_steering);
}

TEST(MpcLateralControllerTest, RejectsInsufficientPath) {
  MpcLateralController controller(testConfig());
  const MpcLateralResult result =
      controller.calculate({{0.0, 0.0}}, 5.0, 0.0, 0.0, 0.05);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.error, "INVALID_MPC_INPUT");
}

TEST(MpcLateralControllerTest, BothSolverSelectionsProduceBoundedCommands) {
  for (const std::string solver : {"osqp", "projected_gradient", "coordinate_search"}) {
    morai_mpc::MpcConfig config = testConfig();
    config.solver = solver;
    MpcLateralController controller(config);
    const MpcLateralResult result = controller.calculate(
        {{0.0, 0.8}, {5.0, 0.8}, {10.0, 0.4}, {20.0, 0.0}}, 5.0, 0.0, 0.0,
        0.05);
    ASSERT_TRUE(result.valid) << solver << ": " << result.error;
    EXPECT_LE(std::abs(result.steering_angle_rad), config.max_steering);
    EXPECT_TRUE(std::isfinite(result.solver_cost));
  }
}

TEST(MpcLateralControllerTest, LinearOsqpCommandsIntoTightIoniq5Curve) {
  morai_mpc::MpcConfig config = testConfig();
  config.solver = "osqp";
  config.wheelbase = 3.0;
  config.max_steering = morai_mpc::deg2rad(40.0);
  config.max_steering_rate = morai_mpc::deg2rad(60.0);
  config.optimizer_model_actuator = true;
  config.actuator_delay = 0.10;
  config.actuator_time_constant = 0.15;
  config.horizon = 24;
  config.w_lateral = 12.0;
  config.w_heading = 7.0;
  config.w_steering = 0.7;
  config.w_rate = 0.24;
  config.w_rate_change = 4.8;
  std::vector<morai_mpc::Point2d> path;
  for (int index = 0; index <= 40; ++index) {
    const double angle = 0.025 * static_cast<double>(index);
    path.push_back({9.25 * std::sin(angle),
                    9.25 * (1.0 - std::cos(angle))});
  }

  MpcLateralController controller(config);
  const MpcLateralResult result =
      controller.calculate(path, 10.0, 0.0, 0.0, 0.05);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.raw_steering_rate_rad_per_sec,
            morai_mpc::deg2rad(5.0));
  EXPECT_LE(result.raw_steering_rate_rad_per_sec,
            morai_mpc::deg2rad(60.0));
}

TEST(MpcLateralControllerTest, CurveUsesYawRateEstimatedEffectiveSteering) {
  morai_mpc::MpcConfig commanded_config = testConfig();
  commanded_config.solver = "osqp";
  commanded_config.optimizer_model_actuator = true;
  commanded_config.actuator_delay = 0.10;
  commanded_config.actuator_time_constant = 0.15;
  commanded_config.curve_yaw_rate_steering_estimate_minimum_speed = 1.0;
  morai_mpc::MpcConfig estimated_config = commanded_config;
  estimated_config.curve_yaw_rate_steering_estimate_gain = 1.0;
  std::vector<morai_mpc::Point2d> right_curve;
  for (int index = 0; index <= 40; ++index) {
    const double angle = 0.025 * static_cast<double>(index);
    right_curve.push_back({9.25 * std::sin(angle),
                           -9.25 * (1.0 - std::cos(angle))});
  }
  const double speed_mps = 10.0;
  const double commanded_steering = morai_mpc::deg2rad(-20.0);
  const double effective_steering = morai_mpc::deg2rad(-10.0);
  const double measured_yaw_rate =
      speed_mps / estimated_config.wheelbase *
      std::tan(effective_steering);
  MpcLateralController commanded_controller(commanded_config);
  MpcLateralController estimated_controller(estimated_config);

  const MpcLateralResult commanded = commanded_controller.calculate(
      right_curve, speed_mps, measured_yaw_rate, commanded_steering, 0.05);
  const MpcLateralResult estimated = estimated_controller.calculate(
      right_curve, speed_mps, measured_yaw_rate, commanded_steering, 0.05);

  ASSERT_TRUE(commanded.valid) << commanded.error;
  ASSERT_TRUE(estimated.valid) << estimated.error;
  EXPECT_LT(estimated.raw_steering_rate_rad_per_sec,
            commanded.raw_steering_rate_rad_per_sec -
                morai_mpc::deg2rad(1.0));
}

TEST(MpcLateralControllerTest, StraightIgnoresYawRateSteeringEstimate) {
  morai_mpc::MpcConfig disabled_config = testConfig();
  disabled_config.solver = "osqp";
  morai_mpc::MpcConfig enabled_config = disabled_config;
  enabled_config.curve_yaw_rate_steering_estimate_gain = 1.0;
  enabled_config.curve_yaw_rate_steering_estimate_minimum_speed = 1.0;
  const std::vector<morai_mpc::Point2d> straight{
      {0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {40.0, 0.0}};
  MpcLateralController disabled_controller(disabled_config);
  MpcLateralController enabled_controller(enabled_config);

  const MpcLateralResult disabled = disabled_controller.calculate(
      straight, 15.0, 0.0, morai_mpc::deg2rad(5.0), 0.05);
  const MpcLateralResult enabled = enabled_controller.calculate(
      straight, 15.0, 0.0, morai_mpc::deg2rad(5.0), 0.05);

  ASSERT_TRUE(disabled.valid) << disabled.error;
  ASSERT_TRUE(enabled.valid) << enabled.error;
  EXPECT_NEAR(enabled.raw_steering_rate_rad_per_sec,
              disabled.raw_steering_rate_rad_per_sec, 1.0e-12);
}

TEST(MpcLateralControllerTest,
     ModerateCurveKeepsYawRateSteeringObservationDisabled) {
  morai_mpc::MpcConfig commanded_config = testConfig();
  commanded_config.solver = "osqp";
  commanded_config.curve_yaw_rate_steering_estimate_gain = 0.0;
  commanded_config.curve_yaw_rate_steering_estimate_activation_start_curvature =
      0.030;
  commanded_config.curve_yaw_rate_steering_estimate_activation_full_curvature =
      0.050;
  morai_mpc::MpcConfig observed_config = commanded_config;
  observed_config.curve_yaw_rate_steering_estimate_gain = 1.0;
  observed_config.curve_yaw_rate_steering_estimate_minimum_speed = 1.0;
  std::vector<morai_mpc::Point2d> moderate_curve;
  for (int index = 0; index <= 80; ++index) {
    const double angle = 0.01 * static_cast<double>(index);
    moderate_curve.push_back({50.0 * std::sin(angle),
                              50.0 * (1.0 - std::cos(angle))});
  }
  MpcLateralController commanded_controller(commanded_config);
  MpcLateralController observed_controller(observed_config);
  const double speed_mps = 12.0;
  const double measured_yaw_rate =
      speed_mps / observed_config.wheelbase *
      std::tan(morai_mpc::deg2rad(4.0));

  const MpcLateralResult commanded = commanded_controller.calculate(
      moderate_curve, speed_mps, measured_yaw_rate,
      morai_mpc::deg2rad(8.0), 0.05);
  const MpcLateralResult observed = observed_controller.calculate(
      moderate_curve, speed_mps, measured_yaw_rate,
      morai_mpc::deg2rad(8.0), 0.05);

  ASSERT_TRUE(commanded.valid) << commanded.error;
  ASSERT_TRUE(observed.valid) << observed.error;
  EXPECT_NEAR(observed.yaw_rate_steering_estimate_blend, 0.0, 1.0e-12);
  EXPECT_NEAR(observed.raw_steering_rate_rad_per_sec,
              commanded.raw_steering_rate_rad_per_sec, 1.0e-12);
}

TEST(MpcLateralControllerTest, CurveSteeringObservationBlendRampsSmoothly) {
  morai_mpc::MpcConfig config = testConfig();
  config.solver = "osqp";
  config.curve_yaw_rate_steering_estimate_gain = 1.0;
  config.curve_yaw_rate_steering_estimate_minimum_speed = 1.0;
  config.curve_yaw_rate_steering_estimate_activation_start_curvature = 0.010;
  config.curve_yaw_rate_steering_estimate_activation_full_curvature = 0.020;
  config.curve_yaw_rate_steering_estimate_blend_time_constant = 0.15;
  std::vector<morai_mpc::Point2d> tight_curve;
  for (int index = 0; index <= 60; ++index) {
    const double angle = 0.02 * static_cast<double>(index);
    tight_curve.push_back({20.0 * std::sin(angle),
                           20.0 * (1.0 - std::cos(angle))});
  }
  MpcLateralController controller(config);

  const MpcLateralResult first = controller.calculate(
      tight_curve, 10.0, 0.4, morai_mpc::deg2rad(8.0), 0.05);
  const MpcLateralResult second = controller.calculate(
      tight_curve, 10.0, 0.4, morai_mpc::deg2rad(8.0), 0.05);

  ASSERT_TRUE(first.valid) << first.error;
  ASSERT_TRUE(second.valid) << second.error;
  EXPECT_GT(first.yaw_rate_steering_estimate_blend, 0.0);
  EXPECT_LT(first.yaw_rate_steering_estimate_blend, 1.0);
  EXPECT_GT(second.yaw_rate_steering_estimate_blend,
            first.yaw_rate_steering_estimate_blend);
  EXPECT_LT(second.yaw_rate_steering_estimate_blend, 1.0);
}

TEST(MpcLateralControllerTest,
     StraightYawRateSteeringObservationCompensatesActuatorLag) {
  morai_mpc::MpcConfig commanded_config = testConfig();
  commanded_config.solver = "osqp";
  morai_mpc::MpcConfig observed_config = commanded_config;
  observed_config.straight_yaw_rate_steering_estimate_gain = 1.0;
  observed_config.straight_yaw_rate_steering_estimate_minimum_speed = 1.0;
  observed_config.straight_yaw_rate_steering_estimate_filter_time_constant =
      0.10;
  const std::vector<morai_mpc::Point2d> straight{
      {0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {40.0, 0.0}};
  const double speed_mps = 15.0;
  const double effective_steering = morai_mpc::deg2rad(1.5);
  const double measured_yaw_rate =
      speed_mps / observed_config.wheelbase * std::tan(effective_steering);
  MpcLateralController commanded_controller(commanded_config);
  MpcLateralController observed_controller(observed_config);

  const MpcLateralResult commanded = commanded_controller.calculate(
      straight, speed_mps, measured_yaw_rate, 0.0, 0.05);
  const MpcLateralResult observed = observed_controller.calculate(
      straight, speed_mps, measured_yaw_rate, 0.0, 0.05);

  ASSERT_TRUE(commanded.valid) << commanded.error;
  ASSERT_TRUE(observed.valid) << observed.error;
  EXPECT_LT(observed.raw_steering_rate_rad_per_sec,
            commanded.raw_steering_rate_rad_per_sec -
                morai_mpc::deg2rad(1.0));
}

TEST(MpcLateralControllerTest,
     StraightDynamicModelDampsYawWithoutFakingSteeringAngle) {
  morai_mpc::MpcConfig kinematic_config = testConfig();
  kinematic_config.solver = "osqp";
  kinematic_config.optimizer_model_actuator = true;
  kinematic_config.actuator_delay = 0.10;
  kinematic_config.actuator_time_constant = 0.15;
  morai_mpc::MpcConfig dynamic_config = kinematic_config;
  dynamic_config.straight_dynamic_model_enabled = true;
  dynamic_config.straight_dynamic_model_minimum_speed = 5.0;
  dynamic_config.straight_dynamic_model_maximum_curvature = 0.003;
  dynamic_config.straight_dynamic_yaw_rate_weight = 4.0;
  std::vector<morai_mpc::Point2d> points;
  for (int index = 0; index <= 80; ++index) {
    points.push_back({static_cast<double>(index), 0.0});
  }
  const morai_mpc::ReferencePath path(
      points, false, morai_mpc::CoordinateFrame::Global);
  const morai_mpc::VehicleState vehicle{
      0.0, 0.0, 0.0, 15.0, 0.0, 0.0, morai_mpc::deg2rad(3.0)};
  morai_mpc::MpcController kinematic_controller(kinematic_config);
  morai_mpc::MpcController dynamic_controller(dynamic_config);

  const morai_mpc::MpcCommand kinematic =
      kinematic_controller.compute(vehicle, path, dynamic_config.dt);
  const morai_mpc::MpcCommand dynamic =
      dynamic_controller.compute(vehicle, path, dynamic_config.dt);

  ASSERT_TRUE(kinematic.success) << kinematic.message;
  ASSERT_TRUE(dynamic.success) << dynamic.message;
  EXPECT_LT(dynamic.steering_rate,
            kinematic.steering_rate - morai_mpc::deg2rad(1.0));
}

TEST(MpcLateralControllerTest, StraightDynamicModelLeavesCurvesKinematic) {
  morai_mpc::MpcConfig disabled_config = testConfig();
  disabled_config.solver = "osqp";
  morai_mpc::MpcConfig enabled_config = disabled_config;
  enabled_config.straight_dynamic_model_enabled = true;
  enabled_config.straight_dynamic_model_minimum_speed = 5.0;
  enabled_config.straight_dynamic_model_maximum_curvature = 0.003;
  std::vector<morai_mpc::Point2d> curve;
  for (int index = 0; index <= 40; ++index) {
    const double angle = 0.025 * static_cast<double>(index);
    curve.push_back({9.25 * std::sin(angle),
                     9.25 * (1.0 - std::cos(angle))});
  }
  const morai_mpc::ReferencePath path(
      curve, false, morai_mpc::CoordinateFrame::Global);
  const morai_mpc::VehicleState vehicle{0.0, 0.0, 0.0, 10.0, 0.0};
  morai_mpc::MpcController disabled_controller(disabled_config);
  morai_mpc::MpcController enabled_controller(enabled_config);

  const morai_mpc::MpcCommand disabled =
      disabled_controller.compute(vehicle, path, enabled_config.dt);
  const morai_mpc::MpcCommand enabled =
      enabled_controller.compute(vehicle, path, enabled_config.dt);

  ASSERT_TRUE(disabled.success) << disabled.message;
  ASSERT_TRUE(enabled.success) << enabled.message;
  EXPECT_NEAR(enabled.steering_rate, disabled.steering_rate, 1.0e-12);
}

TEST(MpcLateralControllerTest, StraightDynamicModelClosedLoopConverges) {
  morai_mpc::MpcConfig config = testConfig();
  config.solver = "osqp";
  config.horizon = 24;
  config.w_lateral = 12.0;
  config.w_heading = 7.0;
  config.w_steering = 0.7;
  config.w_rate = 0.24;
  config.w_rate_change = 4.8;
  config.straight_dynamic_model_enabled = true;
  config.straight_dynamic_model_minimum_speed = 5.0;
  config.straight_dynamic_model_maximum_curvature = 0.003;
  config.straight_dynamic_sideslip_weight = 1.0;
  config.straight_dynamic_yaw_rate_weight = 4.0;
  std::vector<morai_mpc::Point2d> points;
  for (int index = 0; index <= 260; ++index) {
    points.push_back({static_cast<double>(index), 0.0});
  }
  const morai_mpc::ReferencePath path(
      points, false, morai_mpc::CoordinateFrame::Global);
  morai_mpc::MpcController controller(config);
  morai_mpc::VehicleState vehicle{
      0.0, 0.20, morai_mpc::deg2rad(1.0), 15.0, 0.0, 0.0, 0.0};
  double sideslip = 0.0;
  double squared_lateral_error = 0.0;
  for (int iteration = 0; iteration < 200; ++iteration) {
    vehicle.lateral_velocity = vehicle.speed * std::tan(sideslip);
    const morai_mpc::MpcCommand command =
        controller.compute(vehicle, path, config.dt);
    ASSERT_TRUE(command.success) << command.message;
    vehicle.steering = morai_mpc::clamp(
        vehicle.steering + config.dt * command.steering_rate,
        -config.max_steering, config.max_steering);
    const double speed = vehicle.speed;
    const double mass = config.dynamic_mass;
    const double inertia = config.dynamic_yaw_inertia;
    const double cf = config.dynamic_front_cornering_stiffness;
    const double cr = config.dynamic_rear_cornering_stiffness;
    const double a = config.dynamic_front_axle_to_cg;
    const double b = config.dynamic_rear_axle_to_cg;
    const double sideslip_rate =
        -2.0 * (cf + cr) / (mass * speed) * sideslip +
        (-1.0 - 2.0 * (cf * a - cr * b) /
                    (mass * speed * speed)) *
            vehicle.yaw_rate +
        2.0 * cf / (mass * speed) * vehicle.steering;
    const double yaw_acceleration =
        -2.0 * (cf * a - cr * b) / inertia * sideslip -
        2.0 * (cf * a * a + cr * b * b) /
            (inertia * speed) * vehicle.yaw_rate +
        2.0 * cf * a / inertia * vehicle.steering;
    sideslip += config.dt * sideslip_rate;
    vehicle.yaw_rate += config.dt * yaw_acceleration;
    vehicle.yaw = morai_mpc::normalizeAngle(
        vehicle.yaw + config.dt * vehicle.yaw_rate);
    vehicle.x += config.dt * speed * std::cos(vehicle.yaw + sideslip);
    vehicle.y += config.dt * speed * std::sin(vehicle.yaw + sideslip);
    squared_lateral_error += vehicle.y * vehicle.y;
  }

  EXPECT_LT(std::abs(vehicle.y), 0.05);
  EXPECT_LT(std::sqrt(squared_lateral_error / 200.0), 0.10);
  EXPECT_LT(std::abs(vehicle.yaw_rate), morai_mpc::deg2rad(0.5));
}

TEST(MpcLateralControllerTest, LinearOsqpStabilizesHighSpeedStraightError) {
  morai_mpc::MpcConfig config = testConfig();
  config.solver = "osqp";
  config.optimizer_model_actuator = true;
  config.actuator_delay = 0.10;
  config.actuator_time_constant = 0.15;
  config.horizon = 24;
  config.w_rate_change = 4.8;
  config.w_lateral = 12.0;
  config.w_heading = 7.0;
  config.w_steering = 0.7;
  config.w_rate = 0.24;
  std::vector<morai_mpc::Point2d> points;
  for (int index = 0; index <= 240; ++index) {
    points.push_back({static_cast<double>(index), 0.0});
  }
  morai_mpc::ReferencePath path(
      points, false, morai_mpc::CoordinateFrame::Global);
  morai_mpc::MpcController controller(config);
  morai_mpc::VehicleState vehicle{0.0, 0.20, 0.0, 15.0, 0.0};
  std::deque<double> delay_line(2U, 0.0);
  double applied_rate = 0.0;
  int previous_sign = 0;
  int reversals = 0;
  double squared_lateral_error = 0.0;
  for (int iteration = 0; iteration < 200; ++iteration) {
    const morai_mpc::MpcCommand command =
        controller.compute(vehicle, path, config.dt);
    ASSERT_TRUE(command.success) << command.message;
    const double delayed_rate = delay_line.front();
    delay_line.pop_front();
    delay_line.push_back(command.steering_rate);
    const double alpha =
        1.0 - std::exp(-config.dt / config.actuator_time_constant);
    applied_rate += alpha * (delayed_rate - applied_rate);
    vehicle.steering = morai_mpc::clamp(
        vehicle.steering + config.dt * applied_rate,
        -config.max_steering, config.max_steering);
    vehicle.yaw = morai_mpc::normalizeAngle(
        vehicle.yaw + config.dt * vehicle.speed / config.wheelbase *
                          std::tan(vehicle.steering));
    vehicle.x += config.dt * vehicle.speed * std::cos(vehicle.yaw);
    vehicle.y += config.dt * vehicle.speed * std::sin(vehicle.yaw);
    squared_lateral_error += vehicle.y * vehicle.y;
    const int sign = command.steering_rate > morai_mpc::deg2rad(1.0)
                         ? 1
                         : command.steering_rate < -morai_mpc::deg2rad(1.0)
                               ? -1
                               : 0;
    if (iteration >= 20 && sign != 0 && previous_sign != 0 &&
        sign != previous_sign) {
      ++reversals;
    }
    if (sign != 0) {
      previous_sign = sign;
    }
  }
  EXPECT_LT(std::abs(vehicle.y), 0.05);
  EXPECT_LT(std::sqrt(squared_lateral_error / 200.0), 0.10);
  EXPECT_LE(reversals, 8);
}

TEST(MpcLateralControllerTest, DampsMeasuredYawRateOnStraightPath) {
  morai_mpc::MpcConfig config = testConfig();
  config.solver = "osqp";
  config.yaw_rate_damping_gain = 0.35;
  MpcLateralController controller(config);
  const MpcLateralResult result = controller.calculate(
      {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}}, 10.0, 0.1,
      0.0, 0.05);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_LT(result.steering_rate_rad_per_sec, 0.0);
  EXPECT_NEAR(result.steering_rate_rad_per_sec, -0.035, 0.005);
}

}  // namespace
} }  // namespace control::tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
