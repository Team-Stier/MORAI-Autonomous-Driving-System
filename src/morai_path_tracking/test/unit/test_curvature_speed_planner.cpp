#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "morai_path_tracking/planning/curvature_speed_planner.hpp"

namespace morai_path_tracking {
namespace {

CurvatureSpeedPlannerConfig baseConfig() {
  CurvatureSpeedPlannerConfig config;
  config.configured_target_speed_mps = 10.0;
  config.minimum_curve_speed_mps = 0.0;
  config.maximum_lateral_acceleration_mps2 = 1.0;
  config.preview_distance_m = 30.0;
  config.lookahead_curvature_preview_distance_m = 8.0;
  config.curvature_sample_spacing_m = 2.0;
  config.curve_approach_deceleration_mps2 = 3.0;
  config.curvature_epsilon_m_inv = 0.001;
  config.target_speed_acceleration_limit_mps2 = 1.0;
  config.curve_target_speed_acceleration_limit_mps2 = 1.0;
  config.target_speed_deceleration_limit_mps2 = 2.0;
  return config;
}

std::vector<Point2d> straightPath() {
  return {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
}

std::vector<Point2d> straightThenQuarterCircle(double straight_length_m,
                                               double radius_m) {
  std::vector<Point2d> path;
  for (double x = 0.0; x <= straight_length_m; x += 0.5) {
    path.push_back({x, 0.0});
  }

  constexpr double kPi = 3.14159265358979323846;
  const double angle_step = 0.5 / radius_m;
  for (double angle = -0.5 * kPi + angle_step; angle < 0.0;
       angle += angle_step) {
    path.push_back(
        {straight_length_m + radius_m * std::cos(angle),
         radius_m + radius_m * std::sin(angle)});
  }
  path.push_back({straight_length_m + radius_m, radius_m});
  return path;
}

std::vector<Point2d> quarterCircle(double radius_m) {
  return straightThenQuarterCircle(0.0, radius_m);
}

CurvatureSpeedPlannerConfig highSpeedConfig() {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.configured_target_speed_mps = 55.0 / 3.6;
  config.minimum_curve_speed_mps = 12.0 / 3.6;
  config.maximum_lateral_acceleration_mps2 = 3.0;
  config.preview_distance_m = 45.0;
  config.lookahead_curvature_preview_distance_m = 8.0;
  config.curvature_sample_spacing_m = 2.0;
  config.curve_approach_deceleration_mps2 = 2.0;
  return config;
}

TEST(CurvatureSpeedPlanner, StraightPathKeepsConfiguredTarget) {
  CurvatureSpeedPlanner planner(baseConfig());

  const CurvatureSpeedPlan result = planner.update(straightPath(), 0.1);

  EXPECT_NEAR(0.0, result.preview_curvature_m_inv, 1.0e-12);
  EXPECT_NEAR(10.0, result.curvature_speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(10.0, result.raw_target_speed_mps, 1.0e-12);
  EXPECT_NEAR(10.0, result.target_speed_mps, 1.0e-12);
}

TEST(CurvatureSpeedPlanner, EstimatesCircumcircleCurvatureAndSpeedLimit) {
  CurvatureSpeedPlanner planner(baseConfig());

  const CurvatureSpeedPlan result = planner.update(quarterCircle(10.0), 0.1);

  EXPECT_NEAR(0.1, result.preview_curvature_m_inv, 5.0e-3);
  EXPECT_NEAR(std::sqrt(10.0), result.curvature_speed_limit_mps, 0.1);
  EXPECT_GT(result.raw_target_speed_mps, result.curvature_speed_limit_mps);
}

TEST(CurvatureSpeedPlanner, RespectsMinimumCurveSpeed) {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.minimum_curve_speed_mps = 2.0;
  config.curvature_sample_spacing_m = 0.2;
  config.curve_approach_deceleration_mps2 = 0.001;
  CurvatureSpeedPlanner planner(config);

  const CurvatureSpeedPlan result = planner.update(quarterCircle(1.0), 0.1);

  EXPECT_LT(result.curvature_speed_limit_mps,
            config.minimum_curve_speed_mps);
  EXPECT_GE(result.raw_target_speed_mps, 2.0);
  EXPECT_LT(result.raw_target_speed_mps, 2.001);
  EXPECT_NEAR(result.raw_target_speed_mps, result.target_speed_mps, 1.0e-12);
}

TEST(CurvatureSpeedPlanner, LimitsTargetDecreaseAndIncreaseByElapsedTime) {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.minimum_curve_speed_mps = 2.0;
  config.curvature_sample_spacing_m = 0.2;
  config.curve_approach_deceleration_mps2 = 0.001;
  CurvatureSpeedPlanner planner(config);
  ASSERT_NEAR(10.0, planner.update(straightPath(), 0.1).target_speed_mps,
              1.0e-9);

  const CurvatureSpeedPlan braking = planner.update(quarterCircle(1.0), 0.5);
  EXPECT_NEAR(2.0, braking.raw_target_speed_mps, 0.001);
  EXPECT_NEAR(9.0, braking.target_speed_mps, 1.0e-9);

  const CurvatureSpeedPlan accelerating = planner.update(straightPath(), 0.5);
  EXPECT_NEAR(10.0, accelerating.raw_target_speed_mps, 1.0e-9);
  EXPECT_NEAR(9.5, accelerating.target_speed_mps, 1.0e-9);
}

TEST(CurvatureSpeedPlanner,
     UsesLowerAccelerationLimitWhileAnotherCurveConstrainsSpeed) {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.curvature_sample_spacing_m = 0.2;
  config.curve_approach_deceleration_mps2 = 0.001;
  config.target_speed_acceleration_limit_mps2 = 1.0;
  config.curve_target_speed_acceleration_limit_mps2 = 0.25;
  CurvatureSpeedPlanner planner(config);

  const CurvatureSpeedPlan sharp_curve =
      planner.update(quarterCircle(1.0), 0.1);
  const CurvatureSpeedPlan next_curve =
      planner.update(straightThenQuarterCircle(10.0, 10.0), 1.0);
  const CurvatureSpeedPlan clear_straight =
      planner.update(straightPath(), 1.0);

  ASSERT_LT(sharp_curve.target_speed_mps,
            next_curve.raw_target_speed_mps);
  ASSERT_LT(next_curve.raw_target_speed_mps,
            config.configured_target_speed_mps);
  EXPECT_NEAR(sharp_curve.target_speed_mps + 0.25,
              next_curve.target_speed_mps, 1.0e-9);
  EXPECT_NEAR(next_curve.target_speed_mps + 1.0,
              clear_straight.target_speed_mps, 1.0e-9);
}

TEST(CurvatureSpeedPlanner, FiltersRawTargetBeforeApplyingSlewLimit) {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.minimum_curve_speed_mps = 2.0;
  config.curvature_sample_spacing_m = 0.2;
  config.curve_approach_deceleration_mps2 = 0.001;
  config.target_speed_filter_time_constant_sec = 0.5;
  config.target_speed_acceleration_limit_mps2 = 100.0;
  config.target_speed_deceleration_limit_mps2 = 100.0;
  CurvatureSpeedPlanner planner(config);
  ASSERT_NEAR(10.0, planner.update(straightPath(), 0.1).target_speed_mps,
              1.0e-9);

  const CurvatureSpeedPlan braking = planner.update(quarterCircle(1.0), 0.5);

  EXPECT_NEAR(2.0, braking.raw_target_speed_mps, 0.001);
  EXPECT_GT(braking.filtered_target_speed_mps,
            braking.raw_target_speed_mps);
  EXPECT_LT(braking.filtered_target_speed_mps, 10.0);
  EXPECT_NEAR(braking.filtered_target_speed_mps,
              braking.target_speed_mps, 1.0e-12);
}

TEST(CurvatureSpeedPlanner, ZeroTargetFilterTimeConstantDisablesFilter) {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.minimum_curve_speed_mps = 2.0;
  config.curvature_sample_spacing_m = 0.2;
  config.curve_approach_deceleration_mps2 = 0.001;
  config.target_speed_filter_time_constant_sec = 0.0;
  config.target_speed_deceleration_limit_mps2 = 100.0;
  CurvatureSpeedPlanner planner(config);
  ASSERT_NEAR(10.0, planner.update(straightPath(), 0.1).target_speed_mps,
              1.0e-9);

  const CurvatureSpeedPlan braking = planner.update(quarterCircle(1.0), 0.5);

  EXPECT_NEAR(braking.raw_target_speed_mps,
              braking.filtered_target_speed_mps, 1.0e-12);
  EXPECT_NEAR(braking.raw_target_speed_mps,
              braking.target_speed_mps, 1.0e-12);
}

TEST(CurvatureSpeedPlanner, ResetInitializesFromCurrentPath) {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.minimum_curve_speed_mps = 2.0;
  config.curvature_sample_spacing_m = 0.2;
  config.curve_approach_deceleration_mps2 = 0.001;
  CurvatureSpeedPlanner planner(config);
  ASSERT_NEAR(10.0, planner.update(straightPath(), 0.1).target_speed_mps,
              1.0e-9);

  planner.reset();

  const CurvatureSpeedPlan after_reset =
      planner.update(quarterCircle(1.0), 0.1);
  EXPECT_NEAR(2.0, after_reset.target_speed_mps, 0.001);
  EXPECT_NEAR(after_reset.raw_target_speed_mps,
              after_reset.filtered_target_speed_mps, 1.0e-12);
}

TEST(CurvatureSpeedPlanner, FarCurveUsesDistanceAwareApproachSpeed) {
  const CurvatureSpeedPlannerConfig config = highSpeedConfig();
  CurvatureSpeedPlanner planner(config);

  const CurvatureSpeedPlan plan =
      planner.update(straightThenQuarterCircle(30.0, 10.0), 0.1);

  EXPECT_GT(plan.speed_limiting_curve_distance_m, 20.0);
  EXPECT_GT(plan.raw_target_speed_mps, 43.0 / 3.6);
  EXPECT_LT(plan.raw_target_speed_mps, 46.0 / 3.6);
  EXPECT_GT(plan.raw_target_speed_mps, plan.curvature_speed_limit_mps);
  EXPECT_LT(plan.raw_target_speed_mps, config.configured_target_speed_mps);
  EXPECT_NEAR(0.0, plan.lookahead_curvature_m_inv, 1.0e-9);
}

TEST(CurvatureSpeedPlanner, MildCurveDoesNotOverReduceTargetSpeed) {
  const CurvatureSpeedPlannerConfig config = highSpeedConfig();
  CurvatureSpeedPlanner planner(config);

  const CurvatureSpeedPlan plan = planner.update(quarterCircle(50.0), 0.1);

  EXPECT_GT(plan.raw_target_speed_mps, 43.0 / 3.6);
  EXPECT_LE(plan.raw_target_speed_mps, config.configured_target_speed_mps);
  EXPECT_GT(plan.lookahead_curvature_m_inv, 0.0);
}

TEST(CurvatureSpeedPlanner, ContinuouslyAddsMoreReductionAsCurvatureGrows) {
  CurvatureSpeedPlannerConfig baseline_config = highSpeedConfig();
  baseline_config.curvature_sample_spacing_m = 2.0;
  baseline_config.curve_approach_deceleration_mps2 = 0.001;
  baseline_config.curvature_speed_reduction_gain_m = 0.0;
  CurvatureSpeedPlannerConfig reduced_config = baseline_config;
  reduced_config.curvature_speed_reduction_gain_m = 2.5;

  CurvatureSpeedPlanner baseline_mild(baseline_config);
  CurvatureSpeedPlanner reduced_mild(reduced_config);
  CurvatureSpeedPlanner baseline_sharp(baseline_config);
  CurvatureSpeedPlanner reduced_sharp(reduced_config);

  const double mild_baseline =
      baseline_mild.update(quarterCircle(50.0), 0.1).raw_target_speed_mps;
  const double mild_reduced =
      reduced_mild.update(quarterCircle(50.0), 0.1).raw_target_speed_mps;
  const double sharp_baseline =
      baseline_sharp.update(quarterCircle(10.0), 0.1).raw_target_speed_mps;
  const double sharp_reduced =
      reduced_sharp.update(quarterCircle(10.0), 0.1).raw_target_speed_mps;

  EXPECT_LT(mild_reduced, mild_baseline);
  EXPECT_LT(sharp_reduced, sharp_baseline);
  EXPECT_GT(mild_reduced / mild_baseline,
            sharp_reduced / sharp_baseline);
  EXPECT_GT(sharp_reduced, 15.0 / 3.6);
  EXPECT_LT(sharp_reduced, 16.5 / 3.6);
}

TEST(CurvatureSpeedPlanner, ApproachingCurveLowersSpeedAndActivatesLdCurvature) {
  const CurvatureSpeedPlannerConfig config = highSpeedConfig();
  CurvatureSpeedPlanner far_planner(config);
  CurvatureSpeedPlanner near_planner(config);

  const CurvatureSpeedPlan far_plan =
      far_planner.update(straightThenQuarterCircle(30.0, 10.0), 0.1);
  const CurvatureSpeedPlan near_plan =
      near_planner.update(straightThenQuarterCircle(4.0, 10.0), 0.1);

  EXPECT_LT(near_plan.raw_target_speed_mps, far_plan.raw_target_speed_mps);
  EXPECT_GT(near_plan.lookahead_curvature_m_inv, 0.0);
}

TEST(CurvatureSpeedPlanner, SampleSpacingSmoothsSubMetreSplice) {
  const std::vector<Point2d> path = {
      {0.0, 0.0}, {0.2, 0.0}, {0.4, 0.2}, {1.0, 0.0},
      {2.0, 0.0}, {4.0, 0.0}, {6.0, 0.0}, {8.0, 0.0}};
  CurvatureSpeedPlannerConfig fine_config = highSpeedConfig();
  fine_config.curvature_sample_spacing_m = 0.2;
  CurvatureSpeedPlannerConfig smoothed_config = highSpeedConfig();
  smoothed_config.curvature_sample_spacing_m = 2.0;
  CurvatureSpeedPlanner fine_planner(fine_config);
  CurvatureSpeedPlanner smoothed_planner(smoothed_config);

  const CurvatureSpeedPlan fine_plan = fine_planner.update(path, 0.1);
  const CurvatureSpeedPlan smoothed_plan = smoothed_planner.update(path, 0.1);

  EXPECT_LT(smoothed_plan.lookahead_curvature_m_inv,
            fine_plan.lookahead_curvature_m_inv);
}

TEST(CurvatureSpeedPlanner, RejectsInvalidConfigurationAndInputs) {
  CurvatureSpeedPlannerConfig config = baseConfig();
  config.minimum_curve_speed_mps = 11.0;
  EXPECT_THROW(
      {
        const CurvatureSpeedPlanner invalid_planner(config);
        (void)invalid_planner;
      },
      std::invalid_argument);

  config = baseConfig();
  config.preview_distance_m = 0.0;
  EXPECT_THROW(
      {
        const CurvatureSpeedPlanner invalid_planner(config);
        (void)invalid_planner;
      },
      std::invalid_argument);

  config = baseConfig();
  config.lookahead_curvature_preview_distance_m = 0.0;
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.curvature_sample_spacing_m = 0.0;
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.curve_approach_deceleration_mps2 =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.curvature_speed_reduction_gain_m = -0.1;
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.curvature_speed_reduction_gain_m =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.target_speed_filter_time_constant_sec = -0.1;
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.target_speed_filter_time_constant_sec =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.curve_target_speed_acceleration_limit_mps2 = 0.0;
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  config = baseConfig();
  config.curve_target_speed_acceleration_limit_mps2 =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ CurvatureSpeedPlanner invalid_planner(config); },
               std::invalid_argument);

  CurvatureSpeedPlanner planner(baseConfig());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(planner.update({{0.0, 0.0}, {1.0, nan}, {2.0, 0.0}}, 0.1),
               std::invalid_argument);
  EXPECT_THROW(planner.update(straightPath(), 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
