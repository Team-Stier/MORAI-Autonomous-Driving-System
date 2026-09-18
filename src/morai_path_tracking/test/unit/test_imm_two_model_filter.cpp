#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "morai_path_tracking/controllers/lateral/imm_two_model_filter.hpp"

namespace morai_path_tracking {
namespace {

ImmConfig testConfig() {
  ImmConfig config;
  config.mass_kg = 2000.0;
  config.yaw_inertia_kgm2 = 4000.0;
  config.front_cornering_stiffness_n_per_rad = 60000.0;
  config.rear_cornering_stiffness_n_per_rad = 60000.0;
  config.front_axle_to_cg_m = 1.5;
  config.rear_axle_to_cg_m = 1.5;
  config.process_noise_sideslip = 1.0e-4;
  config.process_noise_yaw_rate = 1.0e-4;
  config.measurement_noise_sideslip = 1.0e-4;
  config.measurement_noise_yaw_rate = 1.0e-4;
  config.initial_covariance_sideslip = 1.0e-4;
  config.initial_covariance_yaw_rate = 1.0e-4;
  config.initial_pure_pursuit_probability = 0.5;
  config.initial_stanley_probability = 0.5;
  config.stanley_probability_min = 0.001;
  config.stanley_probability_max = 0.999;
  config.transition_pure_pursuit_to_pure_pursuit = 1.0;
  config.transition_pure_pursuit_to_stanley = 0.0;
  config.transition_stanley_to_pure_pursuit = 0.0;
  config.transition_stanley_to_stanley = 1.0;
  config.transition_speed_gain = 0.0;
  config.transition_reference_speed_mps = 60.0 / 3.6;
  config.minimum_model_speed_mps = 1.0;
  return config;
}

TEST(ImmTwoModelFilter, ZeroStateAndZeroSteeringRemainZero) {
  ImmTwoModelFilter filter(testConfig());

  const ImmResult result =
      filter.update(10.0, 0.0, 0.0, 0.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.0, result.pure_pursuit_state.sideslip_rad, 1.0e-12);
  EXPECT_NEAR(0.0, result.pure_pursuit_state.yaw_rate_radps, 1.0e-12);
  EXPECT_NEAR(0.0, result.stanley_state.sideslip_rad, 1.0e-12);
  EXPECT_NEAR(0.0, result.stanley_state.yaw_rate_radps, 1.0e-12);
  EXPECT_NEAR(0.5, result.pure_pursuit_probability, 1.0e-12);
  EXPECT_NEAR(0.5, result.stanley_probability, 1.0e-12);
}

TEST(ImmTwoModelFilter, StanleyPredictionMatchingMeasurementRaisesProbability) {
  ImmTwoModelFilter filter(testConfig());

  // From the zero state at vx=10 m/s and dt=0.1 s, delta=0.1 rad gives
  // beta=0.06 rad and yaw_rate=0.45 rad/s with the configured bicycle model.
  const ImmResult result =
      filter.update(10.0, 0.06, 0.45, 0.0, 0.1, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.stanley_probability, 0.9);
  EXPECT_LT(result.stanley_innovation_norm,
            result.pure_pursuit_innovation_norm);
}

TEST(ImmTwoModelFilter,
     PurePursuitPredictionMatchingMeasurementRaisesProbability) {
  ImmTwoModelFilter filter(testConfig());

  const ImmResult result =
      filter.update(10.0, -0.06, -0.45, -0.1, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_GT(result.pure_pursuit_probability, 0.9);
  EXPECT_LT(result.pure_pursuit_innovation_norm,
            result.stanley_innovation_norm);
}

TEST(ImmTwoModelFilter, EqualCandidatesPreserveEqualProbabilities) {
  ImmTwoModelFilter filter(testConfig());

  const ImmResult result =
      filter.update(12.0, 0.01, 0.05, 0.02, 0.02, 0.05);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.5, result.pure_pursuit_probability, 1.0e-12);
  EXPECT_NEAR(0.5, result.stanley_probability, 1.0e-12);
  EXPECT_NEAR(result.pure_pursuit_innovation_norm,
              result.stanley_innovation_norm, 1.0e-12);
}

TEST(ImmTwoModelFilter, SpeedDependentPriorContinuouslyFavoursStanley) {
  ImmConfig config = testConfig();
  config.transition_pure_pursuit_to_pure_pursuit = 0.9;
  config.transition_pure_pursuit_to_stanley = 0.1;
  config.transition_stanley_to_pure_pursuit = 0.95;
  config.transition_stanley_to_stanley = 0.05;
  config.transition_speed_gain = 0.1;

  ImmTwoModelFilter low_speed_filter(config);
  ImmTwoModelFilter high_speed_filter(config);
  const ImmResult low_speed =
      low_speed_filter.update(0.5, 0.0, 0.0, 0.0, 0.0, 0.05);
  const ImmResult high_speed =
      high_speed_filter.update(60.0 / 3.6, 0.0, 0.0, 0.0, 0.0, 0.05);

  ASSERT_TRUE(low_speed.valid) << low_speed.error;
  ASSERT_TRUE(high_speed.valid) << high_speed.error;
  EXPECT_GT(high_speed.stanley_probability,
            low_speed.stanley_probability);
  EXPECT_NEAR(1.0, high_speed.pure_pursuit_probability +
                       high_speed.stanley_probability,
              1.0e-12);
}

TEST(ImmTwoModelFilter, ProbabilityBoundsRemainFiniteAndNormalized) {
  ImmConfig config = testConfig();
  config.stanley_probability_min = 0.2;
  config.stanley_probability_max = 0.8;
  ImmTwoModelFilter filter(config);

  const ImmResult result =
      filter.update(10.0, 10.0, 10.0, 0.0, 0.7, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_TRUE(std::isfinite(result.pure_pursuit_probability));
  EXPECT_TRUE(std::isfinite(result.stanley_probability));
  EXPECT_GE(result.stanley_probability, 0.2);
  EXPECT_LE(result.stanley_probability, 0.8);
  EXPECT_NEAR(1.0, result.pure_pursuit_probability +
                       result.stanley_probability,
              1.0e-12);
}

TEST(ImmTwoModelFilter, ResetRestoresInitialStateAndProbabilities) {
  ImmConfig config = testConfig();
  config.initial_pure_pursuit_probability = 0.7;
  config.initial_stanley_probability = 0.3;
  ImmTwoModelFilter filter(config);
  ASSERT_TRUE(filter.update(10.0, 0.06, 0.45, 0.0, 0.1, 0.1).valid);

  filter.reset();
  const ImmResult result =
      filter.update(10.0, 0.0, 0.0, 0.0, 0.0, 0.1);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.7, result.pure_pursuit_probability, 1.0e-12);
  EXPECT_NEAR(0.3, result.stanley_probability, 1.0e-12);
  EXPECT_NEAR(0.0, result.pure_pursuit_state.sideslip_rad, 1.0e-12);
  EXPECT_NEAR(0.0, result.stanley_state.yaw_rate_radps, 1.0e-12);
}

TEST(ImmTwoModelFilter, LowSpeedUsesFiniteModelSpeed) {
  ImmTwoModelFilter filter(testConfig());

  const ImmResult result =
      filter.update(0.0, 0.01, 0.02, 0.1, -0.1, 0.05);

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_TRUE(std::isfinite(result.pure_pursuit_innovation_norm));
  EXPECT_TRUE(std::isfinite(result.stanley_innovation_norm));
}

TEST(ImmTwoModelFilter, RejectsInvalidConfiguration) {
  const auto invalid = [](const ImmConfig& config) {
    return !ImmTwoModelFilter(config)
                .update(10.0, 0.0, 0.0, 0.0, 0.0, 0.05)
                .valid;
  };

  ImmConfig config = testConfig();
  config.mass_kg = 0.0;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.yaw_inertia_kgm2 = -1.0;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.front_cornering_stiffness_n_per_rad = 0.0;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.rear_axle_to_cg_m = 0.0;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.process_noise_sideslip = -0.1;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.measurement_noise_yaw_rate = 0.0;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.initial_covariance_sideslip = 0.0;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.initial_pure_pursuit_probability = 0.8;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.stanley_probability_min = 0.8;
  config.stanley_probability_max = 0.2;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.transition_pure_pursuit_to_pure_pursuit = 0.8;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.transition_speed_gain = 1.1;
  EXPECT_TRUE(invalid(config));
  config = testConfig();
  config.minimum_model_speed_mps = 0.0;
  EXPECT_TRUE(invalid(config));
}

TEST(ImmTwoModelFilter, RejectsInvalidRuntimeInputWithoutChangingState) {
  ImmTwoModelFilter filter(testConfig());
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(filter.update(nan, 0.0, 0.0, 0.0, 0.0, 0.05).valid);
  EXPECT_FALSE(filter.update(10.0, nan, 0.0, 0.0, 0.0, 0.05).valid);
  EXPECT_FALSE(filter.update(10.0, 0.0, nan, 0.0, 0.0, 0.05).valid);
  EXPECT_FALSE(filter.update(10.0, 0.0, 0.0, nan, 0.0, 0.05).valid);
  EXPECT_FALSE(filter.update(10.0, 0.0, 0.0, 0.0, nan, 0.05).valid);
  EXPECT_FALSE(filter.update(10.0, 0.0, 0.0, 0.0, 0.0, 0.0).valid);

  const ImmResult result =
      filter.update(10.0, 0.0, 0.0, 0.0, 0.0, 0.05);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.5, result.pure_pursuit_probability, 1.0e-12);
  EXPECT_NEAR(0.5, result.stanley_probability, 1.0e-12);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
