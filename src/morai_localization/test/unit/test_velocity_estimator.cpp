#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "morai_localization/estimation/velocity_estimator.hpp"

namespace morai_localization {
namespace {

TEST(VelocityEstimator, NeedsTwoSamples) {
  VelocityEstimator estimator(VelocityEstimatorConfig{});
  EXPECT_FALSE(estimator.update(0.0, 0.0, 1.0, 0.0).valid);
  const auto estimate = estimator.update(0.4, 0.0, 1.2, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(2.0, estimate.longitudinal_mps, 1.0e-9);
  EXPECT_NEAR(0.0, estimate.lateral_mps, 1.0e-9);
}

TEST(VelocityEstimator, ProjectsMapVelocityWithImuYaw) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, M_PI_2);
  const auto estimate = estimator.update(0.0, 0.4, 1.2, M_PI_2);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(2.0, estimate.longitudinal_mps, 1.0e-9);
  EXPECT_NEAR(0.0, estimate.lateral_mps, 1.0e-9);
}

TEST(VelocityEstimator, AppliesTimeConstantFilter) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.1;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  ASSERT_TRUE(estimator.update(0.1, 0.0, 1.1, 0.0).valid);
  const auto estimate = estimator.update(0.3, 0.0, 1.2, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(1.5, estimate.longitudinal_mps, 1.0e-9);
}

TEST(VelocityEstimator, ReportsSignedReverseSpeed) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(1.0, 0.0, 1.0, 0.0);
  const auto estimate = estimator.update(0.6, 0.0, 1.2, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(-2.0, estimate.longitudinal_mps, 1.0e-9);
  EXPECT_NEAR(0.0, estimate.lateral_mps, 1.0e-9);
}

TEST(VelocityEstimator, DisablesFilterWhenTimeConstantIsZero) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  ASSERT_TRUE(estimator.update(0.1, 0.0, 1.1, 0.0).valid);
  const auto estimate = estimator.update(0.3, 0.0, 1.2, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(2.0, estimate.longitudinal_mps, 1.0e-9);
}

TEST(VelocityEstimator, RejectsNonFiniteInput) {
  VelocityEstimator estimator(VelocityEstimatorConfig{});
  EXPECT_FALSE(estimator.update(0.0, 0.0, 1.0, 0.0).valid);
  EXPECT_FALSE(estimator.update(std::numeric_limits<double>::quiet_NaN(),
                                0.0, 1.1, 0.0)
                   .valid);
  EXPECT_FALSE(estimator.update(0.1, 0.0, 1.1,
                                std::numeric_limits<double>::infinity())
                   .valid);
}

TEST(VelocityEstimator, RejectsTimeReversalAndResetsBaseline) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  EXPECT_FALSE(estimator.update(0.1, 0.0, 0.9, 0.0).valid);
  const auto estimate = estimator.update(0.3, 0.0, 1.0, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(2.0, estimate.longitudinal_mps, 1.0e-9);
}

TEST(VelocityEstimator, RejectsTooShortIntervalAndResetsBaseline) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  EXPECT_FALSE(estimator.update(0.001, 0.0, 1.001, 0.0).valid);
  const auto estimate = estimator.update(0.101, 0.0, 1.101, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(1.0, estimate.longitudinal_mps, 1.0e-9);
}

TEST(VelocityEstimator, RejectsTooLongIntervalAndResetsBaseline) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  EXPECT_FALSE(estimator.update(1.0, 0.0, 1.3, 0.0).valid);
  const auto estimate = estimator.update(1.1, 0.0, 1.4, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(1.0, estimate.longitudinal_mps, 1.0e-9);
}

TEST(VelocityEstimator, RejectsExcessiveSpeedAndResetsBaseline) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  config.maximum_speed_mps = 2.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  EXPECT_FALSE(estimator.update(1.0, 0.0, 1.1, 0.0).valid);
  const auto estimate = estimator.update(1.1, 0.0, 1.2, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(1.0, estimate.longitudinal_mps, 1.0e-9);
}

TEST(VelocityEstimator, ResetRequiresNewBaseline) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  ASSERT_TRUE(estimator.update(0.1, 0.0, 1.1, 0.0).valid);
  estimator.reset();
  EXPECT_FALSE(estimator.update(0.2, 0.0, 1.2, 0.0).valid);
  EXPECT_TRUE(estimator.update(0.3, 0.0, 1.3, 0.0).valid);
}

TEST(VelocityEstimator, RejectsInvalidConfig) {
  VelocityEstimatorConfig config;
  config.minimum_dt_sec = 0.0;
  EXPECT_THROW({ VelocityEstimator estimator(config); }, std::invalid_argument);

  config = VelocityEstimatorConfig{};
  config.maximum_dt_sec = config.minimum_dt_sec;
  EXPECT_THROW({ VelocityEstimator estimator(config); }, std::invalid_argument);

  config = VelocityEstimatorConfig{};
  config.maximum_speed_mps = std::numeric_limits<double>::infinity();
  EXPECT_THROW({ VelocityEstimator estimator(config); }, std::invalid_argument);

  config = VelocityEstimatorConfig{};
  config.filter_time_constant_sec = -0.1;
  EXPECT_THROW({ VelocityEstimator estimator(config); }, std::invalid_argument);
}

}  // namespace
}  // namespace morai_localization

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
