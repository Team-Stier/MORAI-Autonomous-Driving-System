#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "morai_path_tracking/controllers/lateral/pure_pursuit.hpp"

namespace morai_path_tracking {
namespace {

TEST(PurePursuit, StraightPathProducesZeroSteering) {
  const std::vector<Point2d> path{{-1.0, 0.0}, {0.0, 0.0}, {10.0, 0.0}};
  const auto result = computePurePursuit(path, 2.0, 0.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(4.0, result.lookahead_m, 1.0e-9);
  EXPECT_NEAR(4.0, result.target.x, 1.0e-9);
  EXPECT_NEAR(0.0, result.steering_angle_rad, 1.0e-9);
}

TEST(PurePursuit, LeftTargetProducesPositiveSteering) {
  const std::vector<Point2d> path{{0.0, 0.0}, {4.0, 2.0}, {8.0, 4.0}};
  const auto result = computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.steering_angle_rad, 0.0);
}

TEST(PurePursuit, RightTargetProducesNegativeSteering) {
  const std::vector<Point2d> path{{0.0, 0.0}, {4.0, -2.0}, {8.0, -4.0}};
  const auto result = computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.steering_angle_rad, 0.0);
}

TEST(PurePursuit, IgnoresFirstPointBehindVehicle) {
  const std::vector<Point2d> path{{-5.0, 0.0}, {5.0, 0.0}};
  const auto result = computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(3.0, result.target.x, 1.0e-9);
}

TEST(PurePursuit, InterpolatesLookaheadCircleIntersection) {
  const std::vector<Point2d> path{{0.0, 0.0}, {10.0, 0.0}};
  const auto result = computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(3.0, result.target.x, 1.0e-9);
  EXPECT_NEAR(0.0, result.target.y, 1.0e-9);
}

TEST(PurePursuit, UsesFarthestForwardPointWhenPathIsShort) {
  const std::vector<Point2d> path{{0.25, 0.0}, {2.0, 1.0}};
  const auto result = computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(2.0, result.target.x, 1.0e-9);
  EXPECT_NEAR(1.0, result.target.y, 1.0e-9);
}

TEST(PurePursuit, RejectsPathWithoutForwardTarget) {
  const std::vector<Point2d> path{{-3.0, 0.0}, {0.0, 1.0}, {-1.0, 2.0}};
  EXPECT_FALSE(computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{}).valid);
}

TEST(PurePursuit, RejectsPathWithOnlyNonFinitePoints) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Point2d> path{{nan, 1.0}, {1.0, nan}};
  EXPECT_FALSE(computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{}).valid);
}

TEST(PurePursuit, ClampsDynamicLookahead) {
  const std::vector<Point2d> path{{0.0, 0.0}, {10.0, 0.0}};
  EXPECT_NEAR(3.0,
              computePurePursuit(path, 0.0, 0.0, PurePursuitConfig{}).lookahead_m,
              1.0e-9);
  EXPECT_NEAR(6.0,
              computePurePursuit(path, 100.0, 0.0, PurePursuitConfig{}).lookahead_m,
              1.0e-9);
}

TEST(PurePursuit, ShortensLookaheadForPreviewCurvature) {
  PurePursuitConfig config;
  config.lookahead_min_m = 1.0;
  config.lookahead_curvature_gain_m = 5.0;
  const std::vector<Point2d> path{{0.0, 0.0}, {10.0, 0.0}};

  const auto straight = computePurePursuit(path, 2.0, 0.0, config);
  const auto curved = computePurePursuit(path, 2.0, 0.2, config);

  ASSERT_TRUE(straight.valid);
  ASSERT_TRUE(curved.valid);
  EXPECT_NEAR(4.0, straight.lookahead_m, 1.0e-9);
  EXPECT_NEAR(2.0, curved.lookahead_m, 1.0e-9);
}

TEST(PurePursuit, RejectsInvalidPreviewCurvature) {
  const std::vector<Point2d> path{{0.0, 0.0}, {10.0, 0.0}};
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(computePurePursuit(path, 2.0, -0.1, PurePursuitConfig{}).valid);
  EXPECT_FALSE(computePurePursuit(path, 2.0, nan, PurePursuitConfig{}).valid);
}

TEST(PurePursuit, ClampsSteeringToConfiguredPhysicalLimit) {
  PurePursuitConfig config;
  config.minimum_target_distance_m = 0.001;
  const std::vector<Point2d> left_path{{0.001, 0.01}};
  const std::vector<Point2d> right_path{{0.001, -0.01}};
  EXPECT_NEAR(config.maximum_steering_angle_rad,
              computePurePursuit(left_path, 0.0, 0.0, config).steering_angle_rad,
              1.0e-9);
  EXPECT_NEAR(-config.maximum_steering_angle_rad,
              computePurePursuit(right_path, 0.0, 0.0, config).steering_angle_rad,
              1.0e-9);
}

TEST(PurePursuit, RejectsInvalidConfiguration) {
  const std::vector<Point2d> path{{0.0, 0.0}, {10.0, 0.0}};
  PurePursuitConfig config;
  config.wheelbase_m = 0.0;
  EXPECT_THROW(computePurePursuit(path, 0.0, 0.0, config),
               std::invalid_argument);

  config = PurePursuitConfig{};
  config.lookahead_speed_gain_sec = -0.1;
  EXPECT_THROW(computePurePursuit(path, 0.0, 0.0, config),
               std::invalid_argument);

  config = PurePursuitConfig{};
  config.lookahead_min_m = 7.0;
  EXPECT_THROW(computePurePursuit(path, 0.0, 0.0, config),
               std::invalid_argument);

  config = PurePursuitConfig{};
  config.minimum_target_distance_m = 0.0;
  EXPECT_THROW(computePurePursuit(path, 0.0, 0.0, config),
               std::invalid_argument);

  config = PurePursuitConfig{};
  config.maximum_steering_angle_rad = std::numeric_limits<double>::infinity();
  EXPECT_THROW(computePurePursuit(path, 0.0, 0.0, config),
               std::invalid_argument);

  config = PurePursuitConfig{};
  config.lookahead_curvature_gain_m = -0.1;
  EXPECT_THROW(computePurePursuit(path, 0.0, 0.0, config),
               std::invalid_argument);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
