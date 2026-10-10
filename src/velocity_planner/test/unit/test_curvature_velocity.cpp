#include <velocity_planner/cores/curvature_velocity.hpp>
#include <velocity_planner/parameters/curvature_velocity.hpp>
#include <velocity_planner/validators/curvature_inputs.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

namespace {
velocity_planner::CurvatureVelocityParameters parameters() {
  velocity_planner::CurvatureVelocityParameters result;
  result.configured_target_speed_mps = 10.0;
  result.minimum_curve_speed_mps = 0.0;
  result.maximum_lateral_acceleration_mps2 = 1.0;
  result.preview_distance_m = 30.0;
  result.curvature_sample_spacing_m = 0.2;
  result.curve_approach_deceleration_mps2 = 0.001;
  result.target_speed_filter_time_constant_sec = 0.0;
  return result;
}

interfaces::Path path(double straight_m, double radius_m = 0.0) {
  interfaces::Path result;
  result.header.frame_id = "map";
  result.header.stamp.fromSec(10.0);
  const auto append = [&](double x, double y) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = 100.0 + x;
    point.pose.pose.position.y = 200.0 + y;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    point.acceleration = std::numeric_limits<double>::quiet_NaN();
    result.points.push_back(point);
  };
  for (double x = 0.0; x <= straight_m; x += 0.1) append(x, 0.0);
  if (radius_m > 0.0) {
    constexpr double half_pi = 1.57079632679489661923;
    for (double angle = 0.1 / radius_m; angle < half_pi; angle += 0.1 / radius_m)
      append(straight_m + radius_m * std::sin(angle), radius_m * (1.0 - std::cos(angle)));
    append(straight_m + radius_m, radius_m);
  }
  return result;
}

tf2_msgs::TFMessage transforms() {
  tf2_msgs::TFMessage result;
  result.transforms.resize(1);
  auto& transform = result.transforms.front();
  transform.header.frame_id = "map";
  transform.header.stamp.fromSec(10.0);
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 100.0;
  transform.transform.translation.y = 200.0;
  transform.transform.rotation.w = 1.0;
  return result;
}
}  // namespace

TEST(CurvatureVelocity, KeepsStraightSpeedGeometryAndPassThroughMessages) {
  velocity_planner::CurvatureVelocity core(parameters());
  const auto input = path(20.0);
  const auto tf = transforms();
  interfaces::TrafficLight light;
  light.state = interfaces::TrafficLight::RED;
  interfaces::CompetitionVehicleStatus competition;
  competition.signed_velocity_mps = 5.0;
  const auto output = core.process(input, light, competition, tf);
  const auto& result = std::get<0>(output);
  ASSERT_EQ(input.points.size(), result.points.size());
  EXPECT_EQ(input.header, result.header);
  for (std::size_t index = 0; index < result.points.size(); ++index) {
    EXPECT_DOUBLE_EQ(10.0, result.points[index].velocity);
    EXPECT_DOUBLE_EQ(0.0, result.points[index].acceleration);
    EXPECT_EQ(input.points[index].pose, result.points[index].pose);
  }
  EXPECT_EQ(light, std::get<1>(output));
  EXPECT_EQ(competition, std::get<2>(output));
  EXPECT_EQ(tf, std::get<3>(output));
  EXPECT_TRUE(std::isnan(input.points.front().velocity));
}

TEST(CurvatureVelocity, PreservesExistingProfileIncludingZeroAndOnlyFillsNanPoints) {
  velocity_planner::CurvatureVelocity core(parameters());
  auto filled = path(20.0);
  for (auto& point : filled.points) {
    point.velocity = 3.0;
    point.acceleration = 0.4;
  }
  filled.points.front().velocity = 0.0;
  EXPECT_EQ(filled, std::get<0>(core.process(filled, {}, {}, transforms())));

  auto mixed = path(0.0, 10.0);
  mixed.points.front().velocity = 0.0;
  mixed.points.front().acceleration = 0.4;
  mixed.points[1].velocity = 8.0;
  mixed.points[1].acceleration = -0.2;
  const auto output = core.process(mixed, {}, {}, transforms());
  const auto& result = std::get<0>(output);
  EXPECT_EQ(mixed.points.front(), result.points.front());
  EXPECT_EQ(mixed.points[1], result.points[1]);
  EXPECT_NEAR(3.16234, result.points[2].velocity, 0.003);
  EXPECT_DOUBLE_EQ(0.0, result.points[2].acceleration);
  EXPECT_TRUE(std::isnan(mixed.points[2].velocity));
}

TEST(CurvatureVelocity, ComputesCircleLimitAtVehicleMapPositionAndExtraReduction) {
  auto config = parameters();
  velocity_planner::CurvatureVelocity base(config);
  config.curvature_speed_reduction_gain_m = 2.5;
  velocity_planner::CurvatureVelocity reduced(config);
  const auto input = path(0.0, 10.0);
  const auto normal = base.process(input, {}, {}, transforms());
  const auto extra = reduced.process(input, {}, {}, transforms());
  EXPECT_NEAR(3.16234, std::get<0>(normal).points.front().velocity, 0.003);
  EXPECT_NEAR(2.52990, std::get<0>(extra).points.front().velocity, 0.003);
}

TEST(CurvatureVelocity, BrakesEarlierAsTheCurveApproaches) {
  auto config = parameters();
  config.configured_target_speed_mps = 55.0 / 3.6;
  config.minimum_curve_speed_mps = 12.0 / 3.6;
  config.maximum_lateral_acceleration_mps2 = 3.0;
  config.preview_distance_m = 45.0;
  config.curvature_sample_spacing_m = 2.0;
  config.curve_approach_deceleration_mps2 = 2.0;
  velocity_planner::CurvatureVelocity far_core(config), near_core(config);
  const auto far = far_core.process(path(30.0, 10.0), {}, {}, transforms());
  const auto near = near_core.process(path(4.0, 10.0), {}, {}, transforms());
  const double far_speed = std::get<0>(far).points.front().velocity;
  EXPECT_GT(far_speed, 43.0 / 3.6);
  EXPECT_LT(far_speed, 46.0 / 3.6);
  EXPECT_LT(std::get<0>(near).points.front().velocity, far_speed);
  auto moved_tf = transforms();
  moved_tf.transforms.front().transform.translation.x += 26.0;
  velocity_planner::CurvatureVelocity moved_core(config);
  const auto moved = moved_core.process(path(30.0, 10.0), {}, {}, moved_tf);
  EXPECT_NEAR(std::get<0>(near).points.front().velocity, std::get<0>(moved).points.front().velocity, 0.01);
}

TEST(CurvatureVelocity, UsesPathTimeForFilterAndAccelerationLimits) {
  auto config = parameters();
  config.minimum_curve_speed_mps = 2.0;
  config.target_speed_filter_time_constant_sec = 0.5;
  config.target_speed_acceleration_limit_mps2 = 100.0;
  config.curve_target_speed_acceleration_limit_mps2 = 100.0;
  config.target_speed_deceleration_limit_mps2 = 100.0;
  velocity_planner::CurvatureVelocity filtered(config);
  filtered.process(path(20.0), {}, {}, transforms());
  auto curve = path(0.0, 1.0);
  curve.header.stamp.fromSec(10.5);
  const auto output = filtered.process(curve, {}, {}, transforms());
  EXPECT_NEAR(4.9431, std::get<0>(output).points.front().velocity, 0.002);

  config.target_speed_filter_time_constant_sec = 0.0;
  config.target_speed_deceleration_limit_mps2 = 2.0;
  config.target_speed_acceleration_limit_mps2 = 1.0;
  velocity_planner::CurvatureVelocity limited(config);
  limited.process(path(20.0), {}, {}, transforms());
  EXPECT_NEAR(9.0, std::get<0>(limited.process(curve, {}, {}, transforms())).points.front().velocity, 1e-9);
  auto straight = path(20.0);
  straight.header.stamp.fromSec(11.0);
  EXPECT_NEAR(9.5, std::get<0>(limited.process(straight, {}, {}, transforms())).points.front().velocity, 1e-9);
  EXPECT_NEAR(9.5, std::get<0>(limited.process(straight, {}, {}, transforms())).points.front().velocity, 1e-9);
  straight.header.stamp.fromSec(10.0);
  EXPECT_NEAR(9.5, std::get<0>(limited.process(straight, {}, {}, transforms())).points.front().velocity, 1e-9);
  straight.header.stamp.fromSec(11.25);
  EXPECT_NEAR(9.75, std::get<0>(limited.process(straight, {}, {}, transforms())).points.front().velocity, 1e-9);
}

TEST(CurvatureVelocity, ValidatesGeometryBeforeCoreAndIgnoresUninjectedSpeeds) {
  velocity_planner::CurvatureInputs validator;
  node_pipeline::HealthLimits limits;
  limits.frame = "map";
  std::vector<node_pipeline::Kind> issues;
  auto input = path(20.0);
  validator(input, limits, issues);
  validator(transforms(), limits, issues);
  EXPECT_TRUE(issues.empty());
  input.points.front().velocity = std::numeric_limits<double>::infinity();
  validator(input, limits, issues);
  EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(), node_pipeline::Kind::Inf));
  issues.clear();
  input.points.front().velocity = std::numeric_limits<double>::quiet_NaN();
  input.points.front().pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  validator(input, limits, issues);
  EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(), node_pipeline::Kind::NaN));
  issues.clear();
  validator(interfaces::Path{}, limits, issues);
  EXPECT_FALSE(issues.empty());
  issues.clear();
  validator(tf2_msgs::TFMessage{}, limits, issues);
  EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(), node_pipeline::Kind::Missing));
}

TEST(CurvatureVelocity, LoadsSourceTuningAndRejectsZeroSampleSpacing) {
  std::ifstream file(std::string(NP_PACKAGE_DIR) + "/config/velocity_planner/parameters/curvature_velocity.yaml");
  ASSERT_TRUE(file);
  std::string yaml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  const auto config = velocity_planner::loadCurvatureVelocityParameters(node_pipeline::ParamReader::fromString(yaml));
  velocity_planner::CurvatureVelocity core(config);
  EXPECT_NEAR(5.7271, std::get<0>(core.process(path(0.0, 10.0), {}, {}, transforms())).points.front().velocity, 0.01);
  const auto start = yaml.find("curvature_sample_spacing_m:");
  ASSERT_NE(std::string::npos, start);
  yaml.replace(start, yaml.find('\n', start) - start, "curvature_sample_spacing_m: 0.0");
  EXPECT_THROW(velocity_planner::loadCurvatureVelocityParameters(node_pipeline::ParamReader::fromString(yaml)),
               std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
