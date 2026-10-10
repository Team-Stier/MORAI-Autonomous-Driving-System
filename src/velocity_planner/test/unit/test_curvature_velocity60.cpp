#include <velocity_planner/cores/curvature_velocity60.hpp>
#include <velocity_planner/parameters/curvature_velocity60.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <tuple>

namespace {
velocity_planner::CurvatureVelocity60Parameters parameters() {
  std::ifstream file(std::string(NP_PACKAGE_DIR) + "/config/velocity_planner/parameters/curvature_velocity60.yaml");
  const std::string yaml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return velocity_planner::loadCurvatureVelocity60Parameters(node_pipeline::ParamReader::fromString(yaml));
}
interfaces::Path path() {
  interfaces::Path result;
  result.header.frame_id = "map";
  result.header.stamp.fromSec(10.0);
  result.points.resize(3);
  for (std::size_t index = 0; index < result.points.size(); ++index) {
    auto& point = result.points[index];
    point.pose.header = result.header;
    point.pose.pose.position.x = 100.0 + 10.0 * index;
    point.pose.pose.position.y = 200.0;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    point.acceleration = 0.0;
  }
  return result;
}
tf2_msgs::TFMessage transforms() {
  tf2_msgs::TFMessage result;
  result.transforms.resize(1);
  auto& transform = result.transforms.front();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 100.0;
  transform.transform.translation.y = 200.0;
  transform.transform.rotation.w = 1.0;
  return result;
}
}  // namespace

TEST(CurvatureVelocity60, UsesConfiguredSpeedCapOnStraightPath) {
  velocity_planner::CurvatureVelocity60 core(parameters());
  const auto input = path();
  const auto tf = transforms();
  interfaces::TrafficLight light;
  light.state = interfaces::TrafficLight::GREEN;
  interfaces::CompetitionVehicleStatus competition;
  competition.signed_velocity_mps = 5.0;
  const auto output = core.process(input, light, competition, tf);
  const auto& result = std::get<0>(output);
  EXPECT_EQ(input.header, result.header);
  ASSERT_EQ(input.points.size(), result.points.size());
  for (std::size_t index = 0; index < result.points.size(); ++index) {
    EXPECT_NEAR(60.0 / 3.6, result.points[index].velocity, 1e-9);
    EXPECT_DOUBLE_EQ(0.0, result.points[index].acceleration);
    EXPECT_EQ(input.points[index].pose, result.points[index].pose);
  }
  EXPECT_EQ(light, std::get<1>(output));
  EXPECT_EQ(competition, std::get<2>(output));
  EXPECT_EQ(tf, std::get<3>(output));
}
TEST(CurvatureVelocity60, PreservesZeroAndOnlyFillsNanSpeeds) {
  velocity_planner::CurvatureVelocity60 core(parameters());
  auto input = path();
  input.points[0].velocity = 0.0;
  input.points[0].acceleration = 0.4;
  input.points[1].velocity = 5.0;
  input.points[1].acceleration = -0.2;
  const auto output = core.process(input, {}, {}, transforms());
  const auto& result = std::get<0>(output);
  EXPECT_EQ(input.points[0], result.points[0]);
  EXPECT_EQ(input.points[1], result.points[1]);
  EXPECT_NEAR(60.0 / 3.6, result.points[2].velocity, 1e-9);
  EXPECT_TRUE(std::isnan(input.points[2].velocity));
  EXPECT_EQ(result, std::get<0>(core.process(result, {}, {}, transforms())));
}
TEST(CurvatureVelocity60, RetainsSourceCurvatureTuning) {
  velocity_planner::CurvatureVelocity60 core(parameters());
  auto input = path();
  input.points.clear();
  for (double angle = 0.0; angle <= 1.56; angle += 0.01) {
    interfaces::PathPoint point;
    point.pose.header = input.header;
    point.pose.pose.position.x = 100.0 + 10.0 * std::sin(angle);
    point.pose.pose.position.y = 200.0 + 10.0 * (1.0 - std::cos(angle));
    point.pose.pose.orientation.w = 1.0;
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    input.points.push_back(point);
  }
  const auto output = core.process(input, {}, {}, transforms());
  EXPECT_NEAR(5.7271, std::get<0>(output).points.front().velocity, 0.01);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
