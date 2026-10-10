#include <velocity_planner/cores/plain_velocity60.hpp>
#include <velocity_planner/cores/plain_velocity100.hpp>
#include <velocity_planner/validators/velocity_inputs.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

TEST(PlainVelocity, InjectsSpeedsAndPassesThroughMapInputs) {
  interfaces::Path path;
  path.header.stamp.fromNSec(1234567890);
  path.header.frame_id = "map";
  path.points.resize(2);
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    auto& point = path.points[i];
    point.pose.header = path.header;
    point.pose.pose.position.x = 10;
    point.pose.pose.position.y = 21 + i;
    point.pose.pose.position.z = 32 + i;
    point.pose.pose.orientation.w = 1;
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    point.acceleration = -2;
  }
  interfaces::TrafficLight light;
  light.state = interfaces::TrafficLight::RED;
  interfaces::CompetitionVehicleStatus competition;
  competition.signed_velocity_mps = 5;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms.front().transform.translation.x = 10;
  velocity_planner::PlainVelocity60 plain60;
  velocity_planner::PlainVelocity100 plain100;
  for (const bool fast : {false, true}) {
    const auto output = fast ? plain100.process(path, light, competition, tf)
                             : plain60.process(path, light, competition, tf);
    const auto& message = std::get<0>(output);
    EXPECT_EQ(path.header, message.header);
    ASSERT_EQ(path.points.size(), message.points.size());
    for (std::size_t i = 0; i < message.points.size(); ++i) {
      EXPECT_DOUBLE_EQ((fast ? 100.0 : 60.0) / 3.6, message.points[i].velocity);
      EXPECT_DOUBLE_EQ(0, message.points[i].acceleration);
      EXPECT_EQ(path.points[i].pose, message.points[i].pose);
    }
    EXPECT_EQ(light, std::get<1>(output));
    EXPECT_EQ(competition, std::get<2>(output));
    EXPECT_EQ(tf, std::get<3>(output));
  }
  EXPECT_TRUE(std::isnan(path.points.front().velocity));
}

TEST(PlainVelocity, PreservesExistingValuesAndFillsOnlyNaNPoints) {
  interfaces::Path path;
  path.header.stamp.fromNSec(1234567890);
  path.header.frame_id = "map";
  path.points.resize(3);
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    auto& point = path.points[i];
    point.pose.header = path.header;
    point.pose.pose.position.x = i;
    point.pose.pose.orientation.w = 1;
    point.acceleration = -1.0 - i;
  }
  path.points[0].velocity = 0.0;
  path.points[1].velocity = 5.0;
  path.points[2].velocity = std::numeric_limits<double>::quiet_NaN();
  velocity_planner::PlainVelocity60 plain60;
  velocity_planner::PlainVelocity100 plain100;
  for (const bool fast : {false, true}) {
    const auto output = fast ? plain100.process(path, {}, {}, {})
                             : plain60.process(path, {}, {}, {});
    const auto& message = std::get<0>(output);
    EXPECT_EQ(path.header, message.header);
    ASSERT_EQ(path.points.size(), message.points.size());
    EXPECT_EQ(path.points[0], message.points[0]);
    EXPECT_EQ(path.points[1], message.points[1]);
    EXPECT_EQ(path.points[2].pose, message.points[2].pose);
    EXPECT_DOUBLE_EQ((fast ? 100.0 : 60.0) / 3.6, message.points[2].velocity);
    EXPECT_DOUBLE_EQ(0.0, message.points[2].acceleration);
    const auto repeated = fast ? plain100.process(message, {}, {}, {})
                               : plain60.process(message, {}, {}, {});
    EXPECT_EQ(message, std::get<0>(repeated));
  }
  EXPECT_TRUE(std::isnan(path.points[2].velocity));
  EXPECT_DOUBLE_EQ(-3.0, path.points[2].acceleration);
}

TEST(PlainVelocity, AllowsUninjectedAndFiniteSpeedsButRejectsInfiniteSpeeds) {
  interfaces::Path path;
  path.points.resize(3);
  path.points[0].velocity = std::numeric_limits<double>::quiet_NaN();
  path.points[1].velocity = 0.0;
  path.points[2].velocity = 5.0;
  velocity_planner::VelocityInputs validator;
  std::vector<node_pipeline::Kind> issues;
  validator(path, {}, issues);
  EXPECT_TRUE(issues.empty());
  for (double velocity : {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    issues.clear();
    path.points[1].velocity = velocity;
    validator(path, {}, issues);
    EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(), node_pipeline::Kind::Inf));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
