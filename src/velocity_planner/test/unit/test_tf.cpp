#include <velocity_planner/cores/tf.hpp>
#include <velocity_planner/validators/tf_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

TEST(Tf, TransformsAllPosesAndPreservesSpeedAndAcceleration) {
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
    point.pose.pose.orientation.z = std::sqrt(0.5);
    point.pose.pose.orientation.w = std::sqrt(0.5);
    point.velocity = 7 + i;
    point.acceleration = -2.0 - i;
  }
  tf2_msgs::TFMessage transforms;
  transforms.transforms.resize(2);
  transforms.transforms[0].header.frame_id = "unrelated";
  auto& transform = transforms.transforms[1];
  transform.header = path.header;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 10;
  transform.transform.translation.y = 20;
  transform.transform.translation.z = 30;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);

  velocity_planner::Tf core;
  const auto output = core.process(path, transforms);
  EXPECT_EQ(path.header, output.header);
  ASSERT_EQ(path.points.size(), output.points.size());
  for (std::size_t i = 0; i < output.points.size(); ++i) {
    const auto& point = output.points[i];
    EXPECT_DOUBLE_EQ(path.points[i].velocity, point.velocity);
    EXPECT_DOUBLE_EQ(path.points[i].acceleration, point.acceleration);
    EXPECT_NEAR(1 + i, point.pose.pose.position.x, 1e-12);
    EXPECT_NEAR(0, point.pose.pose.position.y, 1e-12);
    EXPECT_NEAR(2 + i, point.pose.pose.position.z, 1e-12);
    EXPECT_NEAR(0, point.pose.pose.orientation.x, 1e-12);
    EXPECT_NEAR(0, point.pose.pose.orientation.y, 1e-12);
    EXPECT_NEAR(0, point.pose.pose.orientation.z, 1e-12);
    EXPECT_NEAR(1, point.pose.pose.orientation.w, 1e-12);
    EXPECT_EQ(path.points[i].pose.header, point.pose.header);
  }
  EXPECT_EQ(10, path.points.front().pose.pose.position.x);
  EXPECT_EQ(7, path.points.front().velocity);
}

TEST(Tf, AppliesFullThreeDimensionalRotation) {
  interfaces::Path path;
  path.header.frame_id = "map";
  path.points.resize(1);
  path.points[0].pose.header = path.header;
  path.points[0].pose.pose.position.y = 1;
  path.points[0].pose.pose.orientation.w = 1;
  tf2_msgs::TFMessage transforms;
  transforms.transforms.resize(1);
  auto& transform = transforms.transforms.front();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.x = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);

  velocity_planner::Tf core;
  const auto output = core.process(path, transforms);
  const auto& pose = output.points.front().pose.pose;
  EXPECT_NEAR(0, pose.position.x, 1e-12);
  EXPECT_NEAR(0, pose.position.y, 1e-12);
  EXPECT_NEAR(-1, pose.position.z, 1e-12);
  EXPECT_NEAR(-std::sqrt(0.5), pose.orientation.x, 1e-12);
  EXPECT_NEAR(std::sqrt(0.5), pose.orientation.w, 1e-12);
}

TEST(TfInputs, DeclaresTransformAndPosePrerequisites) {
  velocity_planner::TfInputs validator;
  node_pipeline::HealthLimits limits;
  limits.frame = "map";
  std::vector<node_pipeline::Kind> issues;
  tf2_msgs::TFMessage transforms;
  validator(transforms, limits, issues);
  ASSERT_EQ(1, issues.size());
  EXPECT_EQ(node_pipeline::Kind::Missing, issues.front());

  transforms.transforms.resize(2);
  transforms.transforms[0].header.frame_id = "unrelated";
  auto& transform = transforms.transforms[1];
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1;
  issues.clear();
  validator(transforms, limits, issues);
  EXPECT_TRUE(issues.empty());
  transform.transform.rotation.w = 0;
  validator(transforms, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::OutRangePositive, issues.front());
  transform.transform.rotation.w = 1;
  transform.transform.translation.x = std::numeric_limits<double>::infinity();
  issues.clear();
  validator(transforms, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::Inf, issues.front());

  interfaces::Path path;
  path.points.resize(1);
  path.points.front().pose.header.frame_id = "map";
  path.points.front().pose.pose.orientation.w = 1;
  issues.clear();
  validator(path, limits, issues);
  EXPECT_TRUE(issues.empty());
  path.points.front().pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  validator(path, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::NaN, issues.front());
  path.points.front().pose.pose.position.x = 0;
  path.points.front().pose.pose.orientation.w = 0;
  issues.clear();
  validator(path, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::OutRangePositive, issues.front());
  path.points.front().pose.pose.orientation.w = 1;
  path.points.front().pose.header.frame_id = "base_link";
  issues.clear();
  validator(path, limits, issues);
  EXPECT_EQ(node_pipeline::Kind::FrameMismatch, issues.front());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
