#include <velocity_planner/cores/curvature_velocity60.hpp>
#include <velocity_planner/cores/stop_line_pass_through.hpp>
#include <velocity_planner/cores/tf.hpp>
#include <velocity_planner/validators/curvature_inputs.hpp>
#include <velocity_planner/validators/tf_inputs.hpp>
#include <gtest/gtest.h>
#include <limits>
#include <tuple>
#include <vector>

TEST(StationaryPath, PreservesStopGeometryAndAcceptsFreshDepartureSpeeds) {
  interfaces::Path path;
  path.header.frame_id = "map";
  path.header.stamp.fromSec(10.0);
  path.points.resize(3);
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    auto& point = path.points[i];
    point.pose.header = path.header;
    point.pose.pose.position.x = 100.0 + 5.0 * i;
    point.pose.pose.position.y = 200.0;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = 0.0;
    point.acceleration = 0.0;
  }
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  auto& transform = tf.transforms.front();
  transform.header = path.header;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 100.0;
  transform.transform.translation.y = 200.0;
  transform.transform.rotation.w = 1.0;

  node_pipeline::HealthLimits limits;
  limits.frame = "map";
  std::vector<node_pipeline::Kind> issues;
  velocity_planner::CurvatureInputs{}(interfaces::Path{}, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::OutRangeNegative}, issues);
  issues.clear();
  velocity_planner::CurvatureInputs{}(path, limits, issues);
  velocity_planner::CurvatureInputs{}(tf, limits, issues);
  velocity_planner::TfInputs{}(path, limits, issues);
  velocity_planner::TfInputs{}(tf, limits, issues);
  ASSERT_TRUE(issues.empty());

  velocity_planner::CurvatureVelocity60 inject({});
  velocity_planner::StopLineState state({});
  velocity_planner::StopLinePassThrough cutoff(state);
  velocity_planner::Tf to_base;
  for (int frame = 0; frame < 20; ++frame) {
    const auto injected = inject.process(path, {}, {}, tf);
    const auto stopped = cutoff.process(std::get<0>(injected), std::get<3>(injected));
    const auto output = to_base.process(stopped.first, stopped.second);
    ASSERT_EQ(path.points.size(), output.points.size());
    for (std::size_t i = 0; i < output.points.size(); ++i) {
      EXPECT_DOUBLE_EQ(0.0, output.points[i].velocity);
      EXPECT_DOUBLE_EQ(0.0, output.points[i].acceleration);
      EXPECT_DOUBLE_EQ(5.0 * i, output.points[i].pose.pose.position.x);
      EXPECT_DOUBLE_EQ(0.0, output.points[i].pose.pose.position.y);
    }
    EXPECT_EQ(path.header, output.header);  // Pipeline supplies the final base_link headers.
    path.header.stamp.fromSec(10.05 + frame * 0.05);
    for (auto& point : path.points) point.pose.header = path.header;
  }

  for (auto& point : path.points) point.velocity = 3.0;
  auto moving = inject.process(path, {}, {}, tf);
  EXPECT_EQ(path, std::get<0>(moving));
  for (auto& point : path.points) point.velocity = std::numeric_limits<double>::quiet_NaN();
  moving = inject.process(path, {}, {}, tf);
  const auto departure = cutoff.process(std::get<0>(moving), std::get<3>(moving));
  const auto output = to_base.process(departure.first, departure.second);
  ASSERT_EQ(path.points.size(), output.points.size());
  for (const auto& point : output.points) {
    EXPECT_DOUBLE_EQ(60.0 / 3.6, point.velocity);
    EXPECT_DOUBLE_EQ(0.0, point.acceleration);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
