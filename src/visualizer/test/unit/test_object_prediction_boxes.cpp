#include <gtest/gtest.h>
#include <visualizer/cores/object_prediction_boxes.hpp>
#include <visualizer/validators/object_prediction_geometry.hpp>
#include <limits>

using TestedBoxes = visualizer::ObjectPredictionBoxes;
using TestedGeometry = visualizer::ObjectPredictionGeometry;

TEST(ObjectPredictionBoxes, ElevenBoxesUseCurrentThenHalfSecondFuturePoses) {
  interfaces::Objects input;
  input.objects.resize(1);
  auto& object = input.objects.front();
  object.bbox_pose.position.x = 10;
  object.bbox_pose.position.z = 3;
  object.bbox_pose.orientation.w = 1;
  object.bbox_size.x = 4;
  object.bbox_size.y = 2;
  object.bbox_size.z = 2;
  object.prediction_available = true;
  for (unsigned i = 0; i < 10; ++i) {
    object.predicted_bbox_poses[i] = object.bbox_pose;
    object.predicted_bbox_poses[i].position.x += i + 1;
  }
  object.predicted_bbox_poses[9].orientation.z = 1;
  object.predicted_bbox_poses[9].orientation.w = 0;

  const auto marker = TestedBoxes().process(input);
  ASSERT_EQ(marker.points.size(), 264u);
  ASSERT_EQ(marker.colors.size(), marker.points.size());
  EXPECT_DOUBLE_EQ(marker.points[0].x, 8);
  EXPECT_DOUBLE_EQ(marker.points[24].x, 9);
  EXPECT_DOUBLE_EQ(marker.points[240].x, 22);
  EXPECT_DOUBLE_EQ(marker.points[240].y, 1);
  EXPECT_DOUBLE_EQ(marker.points[240].z, 2);
  EXPECT_GT(marker.colors[0].b, marker.colors[0].g);
  EXPECT_GT(marker.colors[24].g, marker.colors[24].r);
  EXPECT_GT(marker.colors[240].r, marker.colors[240].g);
  EXPECT_TRUE(marker.header.frame_id.empty());
  EXPECT_TRUE(marker.header.stamp.isZero());
}

TEST(ObjectPredictionBoxes, UnavailablePredictionDrawsOnlyCurrentAndDeletesPreviousForecast) {
  interfaces::Objects input;
  input.objects.resize(1);
  auto& object = input.objects.front();
  object.bbox_pose.orientation.w = 1;
  object.bbox_size.x = 4;
  object.bbox_size.y = 2;
  object.bbox_size.z = 2;
  TestedBoxes core;
  const auto current = core.process(input);
  ASSERT_EQ(current.points.size(), 24u);
  EXPECT_EQ(current.action, visualization_msgs::Marker::ADD);
  input.objects.clear();
  const auto empty = core.process(input);
  EXPECT_EQ(empty.action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(empty.ns, current.ns);
  EXPECT_EQ(empty.id, current.id);
  EXPECT_FALSE(empty.frame_locked);
}

TEST(ObjectPredictionGeometry, RejectsInvalidAvailableFutureButIgnoresUnavailableStorage) {
  interfaces::Objects input;
  input.objects.resize(1);
  auto& object = input.objects.front();
  object.bbox_pose.orientation.w = 1;
  object.bbox_size.x = 4;
  object.bbox_size.y = 2;
  object.bbox_size.z = 2;
  for (auto& pose : object.predicted_bbox_poses) pose.orientation.w = 1;
  object.predicted_bbox_poses[4].position.x = std::numeric_limits<double>::quiet_NaN();
  std::vector<node_pipeline::Kind> issues;
  TestedGeometry validator;
  validator(input, {}, issues);
  EXPECT_TRUE(issues.empty());
  object.prediction_available = true;
  validator(input, {}, issues);
  EXPECT_FALSE(issues.empty());
  issues.clear();
  object.predicted_bbox_poses[4].position.x = 0;
  object.predicted_bbox_poses[8].orientation.w = 2;
  validator(input, {}, issues);
  EXPECT_FALSE(issues.empty());
}
