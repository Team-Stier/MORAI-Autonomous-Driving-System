#include <gtest/gtest.h>
#include <visualizer/cores/object_boxes.hpp>
#include <algorithm>

TEST(ObjectBoxes, FullQuaternionRotatesNativeBoundingBoxVertices) {
  visualizer::ObjectBoxes core({-1, false});
  interfaces::Objects input;
  input.header.frame_id = "map";
  input.header.stamp = ros::Time(123, 456);
  input.objects.resize(1);
  auto& object = input.objects.front();
  object.bbox_pose.position.x = 10.0;
  object.bbox_pose.position.y = 20.0;
  object.bbox_pose.position.z = 30.0;
  object.bbox_pose.orientation.x = 0.5;
  object.bbox_pose.orientation.y = 0.5;
  object.bbox_pose.orientation.z = -0.5;
  object.bbox_pose.orientation.w = 0.5;
  object.bbox_size.x = 4.0;
  object.bbox_size.y = 2.0;
  object.bbox_size.z = 6.0;

  const auto marker = core.process(input);
  ASSERT_EQ(marker.points.size(), 24u);
  EXPECT_EQ(marker.type, visualization_msgs::Marker::LINE_LIST);
  EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);
  EXPECT_NEAR(marker.points[0].x, 9.0, 1e-9);
  EXPECT_NEAR(marker.points[0].y, 23.0, 1e-9);
  EXPECT_NEAR(marker.points[0].z, 32.0, 1e-9);
  EXPECT_NEAR(marker.points[1].x, 9.0, 1e-9);
  EXPECT_NEAR(marker.points[1].y, 23.0, 1e-9);
  EXPECT_NEAR(marker.points[1].z, 28.0, 1e-9);
  EXPECT_FALSE(marker.frame_locked);
  EXPECT_TRUE(marker.header.frame_id.empty());
  EXPECT_TRUE(marker.header.stamp.isZero());
  EXPECT_DOUBLE_EQ(marker.lifetime.toSec(), 0.5);
}

TEST(ObjectBoxes, MultipleBoxesShareOneMarker) {
  visualizer::ObjectBoxes core;
  interfaces::Objects input;
  input.objects.resize(2);
  for (auto& object : input.objects) {
    object.bbox_pose.position.z = 4.0;
    object.bbox_pose.orientation.w = 1.0;
    object.bbox_size.x = 4.0;
    object.bbox_size.y = 2.0;
    object.bbox_size.z = 2.0;
  }
  input.objects[1].bbox_pose.position.x = 10.0;
  const auto marker = core.process(input);
  ASSERT_EQ(marker.points.size(), 48u);
  EXPECT_DOUBLE_EQ(marker.points[0].x, -2.0);
  EXPECT_DOUBLE_EQ(marker.points[0].z, 3.0);
  EXPECT_DOUBLE_EQ(marker.points[24].x, 8.0);
  EXPECT_EQ(marker.ns, "objects");
  EXPECT_EQ(marker.id, 0);
  EXPECT_FLOAT_EQ(marker.color.r, 0.95f);
  EXPECT_FLOAT_EQ(marker.color.g, 0.15f);
  EXPECT_FLOAT_EQ(marker.color.b, 1.0f);
}

TEST(ObjectBoxes, FlatteningTranslatesCenterAndPreservesRotation) {
  interfaces::Objects input;
  input.objects.resize(1);
  auto& object = input.objects.front();
  object.bbox_pose.position.z = 30.0;
  object.bbox_pose.orientation.x = 0.5;
  object.bbox_pose.orientation.y = 0.5;
  object.bbox_pose.orientation.z = -0.5;
  object.bbox_pose.orientation.w = 0.5;
  object.bbox_size.x = 4.0;
  object.bbox_size.y = 2.0;
  object.bbox_size.z = 6.0;
  const auto actual = visualizer::ObjectBoxes({-1, false}).process(input);
  const auto flattened = visualizer::ObjectBoxes({-1, true}).process(input);
  ASSERT_EQ(actual.points.size(), flattened.points.size());
  for (std::size_t i = 0; i < actual.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(flattened.points[i].x, actual.points[i].x);
    EXPECT_DOUBLE_EQ(flattened.points[i].y, actual.points[i].y);
    EXPECT_NEAR(flattened.points[i].z, actual.points[i].z - 27.0, 1e-9);
  }
  const auto extremes = std::minmax_element(flattened.points.begin(), flattened.points.end(),
      [](const auto& a, const auto& b) { return a.z < b.z; });
  EXPECT_NEAR(extremes.first->z, 1.0, 1e-9);
  EXPECT_NEAR(extremes.second->z, 5.0, 1e-9);
}

TEST(ObjectBoxes, PredictionHorizonsHaveIndependentDeleteAndGradient) {
  interfaces::Objects empty;
  const auto detected = visualizer::ObjectBoxes().process(empty);
  EXPECT_EQ(detected.action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(detected.ns, "objects");
  const auto first = visualizer::ObjectBoxes({0, true}).process(empty);
  const auto last = visualizer::ObjectBoxes({9, true}).process(empty);
  EXPECT_EQ(first.action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(last.action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(first.ns, "predicted_objects_0");
  EXPECT_EQ(last.ns, "predicted_objects_9");
  EXPECT_EQ(first.id, detected.id);
  EXPECT_EQ(last.id, detected.id);
  EXPECT_TRUE(first.points.empty());
  EXPECT_FLOAT_EQ(first.color.r, 0.0f);
  EXPECT_FLOAT_EQ(first.color.g, 1.0f);
  EXPECT_FLOAT_EQ(last.color.r, 1.0f);
  EXPECT_FLOAT_EQ(last.color.g, 0.35f);
  EXPECT_FLOAT_EQ(last.color.b, 0.15f);
}
