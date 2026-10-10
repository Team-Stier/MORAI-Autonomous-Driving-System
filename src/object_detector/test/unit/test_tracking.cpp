#include <object_detector/cores/tracking.hpp>
#include <gtest/gtest.h>
#include <ros/time.h>
#include <cstdint>
#include <initializer_list>
#include <utility>

namespace {
interfaces::Objects observations(std::initializer_list<std::pair<double, double>> centers) {
  interfaces::Objects objects;
  objects.header.stamp.fromSec(9000);
  for (const auto& center : centers) {
    interfaces::Object object;
    object.label = "unknown";
    object.position.x = center.first;
    object.position.y = center.second;
    object.position.z = 0.75;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.w = 1;
    object.bbox_size.x = 4;
    object.bbox_size.y = 1.8;
    object.bbox_size.z = 1.5;
    objects.objects.push_back(object);
  }
  return objects;
}

ros::Time stamp(double time) {
  ros::Time value;
  value.fromSec(100 + time);
  return value;
}
}  // namespace

TEST(Tracking, PreservesBoxesAndIdsAcrossReorderingAndAssignsNewIds) {
  object_detector::Tracking tracker;
  const auto input = observations({{10, -2}, {10, 2}});
  const auto first = tracker.process(input, stamp(0));
  ASSERT_EQ(2u, first.objects.size());
  EXPECT_GT(first.objects[0].id, 0u);
  EXPECT_NE(first.objects[0].id, first.objects[1].id);
  EXPECT_FALSE(first.objects[0].velocity_available);
  EXPECT_FALSE(first.objects[0].acceleration_available);
  EXPECT_EQ(input.objects[0].position, first.objects[0].position);
  EXPECT_EQ(input.objects[0].bbox_pose, first.objects[0].bbox_pose);
  EXPECT_EQ(input.objects[0].bbox_size, first.objects[0].bbox_size);

  const auto next = tracker.process(observations({{10, 1.8}, {10, -1.8}, {20, 0}}),
                                    stamp(0.1));
  ASSERT_EQ(3u, next.objects.size());
  EXPECT_EQ(first.objects[1].id, next.objects[0].id);
  EXPECT_EQ(first.objects[0].id, next.objects[1].id);
  EXPECT_TRUE(next.objects[0].velocity_available);
  EXPECT_NE(first.objects[0].id, next.objects[2].id);
  EXPECT_NE(first.objects[1].id, next.objects[2].id);
  EXPECT_FALSE(next.objects[2].velocity_available);
}

TEST(Tracking, UsesExplicitStampAndReturnsMapVelocity) {
  object_detector::Tracking tracker;
  interfaces::Object object;
  std::uint32_t id = 0;
  for (int i = 0; i <= 30; ++i) {
    const double time = i * 0.1;
    const auto output = tracker.process(observations({{20 - 5 * time, 3 + 0.4 * time}}),
                                        stamp(time));
    ASSERT_EQ(1u, output.objects.size());
    object = output.objects.front();
    if (i == 0) id = object.id;
    EXPECT_EQ(id, object.id);
  }
  EXPECT_TRUE(object.velocity_available);
  EXPECT_NEAR(-5, object.velocity.x, 0.02);
  EXPECT_NEAR(0.4, object.velocity.y, 0.02);
  EXPECT_DOUBLE_EQ(0, object.velocity.z);
  EXPECT_TRUE(object.acceleration_available);
  EXPECT_NEAR(0, object.acceleration.x, 0.02);
  EXPECT_NEAR(0, object.acceleration.y, 0.02);
}

TEST(Tracking, EstimatesVerticalMapMotionAndPreservesGeometry) {
  object_detector::Tracking tracker;
  std::uint32_t id = 0;
  for (int i = 0; i <= 30; ++i) {
    auto input = observations({{10 + 0.1 * i, 2}});
    auto& measured = input.objects.front();
    measured.position.z = 1.25 + 0.1 * i;
    measured.bbox_pose.position = measured.position;
    measured.bbox_pose.orientation.z = 0.6;
    measured.bbox_pose.orientation.w = 0.8;
    measured.bbox_size.x = 4 + i;
    measured.bbox_size.y = 1.8 + 0.2 * i;
    measured.bbox_size.z = 1.5 + 0.3 * i;
    const auto output = tracker.process(input, stamp(0.1 * i));
    ASSERT_EQ(1u, output.objects.size());
    const auto& object = output.objects.front();
    if (i == 0) id = object.id;
    EXPECT_EQ(id, object.id);
    EXPECT_EQ(measured.position, object.position);
    EXPECT_EQ(measured.bbox_pose, object.bbox_pose);
    EXPECT_EQ(measured.bbox_size, object.bbox_size);
    EXPECT_EQ(i > 0, object.velocity_available);
    if (i == 30) EXPECT_NEAR(1.0, object.velocity.z, 0.02);
    EXPECT_EQ(i >= 2, object.acceleration_available);
  }
}

TEST(Tracking, EmptyFrameDropsTracksAndReappearanceGetsANewId) {
  object_detector::Tracking tracker;
  const auto first = tracker.process(observations({{10, 2}}), stamp(0));
  ASSERT_EQ(1u, first.objects.size());
  EXPECT_TRUE(tracker.process(observations({}), stamp(0.1)).objects.empty());
  const auto next = tracker.process(observations({{10, 2}}), stamp(0.2));
  ASSERT_EQ(1u, next.objects.size());
  EXPECT_NE(first.objects[0].id, next.objects[0].id);
  EXPECT_FALSE(next.objects[0].velocity_available);
}

TEST(Tracking, EstimatesThreeAxisMapAccelerationAfterTwoVelocityObservations) {
  object_detector::Tracking tracker;
  interfaces::Object object;
  for (int i = 0; i <= 100; ++i) {
    const double time = i * 0.1;
    auto input = observations({{10 + 0.2*time*time, 2}});
    input.objects.front().position.z = 1 + 0.3*time*time;
    input.objects.front().bbox_pose.position = input.objects.front().position;
    object = tracker.process(input, stamp(time)).objects.front();
    EXPECT_EQ(i > 0, object.velocity_available);
    EXPECT_EQ(i > 1, object.acceleration_available);
  }
  EXPECT_NEAR(0.4, object.acceleration.x, 0.02);
  EXPECT_NEAR(0, object.acceleration.y, 0.02);
  EXPECT_NEAR(0.6, object.acceleration.z, 0.02);
}
