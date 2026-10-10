#include <object_detector/cores/tracker.hpp>
#include <gtest/gtest.h>
#include <cmath>

namespace pkg = object_detector;
namespace {
interfaces::Object box(double x, double y) {
  interfaces::Object object;
  object.label = "unknown";
  object.position.x = x;
  object.position.y = y;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1;
  object.bbox_size.x = 4;
  object.bbox_size.y = 1.8;
  object.bbox_size.z = 1.5;
  return object;
}
interfaces::Objects observations(std::initializer_list<interfaces::Object> boxes) {
  interfaces::Objects objects;
  objects.objects = boxes;
  return objects;
}
interfaces::CompetitionVehicleStatus status(double time, double yaw = 0, double speed = 0) {
  interfaces::CompetitionVehicleStatus value;
  value.header.stamp.fromSec(100 + time);
  value.header.frame_id = "base_link";
  value.yaw_rad = yaw;
  value.velocity_x_mps = speed;
  return value;
}
pkg::TrackerParameters parameters() {
  pkg::TrackerParameters values;
  values.yaw_offset_rad = 0;
  return values;
}
}  // namespace

TEST(Tracker, FirstObservationAssignsIdWithoutInventingMotion) {
  pkg::Tracker tracker(parameters());
  const auto input = observations({box(10, 2)});
  const auto output = tracker.process(input, status(0));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects.front();
  EXPECT_GT(object.id, 0u);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);
  EXPECT_EQ(object.position, input.objects.front().position);
  EXPECT_EQ(object.bbox_pose, input.objects.front().bbox_pose);
  EXPECT_EQ(object.bbox_size, input.objects.front().bbox_size);
  EXPECT_TRUE(output.header.stamp.isZero());
}

TEST(Tracker, EgoTranslationAndHeadingDoNotBecomeStaticObjectMotion) {
  pkg::Tracker tracker(parameters());
  std::uint32_t id = 0;
  interfaces::Object object;
  for (int i = 0; i < 8; ++i) {
    const double time = 0.1 * i;
    const auto output = tracker.process(observations({box(20 - 5*time, 0)}),
      status(time, 1.5707963267948966, 5));
    object = output.objects.front();
    if (i == 0) id = object.id;
    EXPECT_EQ(id, object.id);
  }
  ASSERT_TRUE(object.velocity_available);
  ASSERT_TRUE(object.acceleration_available);
  EXPECT_NEAR(0, object.velocity.x, 1e-5);
  EXPECT_NEAR(0, object.velocity.y, 1e-5);
  EXPECT_NEAR(0, object.acceleration.x, 1e-5);
  EXPECT_NEAR(0, object.acceleration.y, 1e-5);
  EXPECT_NEAR(16.5, object.position.x, 1e-10);
}

TEST(Tracker, FitsMapVelocityAndAccelerationFromInternalHistory) {
  pkg::Tracker tracker(parameters());
  interfaces::Object object;
  for (int i = 0; i < 8; ++i) {
    const double time = 0.1 * i;
    // Map x = 5 + t + t^2; base yaw = +90 degrees.
    object = tracker.process(observations({box(10, -(5 + time + time*time))}),
      status(time, 1.5707963267948966)).objects.front();
    if (i < 4) EXPECT_FALSE(object.acceleration_available);
  }
  ASSERT_TRUE(object.velocity_available);
  ASSERT_TRUE(object.acceleration_available);
  EXPECT_NEAR(2.4, object.velocity.x, 1e-5);
  EXPECT_NEAR(0, object.velocity.y, 1e-5);
  EXPECT_NEAR(2, object.acceleration.x, 1e-5);
  EXPECT_NEAR(0, object.acceleration.y, 1e-5);
  EXPECT_EQ(object.position, object.bbox_pose.position);
}

TEST(Tracker, EgoRotationDoesNotBecomeStaticObjectAcceleration) {
  pkg::Tracker tracker(parameters());
  interfaces::Object object;
  for (int i = 0; i < 11; ++i) {
    const double time = i * 0.1, yaw = time * 0.4;
    const double x = 10 * std::cos(yaw) + 3 * std::sin(yaw);
    const double y = -10 * std::sin(yaw) + 3 * std::cos(yaw);
    object = tracker.process(observations({box(x,y)}), status(time,yaw)).objects.front();
  }
  ASSERT_TRUE(object.acceleration_available);
  EXPECT_NEAR(0, object.velocity.x, 1e-5);
  EXPECT_NEAR(0, object.velocity.y, 1e-5);
  EXPECT_NEAR(0, object.acceleration.x, 1e-5);
  EXPECT_NEAR(0, object.acceleration.y, 1e-5);
}

TEST(Tracker, PreservesIdsAcrossReorderAndShortOcclusionAndExpiresOldTrack) {
  pkg::Tracker tracker(parameters());
  const auto first = tracker.process(observations({box(10,-2),box(10,2)}),status(0));
  const auto lower_id = first.objects[0].id, upper_id = first.objects[1].id;
  const auto second = tracker.process(observations({box(10,1.6),box(10,-1.6)}),status(0.1));
  EXPECT_EQ(upper_id, second.objects[0].id);
  EXPECT_EQ(lower_id, second.objects[1].id);
  EXPECT_TRUE(tracker.process(observations({}),status(0.2)).objects.empty());
  const auto reappeared = tracker.process(observations({box(10,-0.8),box(10,0.8)}),status(0.3));
  EXPECT_EQ(lower_id, reappeared.objects[0].id);
  EXPECT_EQ(upper_id, reappeared.objects[1].id);
  tracker.process(observations({}),status(1));
  const auto expired = tracker.process(observations({box(10,0.8)}),status(1.1));
  EXPECT_NE(upper_id, expired.objects[0].id);
  EXPECT_NE(lower_id, expired.objects[0].id);
}

TEST(Tracker, CircularMotionProvidesMapTurnEvidence) {
  pkg::Tracker tracker(parameters());
  interfaces::Object object;
  for (int i = 0; i <= 10; ++i) {
    const double time = i * 0.1;
    object = tracker.process(observations({box(20 + 10*std::cos(0.4*time),
      10*std::sin(0.4*time))}),status(time)).objects.front();
  }
  ASSERT_TRUE(object.acceleration_available);
  EXPECT_NEAR(-1.557673369, object.velocity.x, 0.03);
  EXPECT_NEAR(3.684243976, object.velocity.y, 0.03);
  EXPECT_NEAR(-1.473697590, object.acceleration.x, 0.08);
  EXPECT_NEAR(-0.623069348, object.acceleration.y, 0.08);
}

TEST(Tracker, PartialRoofThenFullBodyKeepsTheSameTrack) {
  pkg::Tracker tracker(parameters());
  const auto body = box(10, 2);
  auto roof = body;
  roof.bbox_size.z = 0.02;
  const auto first = tracker.process(observations({body}), status(0));
  const auto partial = tracker.process(observations({roof}), status(0.1));
  const auto visible = tracker.process(observations({body}), status(0.2));
  EXPECT_EQ(first.objects.front().id, partial.objects.front().id);
  EXPECT_EQ(first.objects.front().id, visible.objects.front().id);
  EXPECT_EQ(body.bbox_size, visible.objects.front().bbox_size);
}
