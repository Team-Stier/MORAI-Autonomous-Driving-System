#include <object_detector/cores/objects_pass_through.hpp>
#include <gtest/gtest.h>

TEST(ObjectsPassThrough, ReturnsDetectedObjectsUnchanged) {
  interfaces::Objects detected;
  interfaces::Object object;
  object.label = "unknown";
  object.position.x = 4.0;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1.0;
  object.bbox_size.x = 4.5;
  detected.objects.push_back(object);
  object_detector::ObjectsPassThrough core;
  const auto tracked = core.process(detected);
  ASSERT_EQ(1u, tracked.objects.size());
  EXPECT_EQ(detected, tracked);
  EXPECT_EQ(0u, tracked.objects.front().id);
  EXPECT_FALSE(tracked.objects.front().velocity_available);
}
