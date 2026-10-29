#include <gtest/gtest.h>
#include <estop/cores/estop_off.hpp>
#include <estop/cores/estop_overlap.hpp>

namespace {
// Straight path along base_link +x from the rear axle, one pose per metre at the given speed.
interfaces::Path path(double stamp_sec, double length_m, double speed_mps) {
  interfaces::Path message;
  message.header.stamp = ros::Time(stamp_sec);
  message.header.frame_id = "base_link";
  for (double x = 0.0; x <= length_m; x += 1.0) {
    interfaces::PathPoint point;
    point.pose.pose.position.x = x;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = speed_mps;
    message.points.push_back(point);
  }
  return message;
}

// map -> base_link with the ego rear axle at (100, 50) heading along +x.
tf2_msgs::TFMessage transforms() {
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 100.0;
  transform.transform.translation.y = 50.0;
  transform.transform.rotation.w = 1.0;
  tf2_msgs::TFMessage message;
  message.transforms.push_back(transform);
  return message;
}

interfaces::Objects objects(double x, double y, double vx = 0.0, double vy = 0.0) {
  interfaces::Object object;
  object.position.x = x;
  object.position.y = y;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1.0;
  object.bbox_size.x = 1.0;
  object.bbox_size.y = 1.0;
  object.bbox_size.z = 1.0;
  object.velocity.x = vx;
  object.velocity.y = vy;
  object.velocity_available = vx != 0.0 || vy != 0.0;
  interfaces::Objects message;
  message.header.frame_id = "map";
  message.objects.push_back(object);
  return message;
}
}  // namespace

TEST(EstopOverlap, StillObjectOnPathRaisesStop) {
  estop::EstopOverlap core{estop::EstopOverlapParameters{}};
  EXPECT_TRUE(core.process(path(10.0, 30.0, 5.0), objects(115.0, 50.0), transforms()).data);
}

TEST(EstopOverlap, ObjectBesideThePathIsClear) {
  estop::EstopOverlap core{estop::EstopOverlapParameters{}};
  EXPECT_FALSE(core.process(path(10.0, 30.0, 5.0), objects(115.0, 55.0), transforms()).data);
}

TEST(EstopOverlap, ObjectBeyondTheStopPointIsNotReached) {
  estop::EstopOverlap core{estop::EstopOverlapParameters{}};
  auto stopping = path(10.0, 30.0, 5.0);
  for (std::size_t i = 6; i < stopping.points.size(); ++i) stopping.points[i].velocity = 0.0;
  EXPECT_FALSE(core.process(stopping, objects(120.0, 50.0), transforms()).data);
}

TEST(EstopOverlap, CrossingObjectMeetsTheEgoInTime) {
  estop::EstopOverlap core{estop::EstopOverlapParameters{}};
  // 2 s to reach x = 110 at 5 m/s; the object starts 4 m to the side and crosses at 2 m/s.
  EXPECT_TRUE(core.process(path(10.0, 30.0, 5.0), objects(110.0, 54.0, 0.0, -2.0), transforms()).data);
  estop::EstopOverlap later{estop::EstopOverlapParameters{}};
  // Starting 20 m away it reaches the path long after the ego has passed.
  EXPECT_FALSE(later.process(path(10.0, 30.0, 5.0), objects(110.0, 70.0, 0.0, -2.0), transforms()).data);
}

TEST(EstopOverlap, StopClearsOnlyAfterTheReleaseTime) {
  estop::EstopOverlap core{estop::EstopOverlapParameters{}};
  EXPECT_TRUE(core.process(path(10.0, 30.0, 5.0), objects(115.0, 50.0), transforms()).data);
  EXPECT_TRUE(core.process(path(10.5, 30.0, 5.0), objects(115.0, 70.0), transforms()).data);
  EXPECT_FALSE(core.process(path(11.0, 30.0, 5.0), objects(115.0, 70.0), transforms()).data);
}

TEST(EstopOverlap, ObjectBehindTheEgoIsIgnored) {
  estop::EstopOverlap core{estop::EstopOverlapParameters{}};
  EXPECT_FALSE(core.process(path(10.0, 30.0, 5.0), objects(97.0, 50.0), transforms()).data);
}

TEST(EstopOff, AlwaysFalse) {
  estop::EstopOff core;
  EXPECT_FALSE(core.process(path(10.0, 30.0, 5.0)).data);
}
