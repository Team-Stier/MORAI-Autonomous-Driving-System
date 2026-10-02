#include <velocity_planner/core_set.hpp>
#include <gtest/gtest.h>
#include <cmath>

TEST(PlainVelocity, InjectsBothSpeedsAndTransformsAllPoses) {
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
    point.velocity = -1;
    point.acceleration = -2;
  }
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(2);
  tf.transforms[0].header.frame_id = "unrelated";
  auto& transform = tf.transforms[1];
  transform.header = path.header;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 10;
  transform.transform.translation.y = 20;
  transform.transform.translation.z = 30;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);

  velocity_planner::CoreSet cores{velocity_planner::PlainVelocity60{}, velocity_planner::PlainVelocity100{}};
  velocity_planner::Data data;
  data.get<0>().sample.message = std::make_shared<const interfaces::Path>(path);
  for (const auto id : {velocity_planner::CoreId::PlainVelocity60, velocity_planner::CoreId::PlainVelocity100}) {
    EXPECT_FALSE(data.hasInputs(cores.requiredInputs(id)));
  }
  data.get<4>().sample.message = std::make_shared<const tf2_msgs::TFMessage>(tf);
  for (const auto id : {velocity_planner::CoreId::PlainVelocity60, velocity_planner::CoreId::PlainVelocity100}) {
    ASSERT_TRUE(data.hasInputs(cores.requiredInputs(id)));
    const auto output = cores.process(id, data);
    const double speed_mps = (id == velocity_planner::CoreId::PlainVelocity60 ? 60.0 : 100.0) / 3.6;
    EXPECT_TRUE(output.header.stamp.isZero());
    EXPECT_TRUE(output.header.frame_id.empty());
    ASSERT_EQ(path.points.size(), output.points.size());
    for (std::size_t i = 0; i < output.points.size(); ++i) {
      const auto& point = output.points[i];
      EXPECT_DOUBLE_EQ(speed_mps, point.velocity);
      EXPECT_DOUBLE_EQ(0, point.acceleration);
      EXPECT_NEAR(1 + i, point.pose.pose.position.x, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.position.y, 1e-12);
      EXPECT_NEAR(2 + i, point.pose.pose.position.z, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.orientation.x, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.orientation.y, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.orientation.z, 1e-12);
      EXPECT_NEAR(1, point.pose.pose.orientation.w, 1e-12);
      EXPECT_EQ(path.points[i].pose.header, point.pose.header);
    }
  }
  EXPECT_EQ(-1, path.points.front().velocity);
  EXPECT_EQ(10, path.points.front().pose.pose.position.x);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
