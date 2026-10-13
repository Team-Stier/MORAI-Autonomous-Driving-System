#include <gtest/gtest.h>
#include <visualizer/cores/ego_vehicle.hpp>
#include <cmath>

TEST(EgoVehicle, FlattenedOriginKeepsNativeBoxHeightAndRearAxleHeading) {
  visualizer::EgoVehicle core;
  nav_msgs::Odometry ego;
  ego.header.frame_id = "map";
  ego.header.stamp = ros::Time(123, 456);
  ego.pose.pose.position.x = 10.0;
  ego.pose.pose.position.y = 20.0;
  ego.pose.pose.position.z = 3.0;
  ego.pose.pose.orientation.w = 1.0;

  const auto output = core.process(ego);
  ASSERT_EQ(output.points.size(), 30u);
  EXPECT_EQ(output.type, visualization_msgs::Marker::LINE_LIST);
  EXPECT_NEAR(output.points[0].x, 13.845008373260498, 1e-9);
  EXPECT_NEAR(output.points[0].y, 21.080417275428772, 1e-9);
  EXPECT_NEAR(output.points[0].z, -0.3684999644756317, 1e-9);
  EXPECT_NEAR(output.points[8].z, 1.2755463123321533, 1e-9);
  EXPECT_DOUBLE_EQ(output.points[24].x, 10.0);
  EXPECT_DOUBLE_EQ(output.points[24].y, 20.0);
  EXPECT_DOUBLE_EQ(output.points[24].z, 1.8);
  EXPECT_DOUBLE_EQ(output.points[25].x, 15.5);
  EXPECT_EQ(output.ns, "ego");
  EXPECT_FLOAT_EQ(output.color.r, 0.0f);
  EXPECT_FLOAT_EQ(output.color.g, 0.85f);
  EXPECT_FLOAT_EQ(output.color.b, 1.0f);
  EXPECT_TRUE(output.header.frame_id.empty());
  EXPECT_TRUE(output.header.stamp.isZero());
  EXPECT_DOUBLE_EQ(output.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(output.lifetime.toSec(), 0.5);
}

TEST(EgoVehicle, YawRotatesCornersAndHeadingAroundRearAxle) {
  visualizer::EgoVehicle core;
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 10.0;
  ego.pose.pose.position.y = 20.0;
  ego.pose.pose.orientation.z = std::sqrt(0.5);
  ego.pose.pose.orientation.w = std::sqrt(0.5);

  const auto output = core.process(ego);
  ASSERT_EQ(output.points.size(), 30u);
  EXPECT_NEAR(output.points[0].x, 8.919582724571228, 1e-9);
  EXPECT_NEAR(output.points[0].y, 23.845008373260498, 1e-9);
  EXPECT_DOUBLE_EQ(output.points[24].x, 10.0);
  EXPECT_DOUBLE_EQ(output.points[24].y, 20.0);
  EXPECT_NEAR(output.points[25].x, 10.0, 1e-9);
  EXPECT_NEAR(output.points[25].y, 25.5, 1e-9);
}

TEST(EgoVehicle, GroundTruthKeepsFullThreeDimensionalCornerRotation) {
  visualizer::EgoVehicle core({true, false});
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 10.0;
  ego.pose.pose.position.y = 20.0;
  ego.pose.pose.position.z = 3.0;
  // Roll=90 degrees, pitch=90 degrees: (x,y,z) maps to (y,-z,-x).
  ego.pose.pose.orientation.x = 0.5;
  ego.pose.pose.orientation.y = 0.5;
  ego.pose.pose.orientation.z = -0.5;
  ego.pose.pose.orientation.w = 0.5;

  const auto output = core.process(ego);
  ASSERT_EQ(output.points.size(), 30u);
  EXPECT_NEAR(output.points[0].x, 11.080417275428772, 1e-9);
  EXPECT_NEAR(output.points[0].y, 20.368499964475632, 1e-9);
  EXPECT_NEAR(output.points[0].z, -0.845008373260498, 1e-9);
  geometry_msgs::Point center;
  for (std::size_t index = 0; index < 24; ++index) {
    center.x += output.points[index].x / 24;
    center.y += output.points[index].y / 24;
    center.z += output.points[index].z / 24;
  }
  EXPECT_NEAR(center.x, 9.999999821186066, 1e-9);
  EXPECT_NEAR(center.y, 19.546476826071739, 1e-9);
  EXPECT_NEAR(center.z, 1.4724956154823303, 1e-9);
  EXPECT_EQ(output.ns, "gt");
  EXPECT_FLOAT_EQ(output.color.r, 1.0f);
  EXPECT_FLOAT_EQ(output.color.g, 0.35f);
  EXPECT_FLOAT_EQ(output.color.b, 0.05f);
  EXPECT_DOUBLE_EQ(output.points[24].z, 4.8);
  EXPECT_NEAR(output.points[25].z, -0.7, 1e-9);
}
