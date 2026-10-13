#include <gtest/gtest.h>
#include <visualizer/cores/route_mesh.hpp>

namespace {
visualizer::RouteMeshParameters parameters(bool flatten_z = true) {
  lanelet::LineString3d left(20, {lanelet::Point3d(10, 0, 2, 3), lanelet::Point3d(11, 10, 2, 3)});
  lanelet::LineString3d right(21, {lanelet::Point3d(12, 0, -2, 3), lanelet::Point3d(13, 10, -2, 3)});
  visualizer::RouteMeshParameters result;
  result.map = lanelet::utils::createMap({lanelet::Lanelet(30, left, right)});
  result.flatten_z = flatten_z;
  return result;
}

nav_msgs::Path path() {
  nav_msgs::Path result;
  result.header.frame_id = "map";
  result.header.stamp = ros::Time(123, 456);
  result.poses.resize(2);
  result.poses[0].pose.position.x = 1;
  result.poses[1].pose.position.x = 9;
  result.poses[0].pose.position.z = result.poses[1].pose.position.z = 3;
  return result;
}
}  // namespace

TEST(RouteMesh, LaneFillAndBoundariesShareOneTriangleBatch) {
  visualizer::RouteMesh core(parameters());
  const auto marker = core.process(path());
  EXPECT_EQ(marker.ns, "route");
  EXPECT_EQ(marker.type, visualization_msgs::Marker::TRIANGLE_LIST);
  EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);
  ASSERT_EQ(marker.points.size(), 18u);
  ASSERT_EQ(marker.colors.size(), marker.points.size());
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(marker.points[i].z, 0.02);
    EXPECT_FLOAT_EQ(marker.colors[i].a, 0.35f);
  }
  for (std::size_t i = 6; i < 18; ++i) {
    EXPECT_DOUBLE_EQ(marker.points[i].z, 0.04);
    EXPECT_FLOAT_EQ(marker.colors[i].a, 1.0f);
  }
  EXPECT_TRUE(marker.header.frame_id.empty());
  EXPECT_TRUE(marker.header.stamp.isZero());
}

TEST(RouteMesh, RepeatedPositionsReusePayloadAndUnmatchedRouteDeletes) {
  visualizer::RouteMesh core(parameters(false));
  auto input = path();
  const auto original = core.process(input);
  input.header.stamp = ros::Time(200);
  input.poses[0].pose.orientation.w = 1;
  const auto repeated = core.process(input);
  EXPECT_EQ(repeated.points, original.points);
  EXPECT_EQ(repeated.colors, original.colors);
  EXPECT_TRUE(repeated.header.stamp.isZero());
  EXPECT_DOUBLE_EQ(repeated.points.front().z, 3.02);
  input.poses[0].pose.position.y = input.poses[1].pose.position.y = 10;
  const auto changed = core.process(input);
  EXPECT_EQ(changed.action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(changed.ns, original.ns);
  EXPECT_EQ(changed.id, original.id);
  EXPECT_TRUE(changed.points.empty());
  EXPECT_TRUE(changed.colors.empty());
}

TEST(RouteMesh, EmptyPathDeletesExistingBatch) {
  visualizer::RouteMesh core(parameters());
  const auto initial = core.process(nav_msgs::Path{});
  EXPECT_EQ(initial.action, visualization_msgs::Marker::DELETE);
  core.process(path());
  const auto cleared = core.process(nav_msgs::Path{});
  EXPECT_EQ(cleared.action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(cleared.ns, initial.ns);
  EXPECT_EQ(cleared.id, initial.id);
  EXPECT_TRUE(cleared.points.empty());
  EXPECT_TRUE(cleared.colors.empty());
}
