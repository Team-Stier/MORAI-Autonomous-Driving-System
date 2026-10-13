#include <visualizer/validators/ego_pose.hpp>
#include <visualizer/validators/object_geometry.hpp>
#include <visualizer/validators/object_info_geometry.hpp>
#include <visualizer/validators/path_points.hpp>
#include <gtest/gtest.h>
#include <limits>

TEST(InputContracts, RejectsInvalidEgoQuaternion) {
  nav_msgs::Odometry ego;
  std::vector<node_pipeline::Kind> issues;
  visualizer::EgoPose{}(ego, {}, issues);
  EXPECT_FALSE(issues.empty());
  ego.pose.pose.orientation.w = 1;
  issues.clear();
  visualizer::EgoPose{}(ego, {}, issues);
  EXPECT_TRUE(issues.empty());
  ego.pose.pose.orientation.x = ego.pose.pose.orientation.w = 1e200;
  visualizer::EgoPose{}(ego, {}, issues);
  EXPECT_FALSE(issues.empty());
}

TEST(InputContracts, RejectsBrokenPathFrameStampAndVelocity) {
  interfaces::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = ros::Time(1);
  path.points.resize(1);
  path.points[0].velocity = std::numeric_limits<double>::quiet_NaN();
  std::vector<node_pipeline::Kind> issues;
  visualizer::PathPoints{}(path, {}, issues);
  EXPECT_EQ(3u, issues.size());
  path.points[0].pose.header = path.header;
  path.points[0].velocity = 0;
  issues.clear();
  visualizer::PathPoints{}(path, {}, issues);
  EXPECT_TRUE(issues.empty());
}

TEST(InputContracts, RejectsInvalidObjectBoxAndRawObjectInfo) {
  interfaces::Objects objects;
  objects.objects.resize(1);
  std::vector<node_pipeline::Kind> issues;
  visualizer::ObjectGeometry{}(objects, {}, issues);
  EXPECT_FALSE(issues.empty());
  auto& box = objects.objects[0];
  box.bbox_pose.orientation.w = 1;
  box.bbox_size.x = box.bbox_size.y = box.bbox_size.z = 1;
  issues.clear();
  visualizer::ObjectGeometry{}(objects, {}, issues);
  EXPECT_TRUE(issues.empty());
  morai_msgs::ObjectStatusList raw;
  raw.npc_list.resize(1);
  raw.npc_list[0].size.x = raw.npc_list[0].size.y = raw.npc_list[0].size.z = 1;
  raw.npc_list[0].heading = std::numeric_limits<double>::infinity();
  visualizer::ObjectInfoGeometry{}(raw, {}, issues);
  EXPECT_FALSE(issues.empty());
}
