#include <visualizer/parameters/object_info.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

namespace {
visualizer::ObjectInfoParameters parameters() {
  visualizer::ObjectInfoParameters params;
  params.yaw_offset_rad = 0;
  params.profiles.push_back({{4.635, 1.892, 2.434, 1.527504, 0, 0.453523,
                              4.635008, 2.160835, 1.644046}});
  return params;
}
morai_msgs::ObjectStatus actor() {
  morai_msgs::ObjectStatus object;
  object.type = 1;
  object.position.x = 10;
  object.position.y = 20;
  object.position.z = 30;
  object.size.x = 4.635;
  object.size.y = 1.892;
  object.size.z = 2.434;
  return object;
}
nav_msgs::Odometry identityEgo() {
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w = 1;
  return ego;
}
}  // namespace

TEST(ObjectInfo, VerifiedProfileRotatesActualRearAxleBoxAtZeroAndNinetyDegrees) {
  auto params = parameters();
  params.offset_xyz = {{5, 6, 7}};
  for (const double heading : {0, 90}) {
    morai_msgs::ObjectStatusList message;
    auto object = actor();
    object.heading = heading;
    message.npc_list.push_back(object);
    const auto marker = visualizer::ObjectInfo(params).process(message, identityEgo());
    ASSERT_EQ(24u, marker.points.size());
    EXPECT_EQ(visualization_msgs::Marker::LINE_LIST, marker.type);
    EXPECT_TRUE(marker.header.frame_id.empty());
    EXPECT_TRUE(marker.header.stamp.isZero());
    EXPECT_FALSE(marker.frame_locked);
    EXPECT_DOUBLE_EQ(0.5, marker.lifetime.toSec());
    EXPECT_FLOAT_EQ(1, marker.color.b);
    const auto& corner = marker.points[0];
    EXPECT_NEAR(heading == 0 ? 14.21 : 16.0804175, corner.x, 1e-9);
    EXPECT_NEAR(heading == 0 ? 24.9195825 : 25.21, corner.y, 1e-9);
    EXPECT_NEAR(36.6315, corner.z, 1e-9);
  }
  morai_msgs::ObjectStatusList message;
  message.npc_list.push_back(actor());
  const auto marker = visualizer::ObjectInfo(params).process(message, identityEgo());
  double min_z = marker.points[0].z;
  double max_z = min_z;
  for (const auto& point : marker.points) {
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  EXPECT_NEAR(36.6315, min_z, 1e-9);
  EXPECT_NEAR(38.275546, max_z, 1e-9);
}

TEST(ObjectInfo, CapturedIoniq5BoxUsesEgoYawAndSubtractsEgoOriginHeight) {
  auto params = parameters();
  params.profiles = {{{4.635000228881836, 1.8919999599456787, 2.434000015258789,
      1.5275043845176697, -1.7881393432617188e-07, 0.4535231739282608,
      4.635007858276367, 2.160834789276123, 1.6440463066101074}}};
  auto ego = identityEgo();
  ego.pose.pose.position.x = 100;
  ego.pose.pose.position.y = 200;
  ego.pose.pose.position.z = 30;
  ego.pose.pose.orientation.z = ego.pose.pose.orientation.w = std::sqrt(0.5);
  auto object = actor();
  object.position.x = 100;
  object.position.y = 210;
  object.heading = 90;
  morai_msgs::ObjectStatusList message;
  message.npc_list.push_back(object);
  const auto marker = visualizer::ObjectInfo(params).process(message, ego);
  ASSERT_EQ(24u, marker.points.size());
  EXPECT_NEAR(9.21, marker.points[0].x, 1e-6);
  EXPECT_NEAR(-1.0804175, marker.points[0].y, 1e-6);
  EXPECT_NEAR(-0.3685, marker.points[0].z, 1e-6);
}

TEST(ObjectInfo, FullEgoQuaternionTransformsThreeDimensionalCorners) {
  auto ego = identityEgo();
  ego.pose.pose.position.x = 10;
  ego.pose.pose.position.y = 20;
  ego.pose.pose.position.z = 30;
  ego.pose.pose.orientation.x = ego.pose.pose.orientation.y = ego.pose.pose.orientation.w = 0.5;
  ego.pose.pose.orientation.z = -0.5;
  morai_msgs::ObjectStatusList message;
  message.npc_list.push_back(actor());
  const auto marker = visualizer::ObjectInfo(parameters()).process(message, ego);
  ASSERT_EQ(24u, marker.points.size());
  EXPECT_NEAR(0.3685, marker.points[0].x, 1e-9);
  EXPECT_NEAR(-0.79, marker.points[0].y, 1e-9);
  EXPECT_NEAR(1.0804175, marker.points[0].z, 1e-9);
}

TEST(ObjectInfo, EgoColliderPreservesCalibratedCornersUnderRollAndPitch) {
  auto ego = identityEgo();
  ego.pose.pose.position.x = 10;
  ego.pose.pose.position.y = 20;
  ego.pose.pose.position.z = 30;
  ego.pose.pose.orientation.x = ego.pose.pose.orientation.y = ego.pose.pose.orientation.w = 0.5;
  ego.pose.pose.orientation.z = -0.5;
  auto object = actor();
  object.type = -1;
  morai_msgs::ObjectStatusList message;
  message.npc_list.push_back(object);
  const auto marker = visualizer::ObjectInfo(parameters()).process(message, ego);
  ASSERT_EQ(24u, marker.points.size());
  EXPECT_NEAR(-0.79, marker.points[0].x, 1e-9);
  EXPECT_NEAR(-1.0804175, marker.points[0].y, 1e-9);
  EXPECT_NEAR(-0.3685, marker.points[0].z, 1e-9);
}

TEST(ObjectInfo, UnknownAndAmbiguousProfilesExposeOriginAndHeadingForEveryKind) {
  auto params = parameters();
  params.profiles.clear();
  morai_msgs::ObjectStatusList message;
  auto object = actor();
  object.heading = 90;
  message.npc_list.push_back(object);
  message.pedestrian_list.push_back(object);
  message.obstacle_list.push_back(object);
  const auto marker = visualizer::ObjectInfo(params).process(message, identityEgo());
  ASSERT_EQ(18u, marker.points.size());
  EXPECT_DOUBLE_EQ(9.5, marker.points[0].x);
  EXPECT_DOUBLE_EQ(20, marker.points[0].y);
  EXPECT_DOUBLE_EQ(30, marker.points[0].z);
  EXPECT_NEAR(10, marker.points[5].x, 1e-9);
  EXPECT_NEAR(22.5, marker.points[5].y, 1e-9);
  params = parameters();
  params.profiles.push_back(params.profiles.front());
  message.pedestrian_list.clear();
  message.obstacle_list.clear();
  EXPECT_EQ(6u, visualizer::ObjectInfo(params).process(message, identityEgo()).points.size());
  params = parameters();
  message.npc_list.front().type = 2;
  EXPECT_EQ(6u, visualizer::ObjectInfo(params).process(message, identityEgo()).points.size());
  const auto empty = visualizer::ObjectInfo(params).process(morai_msgs::ObjectStatusList{}, identityEgo());
  EXPECT_TRUE(empty.points.empty());
  EXPECT_EQ(visualization_msgs::Marker::DELETE, empty.action);
}

TEST(ObjectInfo, LoadsOnlyFiniteNineValueProfilesAndAllowsNoGeometryProfiles) {
  const std::string base = "yaw_offset_rad: 0\noffset_xyz: [1, 2, 3]\n";
  auto params = visualizer::loadObjectInfoParameters(node_pipeline::ParamReader::fromString(base
      + "profiles:\n  ioniq5: [4.635, 1.892, 2.434, 1.527504, 0, 0.453523, 4.635008, 2.160835, 1.644046]\n"));
  ASSERT_EQ(1u, params.profiles.size());
  EXPECT_DOUBLE_EQ(1, params.offset_xyz[0]);
  EXPECT_DOUBLE_EQ(1.644046, params.profiles[0][8]);
  EXPECT_TRUE(visualizer::loadObjectInfoParameters(node_pipeline::ParamReader::fromString(base + "profiles: {}\n")).profiles.empty());
  EXPECT_THROW(visualizer::loadObjectInfoParameters(node_pipeline::ParamReader::fromString(base
      + "profiles: {bad: [1, 2, 3]}\n")), std::invalid_argument);
  EXPECT_THROW(visualizer::loadObjectInfoParameters(node_pipeline::ParamReader::fromString(base
      + "profiles: {bad: [1, 2, 3, 0, 0, 0, 1, -2, 3]}\n")), std::invalid_argument);
}
