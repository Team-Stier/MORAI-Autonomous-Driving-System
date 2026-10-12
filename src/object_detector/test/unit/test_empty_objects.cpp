#include <object_detector/cores/empty_objects.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cstring>

TEST(EmptyObjects, ReturnsEmptyObjectsAndPreservesCompetition) {
  interfaces::CompetitionVehicleStatus competition;
  competition.header.seq = 7;
  competition.header.stamp = ros::Time(12, 34);
  competition.header.frame_id = "base_link";
  competition.velocity_x_mps = 8.5;
  competition.link_id = "checkpoint_7";
  object_detector::EmptyObjects core;

  const auto result = core.process(competition);
  const auto& objects = std::get<0>(result);
  EXPECT_TRUE(objects.objects.empty());
  EXPECT_TRUE(objects.header.frame_id.empty());
  EXPECT_TRUE(objects.header.stamp.isZero());
  const auto expected = ros::serialization::serializeMessage(competition);
  const auto actual = ros::serialization::serializeMessage(std::get<1>(result));
  ASSERT_EQ(expected.num_bytes, actual.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected.buf.get(), actual.buf.get(), expected.num_bytes));
  EXPECT_TRUE(std::get<0>(core.process({})).objects.empty());
}
