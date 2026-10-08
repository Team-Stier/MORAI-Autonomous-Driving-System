#include <path_planner/parameters/checkpoint_00.hpp>
#include <path_planner/parameters/checkpoint_05.hpp>
#include <path_planner/parameters/checkpoint_15.hpp>
#include <path_planner/parameters/checkpoint_16.hpp>
#include <gtest/gtest.h>

namespace pp = path_planner;
namespace {
pp::CheckpointRouteParameters parameters() {
  return pp::readCheckpointRouteParameters(node_pipeline::ParamReader::fromString(
      "checkpoints_file: checkpoints.csv\n", std::string(NP_PACKAGE_DIR "/../map")));
}
template<class Core> pp::CheckpointRoute::Result run(Core& core) {
  nav_msgs::Odometry ego;
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = 3;
  interfaces::CompetitionVehicleStatus competition;
  competition.velocity_x_mps = 4;
  interfaces::Objects objects;
  objects.objects.resize(2);
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  const auto result = core.process(ego, competition, objects, tf);
  EXPECT_EQ(std::get<0>(result), ego);
  EXPECT_EQ(std::get<1>(result), competition);
  EXPECT_EQ(std::get<2>(result), objects);
  EXPECT_EQ(std::get<3>(result), tf);
  return result;
}
}  // namespace

TEST(CheckpointCores, PassInputsThroughAndReturnRemainingCheckpoints) {
  const auto all = parameters().checkpoints;
  pp::Checkpoint00 section0(parameters());
  pp::Checkpoint05 section5(parameters());
  pp::Checkpoint15 section15(parameters());
  pp::Checkpoint16 section16(parameters());
  const auto first = std::get<4>(run(section0));
  ASSERT_EQ(16u, first.size());
  EXPECT_EQ(all.front().lanelet_id, first.front().lanelet_id);
  const auto fifth = std::get<4>(run(section5));
  ASSERT_EQ(11u, fifth.size());
  EXPECT_EQ(all[5].lanelet_id, fifth.front().lanelet_id);
  EXPECT_EQ(all.back().lanelet_id, fifth.back().lanelet_id);
  const auto last = std::get<4>(run(section15));
  ASSERT_EQ(1u, last.size());
  EXPECT_EQ(all.back().lanelet_id, last.front().lanelet_id);
  EXPECT_TRUE(std::get<4>(run(section16)).empty());
}

TEST(CheckpointCores, LoaderRejectsMalformedCheckpointFiles) {
  EXPECT_THROW(pp::readCheckpoints("x,y,z,yaw,lanelet_id\n0,0,0,0,1\n"), std::invalid_argument);
  EXPECT_THROW(pp::readCheckpointRouteParameters(node_pipeline::ParamReader::fromString("checkpoints_file: missing.csv\n", "/tmp")),
               std::runtime_error);
}
