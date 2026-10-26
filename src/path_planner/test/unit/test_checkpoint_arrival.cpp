#include <path_planner/parameters/checkpoint_arrival.hpp>
#include <gtest/gtest.h>

namespace pp = path_planner;

TEST(CheckpointArrival, ReturnsOnlyTheStartLanelet) {
  const auto reader = node_pipeline::ParamReader::fromString(
      "checkpoints_file: checkpoints.csv\n", std::string(NP_PACKAGE_DIR "/../map"));
  const auto parameters = pp::loadCheckpointArrivalParameters(reader);
  ASSERT_EQ(16u, parameters.checkpoints.size());
  EXPECT_EQ(108300, parameters.checkpoints[0].lanelet_id);
  pp::CheckpointArrival core(parameters);
  nav_msgs::Odometry ego;
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = 3;
  interfaces::CompetitionVehicleStatus competition;
  competition.velocity_x_mps = 4;
  interfaces::Objects objects;
  objects.objects.resize(2);
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.75;
  const auto result = core.process(ego, competition, objects, tf, imu);
  EXPECT_EQ(std::get<0>(result), ego);
  EXPECT_EQ(std::get<1>(result), competition);
  EXPECT_EQ(std::get<2>(result), objects);
  EXPECT_EQ(std::get<3>(result), tf);
  EXPECT_EQ(std::get<4>(result), imu);
  const auto remaining = std::get<5>(result);
  ASSERT_EQ(1u, remaining.size());
  EXPECT_EQ(parameters.checkpoints[0].lanelet_id, remaining[0].lanelet_id);
  EXPECT_DOUBLE_EQ(parameters.checkpoints[0].x_m, remaining[0].x_m);
  EXPECT_DOUBLE_EQ(parameters.checkpoints[0].y_m, remaining[0].y_m);
}
