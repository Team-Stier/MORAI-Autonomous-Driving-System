#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/validators/core_path_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

TEST(CorePathPlanner, UsesMapObjectsWithTranslatedAndRotatedMovingEgo) {
  lanelet::Lanelet lane(1,
      lanelet::LineString3d(2, {lanelet::Point3d(4, 95, 190, 0),
                              lanelet::Point3d(5, 95, 300, 0)}),
      lanelet::LineString3d(3, {lanelet::Point3d(6, 105, 190, 0),
                              lanelet::Point3d(7, 105, 300, 0)}));
  path_planner::Route route = lanelet::routing::LaneletPath({lane});
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(10.0);
  ego.header.frame_id = "map";
  ego.child_frame_id = "base_link";
  ego.pose.pose.position.x = 100;
  ego.pose.pose.position.y = 200;
  ego.pose.pose.orientation.z = ego.pose.pose.orientation.w = std::sqrt(0.5);
  ego.twist.twist.linear.x = 2;
  interfaces::Objects objects;
  objects.header = ego.header;
  path_planner::PredictedObjects predictions;
  predictions.fill(objects);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.z = 9.81;
  path_planner::PathParameters parameters;
  parameters.trajectory_horizon_m = 25;
  parameters.speed_beam_width = 16;
  parameters.speed_time_step_sec = 1;
  path_planner::CorePathPlanner core(parameters);
  const auto free = core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
      tf2_msgs::TFMessage{}, imu, route, predictions);
  ASSERT_FALSE(free.points.empty());
  EXPECT_DOUBLE_EQ(free.points.front().velocity, 2);
  interfaces::Object object;
  object.id = 1;
  object.label = "vehicle";
  object.position.x = 100;
  object.position.y = 201;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.z = object.bbox_pose.orientation.w = std::sqrt(0.5);
  object.bbox_size.x = 4;
  object.bbox_size.y = 40;
  object.bbox_size.z = 1.5;
  object.velocity_available = true;
  objects.objects.push_back(object);
  predictions.fill(objects);
  EXPECT_TRUE(core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
      tf2_msgs::TFMessage{}, imu, route, predictions).points.empty());
}

TEST(CorePathInputs, RejectsUnavailableVelocityWithoutReadingItsNaNValues) {
  interfaces::Objects objects;
  objects.objects.resize(1);
  auto& velocity = objects.objects.front().velocity;
  velocity.x = velocity.y = velocity.z = std::numeric_limits<double>::quiet_NaN();
  std::vector<node_pipeline::Kind> issues;
  const path_planner::CorePathInputs validator;
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);

  objects.objects.front().velocity_available = true;
  objects.objects.front().velocity = geometry_msgs::Vector3{};
  path_planner::PredictedObjects predictions;
  predictions.fill(objects);
  predictions[4].objects.front().velocity_available = false;
  auto& unavailable = predictions[4].objects.front().velocity;
  unavailable.x = unavailable.y = unavailable.z = std::numeric_limits<double>::quiet_NaN();
  issues.clear();
  validator(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);
}

TEST(CorePathInputs, RequiresFiniteAvailableVelocityAndPreservesRouteValidation) {
  interfaces::Objects objects;
  objects.objects.resize(1);
  objects.objects.front().velocity_available = true;
  std::vector<node_pipeline::Kind> issues;
  const path_planner::CorePathInputs validator;
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_TRUE(issues.empty());
  objects.objects.front().velocity.y = std::numeric_limits<double>::quiet_NaN();
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::NaN}), issues);
  issues.clear();
  validator(path_planner::Route{}, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
