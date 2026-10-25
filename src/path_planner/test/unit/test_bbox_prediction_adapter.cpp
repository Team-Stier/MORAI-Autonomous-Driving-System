#include <path_planner/cores/bbox_prediction_adapter.hpp>
#include <path_planner/validators/bbox_prediction_inputs.hpp>
#include <path_planner/validators/roundabout_merge_inputs.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pp = path_planner;
namespace {
interfaces::Objects observation(std::size_t count = 12) {
  interfaces::Objects objects;
  objects.header.seq = 27;
  objects.header.stamp.fromSec(123.25);
  objects.header.frame_id = "map";
  for (std::size_t j = 0; j < count; ++j) {
    interfaces::Object object;
    object.id = 42 + j;
    object.label = j % 2 == 0 ? "vehicle" : "unknown";
    object.position.x = 10.0 * j;
    object.position.y = -3.0;
    object.position.z = 1.0;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.w = 1.0;
    object.bbox_size.x = 4.5;
    object.bbox_size.y = 1.8;
    object.bbox_size.z = 1.5;
    object.velocity.x = 20.0;
    object.acceleration.y = 2.0;
    object.velocity_available = true;
    object.acceleration_available = true;
    object.prediction_available = true;
    for (std::size_t i = 0; i < object.predicted_bbox_poses.size(); ++i) {
      auto& pose = object.predicted_bbox_poses[i];
      const double horizon_sec = 0.5 * (i + 1);
      pose.position.x = object.position.x + 2.0 * horizon_sec;
      pose.position.y = -3.0 + horizon_sec * horizon_sec;
      pose.position.z = 1.0 + 0.2 * horizon_sec;
      pose.orientation.z = std::sin(0.1 * horizon_sec);
      pose.orientation.w = std::cos(0.1 * horizon_sec);
    }
    objects.objects.push_back(object);
  }
  return objects;
}

std::vector<node_pipeline::Kind> issues(const interfaces::Objects& objects) {
  std::vector<node_pipeline::Kind> result;
  pp::BboxPredictionInputs{}(objects, node_pipeline::HealthLimits{}, result);
  return result;
}

bool contains(const std::vector<node_pipeline::Kind>& issues, node_pipeline::Kind kind) {
  return std::find(issues.begin(), issues.end(), kind) != issues.end();
}
}  // namespace

TEST(BboxPredictionAdapter, TransposesEveryObjectAndHorizonWithoutChangingObservedInputs) {
  auto objects = observation();
  const auto original = objects;
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 500.0;
  interfaces::CompetitionVehicleStatus competition;
  competition.gear = 4;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms.front().child_frame_id = "base_link";
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.3;
  const pp::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{});
  const auto result = pp::BboxPredictionAdapter{}.process(ego, competition, objects, tf, imu, route);
  EXPECT_EQ(std::get<0>(result), ego);
  EXPECT_EQ(std::get<1>(result), competition);
  EXPECT_EQ(std::get<2>(result), original);
  EXPECT_EQ(std::get<3>(result), tf);
  EXPECT_EQ(std::get<4>(result), imu);
  EXPECT_EQ(std::get<5>(result), route);
  const auto& predictions = std::get<6>(result);
  ASSERT_EQ(predictions.size(), 10u);
  for (std::size_t i = 0; i < predictions.size(); ++i) {
    EXPECT_EQ(predictions[i].header, original.header);
    ASSERT_EQ(predictions[i].objects.size(), original.objects.size());
    for (std::size_t j = 0; j < original.objects.size(); ++j) {
      const auto& future = predictions[i].objects[j];
      const auto& current = original.objects[j];
      EXPECT_EQ(future.id, current.id);
      EXPECT_EQ(future.label, current.label);
      EXPECT_EQ(future.bbox_size, current.bbox_size);
      EXPECT_EQ(future.bbox_pose, current.predicted_bbox_poses[i]);
      EXPECT_EQ(future.position, current.predicted_bbox_poses[i].position);
      EXPECT_FALSE(future.prediction_available);
      EXPECT_FALSE(future.velocity_available);
      EXPECT_FALSE(future.acceleration_available);
      EXPECT_EQ(future.velocity, geometry_msgs::Vector3{});
      EXPECT_EQ(future.acceleration, geometry_msgs::Vector3{});
    }
  }
  EXPECT_EQ(objects, original);
  std::vector<node_pipeline::Kind> downstream_issues;
  pp::RoundaboutMergeInputs{}(predictions, node_pipeline::HealthLimits{}, downstream_issues);
  EXPECT_TRUE(downstream_issues.empty());
}

TEST(BboxPredictionAdapter, KeepsTenEmptySnapshotsAndObservationHeader) {
  const auto objects = observation(0);
  const auto predictions = std::get<6>(pp::BboxPredictionAdapter{}.process(nav_msgs::Odometry{},
      interfaces::CompetitionVehicleStatus{}, objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  ASSERT_EQ(predictions.size(), 10u);
  for (const auto& snapshot : predictions) EXPECT_EQ(snapshot, objects);
  EXPECT_TRUE(issues(objects).empty());
}

TEST(BboxPredictionInputs, AcceptsPosePredictionsWithoutObservedMotionEstimates) {
  auto objects = observation();
  for (auto& object : objects.objects) {
    object.velocity_available = false;
    object.acceleration_available = false;
    object.velocity.x = std::numeric_limits<double>::quiet_NaN();
    object.acceleration.y = std::numeric_limits<double>::infinity();
  }
  EXPECT_TRUE(issues(objects).empty());
}

TEST(BboxPredictionInputs, ReportsMissingIfAnyObjectHasNoPredictionAndIgnoresItsPoseArray) {
  auto objects = observation();
  auto& unavailable = objects.objects.back();
  unavailable.prediction_available = false;
  unavailable.predicted_bbox_poses[0].position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(issues(objects), std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing});
}

TEST(BboxPredictionInputs, RejectsNonfiniteFuturePositionsAndOrientations) {
  auto objects = observation(1);
  objects.objects.front().predicted_bbox_poses.back().position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(contains(issues(objects), node_pipeline::Kind::NaN));
  objects = observation(1);
  objects.objects.front().predicted_bbox_poses[4].orientation.z = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(contains(issues(objects), node_pipeline::Kind::Inf));
}

TEST(BboxPredictionInputs, RejectsZeroAndNonunitFutureQuaternions) {
  for (double w : {0.0, 2.0}) {
    auto objects = observation(1);
    auto& orientation = objects.objects.front().predicted_bbox_poses.back().orientation;
    orientation = geometry_msgs::Quaternion{};
    orientation.w = w;
    EXPECT_TRUE(contains(issues(objects), node_pipeline::Kind::OutRangePositive));
  }
}

TEST(BboxPredictionInputs, RejectsNonpositiveAndNonfiniteBboxSizes) {
  for (double size : {0.0, -1.0}) {
    auto objects = observation(1);
    objects.objects.front().bbox_size.y = size;
    EXPECT_TRUE(contains(issues(objects), node_pipeline::Kind::OutRangeNegative));
  }
  auto objects = observation(1);
  objects.objects.front().bbox_size.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(contains(issues(objects), node_pipeline::Kind::NaN));
  objects.objects.front().bbox_size.z = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(contains(issues(objects), node_pipeline::Kind::Inf));
}
