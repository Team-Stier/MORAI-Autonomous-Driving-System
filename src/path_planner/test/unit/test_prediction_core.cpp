#include <path_planner/cores/prediction_core.hpp>
#include <path_planner/cores/core_prediction.hpp>
#include <path_planner/validators/prediction_motion_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace pp = path_planner;
namespace {
constexpr double kPi = 3.14159265358979323846;

interfaces::Objects observation() {
  interfaces::Objects objects;
  objects.header.seq = 17;
  objects.header.stamp.fromSec(123.25);
  objects.header.frame_id = "map";
  interfaces::Object object;
  object.id = 42;
  object.label = "vehicle";
  object.position.x = 3.0;
  object.position.y = -4.0;
  object.position.z = 1.0;
  object.velocity.x = 2.0 * kPi;
  object.acceleration.y = 0.4 * kPi * kPi;
  object.velocity_available = true;
  object.acceleration_available = true;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1.0;
  object.bbox_size.x = 4.5;
  object.bbox_size.y = 1.8;
  object.bbox_size.z = 1.5;
  objects.objects.push_back(object);
  return objects;
}
}  // namespace

TEST(PredictionCore, FollowsCounterclockwiseCircleAtAllTenHorizons) {
  auto objects = observation();
  objects.objects.front().bbox_pose.orientation.x = std::sqrt(0.5);
  objects.objects.front().bbox_pose.orientation.w = std::sqrt(0.5);
  const auto predictions = std::get<6>(pp::PredictionCore{}.process(nav_msgs::Odometry{},
      interfaces::CompetitionVehicleStatus{}, objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  ASSERT_EQ(predictions.size(), 10u);
  for (const auto& sample : predictions) {
    ASSERT_EQ(sample.objects.size(), 1u);
    const auto& object = sample.objects.front();
    EXPECT_NEAR(std::hypot(object.position.x - 3.0, object.position.y - 6.0), 10.0, 1e-10);
    EXPECT_NEAR(std::hypot(object.velocity.x, object.velocity.y), 2.0 * kPi, 1e-10);
    EXPECT_EQ(object.bbox_pose.position, object.position);
  }
  const auto& first = predictions.front().objects.front();
  EXPECT_NEAR(first.position.x, 6.090169943749474, 1e-10);
  EXPECT_NEAR(first.position.y, -3.510565162951536, 1e-10);
  const auto& quarter = predictions[4].objects.front();
  EXPECT_NEAR(quarter.position.x, 13.0, 1e-10);
  EXPECT_NEAR(quarter.position.y, 6.0, 1e-10);
  EXPECT_NEAR(quarter.velocity.x, 0.0, 1e-10);
  EXPECT_NEAR(quarter.velocity.y, 2.0 * kPi, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.x, 0.5, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.y, 0.5, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.z, 0.5, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.w, 0.5, 1e-10);
  const auto& half = predictions.back().objects.front();
  EXPECT_NEAR(half.position.x, 3.0, 1e-10);
  EXPECT_NEAR(half.position.y, 16.0, 1e-10);
  EXPECT_NEAR(half.velocity.x, -2.0 * kPi, 1e-10);
  EXPECT_NEAR(half.velocity.y, 0.0, 1e-10);
}

TEST(PredictionCore, FollowsClockwiseCircleWithObliqueVelocityAndIgnoresTangentialAcceleration) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity.x = 4.0;
  object.velocity.y = 3.0;
  // omega=-pi/5 plus an acceleration parallel to velocity.
  object.acceleration.x = 0.6 * kPi + 8.0;
  object.acceleration.y = -0.8 * kPi + 6.0;
  const auto predictions = std::get<6>(pp::PredictionCore{}.process(nav_msgs::Odometry{},
      interfaces::CompetitionVehicleStatus{}, objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  const auto& quarter = predictions[4].objects.front();
  EXPECT_NEAR(quarter.position.x, 3.0 + 35.0 / kPi, 1e-10);
  EXPECT_NEAR(quarter.position.y, -4.0 - 5.0 / kPi, 1e-10);
  EXPECT_NEAR(quarter.velocity.x, 3.0, 1e-10);
  EXPECT_NEAR(quarter.velocity.y, -4.0, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.z, -std::sqrt(0.5), 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.w, std::sqrt(0.5), 1e-10);
  const auto& half = predictions.back().objects.front();
  EXPECT_NEAR(half.position.x, 3.0 + 30.0 / kPi, 1e-10);
  EXPECT_NEAR(half.position.y, -4.0 - 40.0 / kPi, 1e-10);
}

TEST(PredictionCore, IntegratesStraightStationaryAndAlmostStraightMotion) {
  auto objects = observation();
  auto& straight = objects.objects.front();
  straight.velocity.x = 4.0;
  straight.velocity.y = 3.0;
  straight.velocity.z = 0.2;
  straight.acceleration.x = 8.0;
  straight.acceleration.y = 6.0;
  auto stationary = straight;
  stationary.velocity = geometry_msgs::Vector3{};
  stationary.acceleration = geometry_msgs::Vector3{};
  auto tiny_turn = stationary;
  tiny_turn.velocity.x = 4.0;
  tiny_turn.acceleration.y = 4e-13;
  objects.objects.push_back(stationary);
  objects.objects.push_back(tiny_turn);
  const auto predictions = std::get<6>(pp::PredictionCore{}.process(nav_msgs::Odometry{},
      interfaces::CompetitionVehicleStatus{}, objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  const auto& last = predictions.back().objects;
  EXPECT_NEAR(last[0].position.x, 23.0, 1e-10);
  EXPECT_NEAR(last[0].position.y, 11.0, 1e-10);
  EXPECT_NEAR(last[0].position.z, 2.0, 1e-10);
  EXPECT_EQ(last[0].velocity, straight.velocity);
  EXPECT_EQ(last[1], stationary);
  EXPECT_NEAR(last[2].position.x, 23.0, 1e-12);
  EXPECT_NEAR(last[2].position.y, -4.0 + 5e-12, 1e-14);
  EXPECT_NEAR(last[2].velocity.y, 2e-12, 1e-24);
}

TEST(PredictionCore, PreservesObservationHeaderAccelerationAndObjectMetadata) {
  auto objects = observation();
  const auto object = objects.objects.front();
  for (std::uint32_t i = 1; i < 12; ++i) {
    objects.objects.push_back(object);
    objects.objects.back().id += i;
  }
  const auto original = objects;
  auto predictions = std::get<6>(pp::PredictionCore{}.process(nav_msgs::Odometry{},
      interfaces::CompetitionVehicleStatus{}, objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  EXPECT_EQ(objects, original);
  for (const auto& sample : predictions) {
    EXPECT_EQ(sample.header, original.header);
    ASSERT_EQ(sample.objects.size(), original.objects.size());
    for (std::size_t i = 0; i < sample.objects.size(); ++i) {
      const auto& predicted = sample.objects[i];
      const auto& input = original.objects[i];
      EXPECT_EQ(predicted.id, input.id);
      EXPECT_EQ(predicted.label, input.label);
      EXPECT_EQ(predicted.acceleration, input.acceleration);
      EXPECT_EQ(predicted.velocity_available, input.velocity_available);
      EXPECT_EQ(predicted.acceleration_available, input.acceleration_available);
      EXPECT_EQ(predicted.bbox_size, input.bbox_size);
    }
  }
  predictions[0].objects[0].position.x = 999.0;
  EXPECT_EQ(objects, original);
  EXPECT_NE(predictions[1].objects[0].position.x, 999.0);
}

TEST(PredictionCore, KeepsAllTenListsEmptyWhenNoObjectsAreObserved) {
  auto objects = observation();
  objects.objects.clear();
  const auto predictions = std::get<6>(pp::PredictionCore{}.process(nav_msgs::Odometry{},
      interfaces::CompetitionVehicleStatus{}, objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  for (const auto& sample : predictions) EXPECT_EQ(sample, objects);
}

TEST(CorePrediction, PredictsMapMotionIndependentlyOfEgoPoseAndPassesInputsThrough) {
  auto objects = observation();
  objects.objects.front().velocity.z = 0.2;
  objects.objects.front().bbox_pose.orientation.x = std::sqrt(0.5);
  objects.objects.front().bbox_pose.orientation.w = std::sqrt(0.5);
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(123.25);
  ego.pose.pose.position.x = 100.0;
  ego.pose.pose.position.y = -200.0;
  const Eigen::Quaterniond ego_rotation = Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX());
  ego.pose.pose.orientation.x = ego_rotation.x();
  ego.pose.pose.orientation.y = ego_rotation.y();
  ego.pose.pose.orientation.z = ego_rotation.z();
  ego.pose.pose.orientation.w = ego_rotation.w();
  interfaces::CompetitionVehicleStatus competition;
  competition.gear = 4;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms.front().child_frame_id = "base_link";
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.3;
  const pp::Route route;
  const auto result = pp::CorePrediction{}.process(ego, competition, objects, tf, imu, route);
  EXPECT_EQ(std::get<0>(result), ego);
  EXPECT_EQ(std::get<1>(result), competition);
  EXPECT_EQ(std::get<2>(result), objects);
  EXPECT_EQ(std::get<3>(result), tf);
  EXPECT_EQ(std::get<4>(result), imu);
  EXPECT_EQ(std::get<5>(result), route);
  const auto& predicted = std::get<6>(result);
  for (const auto& sample : predicted) {
    EXPECT_EQ(sample.header, objects.header);
    const auto& actual = sample.objects.front();
    EXPECT_EQ(actual.bbox_pose.position, actual.position);
    EXPECT_EQ(actual.acceleration, objects.objects.front().acceleration);
    EXPECT_EQ(actual.bbox_size, objects.objects.front().bbox_size);
  }
  const auto& quarter = predicted[4].objects.front();
  EXPECT_NEAR(quarter.position.x, 13.0, 1e-10);
  EXPECT_NEAR(quarter.position.y, 6.0, 1e-10);
  EXPECT_NEAR(quarter.position.z, 1.5, 1e-10);
  EXPECT_NEAR(quarter.velocity.x, 0.0, 1e-10);
  EXPECT_NEAR(quarter.velocity.y, 2.0 * kPi, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.x, 0.5, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.y, 0.5, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.z, 0.5, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.w, 0.5, 1e-10);
}

TEST(PredictionMotionInputs, RejectsUnavailableMotionBeforePrediction) {
  auto objects = observation();
  pp::PredictionMotionInputs validator;
  for (const auto flags : {std::pair<bool, bool>{false, false}, {false, true}, {true, false}, {true, true}}) {
    objects.objects.front().velocity_available = flags.first;
    objects.objects.front().acceleration_available = flags.second;
    std::vector<node_pipeline::Kind> issues;
    validator(objects, node_pipeline::HealthLimits{}, issues);
    if (flags.first && flags.second) EXPECT_TRUE(issues.empty());
    else EXPECT_EQ(issues, std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing});
  }
}

TEST(PredictionMotionInputs, AcceptsEmptyObservations) {
  std::vector<node_pipeline::Kind> issues;
  pp::PredictionMotionInputs{}(interfaces::Objects{}, node_pipeline::HealthLimits{}, issues);
  EXPECT_TRUE(issues.empty());
}

TEST(PredictionMotionInputs, RejectsNonfiniteAvailableMotionWithoutUsingUnavailableVectors) {
  auto objects = observation();
  pp::PredictionMotionInputs validator;
  objects.objects.front().velocity.z = std::numeric_limits<double>::quiet_NaN();
  objects.objects.front().acceleration.z = std::numeric_limits<double>::infinity();
  std::vector<node_pipeline::Kind> issues;
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ(issues, (std::vector<node_pipeline::Kind>{node_pipeline::Kind::NaN, node_pipeline::Kind::Inf}));
  objects.objects.front().velocity_available = false;
  objects.objects.front().acceleration_available = false;
  issues.clear();
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ(issues, std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing});
  objects.objects.front().velocity_available = true;
  issues.clear();
  validator(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ(issues, (std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing, node_pipeline::Kind::NaN}));
}
