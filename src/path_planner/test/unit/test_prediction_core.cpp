#include <path_planner/cores/prediction_core.hpp>
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
  objects.header.frame_id = "base_link";
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
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
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
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
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
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
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
  auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
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
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
  for (const auto& sample : predictions) EXPECT_EQ(sample, objects);
}

TEST(PredictionCore, DoesNotReadUnavailableVelocity) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity_available = false;
  object.velocity.x = std::numeric_limits<double>::quiet_NaN();
  object.velocity.y = std::numeric_limits<double>::quiet_NaN();
  object.velocity.z = std::numeric_limits<double>::quiet_NaN();
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
  for (const auto& sample : predictions) {
    const auto& predicted = sample.objects.front();
    EXPECT_EQ(predicted.position, object.position);
    EXPECT_EQ(predicted.bbox_pose, object.bbox_pose);
    EXPECT_FALSE(predicted.velocity_available);
    EXPECT_TRUE(std::isnan(predicted.velocity.x));
  }
}

TEST(PredictionCore, DoesNotReadUnavailableAcceleration) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity.x = 4.0;
  object.velocity.y = 3.0;
  object.velocity.z = 0.2;
  object.acceleration_available = false;
  object.acceleration.x = std::numeric_limits<double>::quiet_NaN();
  object.acceleration.y = std::numeric_limits<double>::quiet_NaN();
  object.acceleration.z = std::numeric_limits<double>::quiet_NaN();
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
  const auto& last = predictions.back().objects.front();
  EXPECT_NEAR(last.position.x, 23.0, 1e-10);
  EXPECT_NEAR(last.position.y, 11.0, 1e-10);
  EXPECT_NEAR(last.position.z, 2.0, 1e-10);
  EXPECT_EQ(last.velocity, object.velocity);
  EXPECT_EQ(last.bbox_pose.orientation, object.bbox_pose.orientation);
  EXPECT_FALSE(last.acceleration_available);
  EXPECT_TRUE(std::isnan(last.acceleration.x));
}

TEST(PredictionCore, RotatesMapDisplacementIntoObservationBaseAndKeepsMapVelocity) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity.x = 4.0;
  object.velocity.y = 3.0;
  object.velocity.z = 0.2;
  object.acceleration.x = 8.0;
  object.acceleration.y = 6.0;
  interfaces::CompetitionVehicleStatus status;
  status.header = objects.header;
  status.yaw_rad = kPi / 2.0;
  status.velocity_x_mps = 30.0;
  status.angular_velocity_z_radps = 2.0;
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, status, pp::Route{});
  const auto& last = predictions.back().objects.front();
  EXPECT_NEAR(last.position.x, 18.0, 1e-6);
  EXPECT_NEAR(last.position.y, -24.0, 1e-6);
  EXPECT_NEAR(last.position.z, 2.0, 1e-10);
  EXPECT_EQ(last.velocity, object.velocity);
  EXPECT_EQ(last.acceleration, object.acceleration);
  EXPECT_EQ(last.bbox_pose.position, last.position);
  EXPECT_EQ(last.bbox_pose.orientation, object.bbox_pose.orientation);
  EXPECT_EQ(predictions.back().header, objects.header);
}

TEST(PredictionCore, AppliesFullInverseStatusRollPitchYawRotation) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity.x = 2.0;
  object.velocity.y = 4.0;
  object.velocity.z = 6.0;
  object.acceleration = geometry_msgs::Vector3{};
  interfaces::CompetitionVehicleStatus status;
  status.roll_rad = kPi / 2.0;
  status.pitch_rad = kPi / 2.0;
  status.yaw_rad = kPi / 2.0;
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, status, pp::Route{});
  const auto& last = predictions.back().objects.front();
  EXPECT_NEAR(last.position.x, -27.0, 1e-5);
  EXPECT_NEAR(last.position.y, 16.0, 1e-5);
  EXPECT_NEAR(last.position.z, 11.0, 1e-5);
  EXPECT_EQ(last.velocity, object.velocity);
}

TEST(PredictionCore, MapsCounterclockwiseTurnAxisIntoPitchedObservationBase) {
  const auto objects = observation();
  interfaces::CompetitionVehicleStatus status;
  status.pitch_rad = kPi / 2.0;
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, status, pp::Route{});
  const auto& quarter = predictions[4].objects.front();
  EXPECT_NEAR(quarter.position.x, 3.0, 1e-6);
  EXPECT_NEAR(quarter.position.y, 6.0, 1e-10);
  EXPECT_NEAR(quarter.position.z, 11.0, 1e-6);
  EXPECT_NEAR(quarter.velocity.x, 0.0, 1e-10);
  EXPECT_NEAR(quarter.velocity.y, 2.0 * kPi, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.x, -std::sqrt(0.5), 1e-7);
  EXPECT_NEAR(quarter.bbox_pose.orientation.y, 0.0, 1e-7);
  EXPECT_NEAR(quarter.bbox_pose.orientation.z, 0.0, 1e-7);
  EXPECT_NEAR(quarter.bbox_pose.orientation.w, std::sqrt(0.5), 1e-10);
}

TEST(PredictionCore, RotatesClockwiseMapArcIntoYawedObservationBase) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity.x = 4.0;
  object.velocity.y = 3.0;
  object.acceleration.x = 0.6 * kPi + 8.0;
  object.acceleration.y = -0.8 * kPi + 6.0;
  interfaces::CompetitionVehicleStatus status;
  status.yaw_rad = kPi / 2.0;
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, status, pp::Route{});
  const auto& quarter = predictions[4].objects.front();
  EXPECT_NEAR(quarter.position.x, 3.0 - 5.0 / kPi, 1e-6);
  EXPECT_NEAR(quarter.position.y, -4.0 - 35.0 / kPi, 1e-6);
  EXPECT_NEAR(quarter.velocity.x, 3.0, 1e-10);
  EXPECT_NEAR(quarter.velocity.y, -4.0, 1e-10);
  EXPECT_NEAR(quarter.bbox_pose.orientation.z, -std::sqrt(0.5), 1e-10);
}

TEST(PredictionCore, UsesDefaultUtmGridYawOffsetForMapDisplacement) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity.x = 10.0;
  object.acceleration = geometry_msgs::Vector3{};
  const auto predictions = pp::PredictionCore{}.process(
      objects, interfaces::CompetitionVehicleStatus{}, pp::Route{});
  const auto& first = predictions.front().objects.front();
  EXPECT_NEAR(first.position.x, 7.998618380931231, 1e-10);
  EXPECT_NEAR(first.position.y, -3.882465827027023, 1e-10);
  EXPECT_EQ(first.velocity, object.velocity);
}

TEST(PredictionCore, ReconstructsObservationYawFromNewerStatusAndBodyAngularRate) {
  auto objects = observation();
  auto& object = objects.objects.front();
  object.velocity.x = 10.0;
  object.acceleration = geometry_msgs::Vector3{};
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromSec(123.33);
  status.yaw_rad = 0.08;
  status.angular_velocity_z_radps = 1.0;
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, status, pp::Route{});
  const auto& last = predictions.back().objects.front();
  EXPECT_NEAR(last.position.x, 53.0, 1e-6);
  EXPECT_NEAR(last.position.y, -4.0, 1e-6);
  EXPECT_NEAR(last.position.z, 1.0, 1e-10);
  EXPECT_EQ(last.velocity, object.velocity);
  EXPECT_EQ(last.bbox_pose.position, last.position);
  EXPECT_EQ(predictions.back().header, objects.header);
}

TEST(PredictionCore, RoundTripsFullStatusRpyAndBodyRotationAtObservationTime) {
  auto objects = observation();
  auto& object = objects.objects.front();
  // Observation-base velocity (2,4,6), rotated into map at t=status-0.08 sec.
  object.velocity.x = 0.039977776318792735;
  object.velocity.y = 2.4272336049268675;
  object.velocity.z = 7.078625488363848;
  object.acceleration = geometry_msgs::Vector3{};
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromSec(123.33);
  status.roll_rad = 0.3;
  status.pitch_rad = -0.2;
  status.yaw_rad = 0.4;
  status.angular_velocity_x_radps = 0.5;
  status.angular_velocity_y_radps = -0.3;
  status.angular_velocity_z_radps = 0.7;
  const auto predictions = pp::PredictionCore(pp::PredictionParameters{0.0}).process(
      objects, status, pp::Route{});
  const auto& last = predictions.back().objects.front();
  EXPECT_NEAR(last.position.x, 13.0, 1e-5);
  EXPECT_NEAR(last.position.y, 16.0, 1e-5);
  EXPECT_NEAR(last.position.z, 31.0, 1e-5);
  EXPECT_EQ(last.velocity, object.velocity);
  EXPECT_EQ(last.bbox_pose.position, last.position);
}
