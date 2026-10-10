#include <path_planner/cores/footprint_prediction.hpp>
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <tuple>

namespace pp = path_planner;
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kToleranceM = 1e-7;

Eigen::Quaterniond rotation(double yaw, double roll = 0.0, double pitch = 0.0) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

nav_msgs::Odometry stationaryEgo(double time_sec) {
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(time_sec);
  ego.header.frame_id = "map";
  ego.child_frame_id = "base_link";
  ego.pose.pose.orientation.w = 1.0;
  return ego;
}

nav_msgs::Odometry movingEgo(double time_sec) {
  auto ego = stationaryEgo(time_sec);
  const double elapsed = time_sec - 100.0;
  const double angle = 0.4 * elapsed;
  const Eigen::Vector3d displacement(
      (1.1 * std::sin(angle) - 0.3 * (1.0 - std::cos(angle))) / 0.4,
      (1.1 * (1.0 - std::cos(angle)) + 0.3 * std::sin(angle)) / 0.4,
      0.2 * elapsed);
  const Eigen::Vector3d position = Eigen::Vector3d(5.0, -2.0, 0.0) + rotation(0.35) * displacement;
  ego.pose.pose.position.x = position.x();
  ego.pose.pose.position.y = position.y();
  ego.pose.pose.position.z = position.z();
  const auto q = rotation(0.35 + angle);
  ego.pose.pose.orientation.z = q.z();
  ego.pose.pose.orientation.w = q.w();
  ego.twist.twist.linear.x = 1.1;
  ego.twist.twist.linear.y = 0.3;
  ego.twist.twist.linear.z = 0.2;
  ego.twist.twist.angular.z = 0.4;
  return ego;
}

interfaces::Object object(std::uint32_t id, const nav_msgs::Odometry&,
                          const Eigen::Vector3d& center_map,
                          const Eigen::Quaterniond& orientation_map = Eigen::Quaterniond::Identity()) {
  interfaces::Object result;
  result.id = id;
  result.label = id == 7 ? "first_npc" : "second_npc";
  const auto& p = center_map;
  result.position.x = p.x();
  result.position.y = p.y();
  result.position.z = p.z();
  result.bbox_pose.position = result.position;
  const auto& q = orientation_map;
  result.bbox_pose.orientation.x = q.x();
  result.bbox_pose.orientation.y = q.y();
  result.bbox_pose.orientation.z = q.z();
  result.bbox_pose.orientation.w = q.w();
  result.bbox_size.x = 4.5;
  result.bbox_size.y = 1.8;
  result.bbox_size.z = 1.6;
  result.velocity.x = 1e8;
  result.acceleration.y = -1e8;
  result.velocity_available = false;
  result.acceleration_available = false;
  return result;
}

interfaces::Objects observation(double time_sec, std::initializer_list<interfaces::Object> objects) {
  interfaces::Objects result;
  result.header.seq = 42;
  result.header.stamp.fromSec(time_sec);
  result.header.frame_id = "map";
  result.objects.assign(objects);
  return result;
}

pp::PredictedObjects predict(pp::FootprintPrediction& core, const nav_msgs::Odometry& ego,
                             const interfaces::Objects& objects) {
  return std::get<6>(core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
      tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
}

void expectMetadata(const pp::PredictedObjects& predictions, const interfaces::Objects& source) {
  ASSERT_EQ(predictions.size(), 10u);
  for (const auto& sample : predictions) {
    EXPECT_EQ(sample.header, source.header);
    ASSERT_EQ(sample.objects.size(), source.objects.size());
    for (std::size_t i = 0; i < sample.objects.size(); ++i) {
      EXPECT_EQ(sample.objects[i].id, source.objects[i].id);
      EXPECT_EQ(sample.objects[i].label, source.objects[i].label);
      EXPECT_EQ(sample.objects[i].bbox_size, source.objects[i].bbox_size);
      EXPECT_EQ(sample.objects[i].position, sample.objects[i].bbox_pose.position);
    }
  }
}

void expectPose(const interfaces::Object& actual, const nav_msgs::Odometry&,
                const Eigen::Vector3d& center_map, const Eigen::Quaterniond& orientation_map) {
  const auto& p = center_map;
  EXPECT_NEAR(actual.position.x, p.x(), kToleranceM);
  EXPECT_NEAR(actual.position.y, p.y(), kToleranceM);
  EXPECT_NEAR(actual.position.z, p.z(), kToleranceM);
  const auto& q = actual.bbox_pose.orientation;
  const Eigen::Quaterniond actual_rotation(q.w, q.x, q.y, q.z);
  EXPECT_NEAR(actual_rotation.angularDistance(orientation_map), 0.0, 1e-8);
}

void expectVelocity(const interfaces::Object& actual, const nav_msgs::Odometry&,
                    const Eigen::Vector3d& velocity_map) {
  const auto& v = velocity_map;
  EXPECT_TRUE(actual.velocity_available);
  EXPECT_NEAR(actual.velocity.x, v.x(), kToleranceM);
  EXPECT_NEAR(actual.velocity.y, v.y(), kToleranceM);
  EXPECT_NEAR(actual.velocity.z, v.z(), kToleranceM);
}
}  // namespace

TEST(FootprintPrediction, EstimatesStraightAndStationaryMotionWithoutInputVelocity) {
  pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
  interfaces::Objects objects;
  pp::PredictedObjects predictions;
  for (int i = 0; i <= 10; ++i) {
    const double elapsed = 0.1 * i;
    const auto ego = stationaryEgo(100.0 + elapsed);
    objects = observation(100.0 + elapsed,
        {object(7, ego, {10.0 + 4.0 * elapsed, -3.0 + 0.5 * elapsed, 1.0 + 0.2 * elapsed}, rotation(0.3)),
         object(8, ego, {20.0, 5.0, 0.8}, rotation(-0.2))});
    predictions = predict(core, ego, objects);
  }
  expectMetadata(predictions, objects);
  for (std::size_t k = 0; k < predictions.size(); ++k) {
    const double horizon = 0.5 * (k + 1);
    expectPose(predictions[k].objects[0], stationaryEgo(101.0),
        {14.0 + 4.0 * horizon, -2.5 + 0.5 * horizon, 1.2 + 0.2 * horizon}, rotation(0.3));
    expectVelocity(predictions[k].objects[0], stationaryEgo(101.0), {4.0, 0.5, 0.2});
    expectPose(predictions[k].objects[1], stationaryEgo(101.0), {20.0, 5.0, 0.8}, rotation(-0.2));
    expectVelocity(predictions[k].objects[1], stationaryEgo(101.0), Eigen::Vector3d::Zero());
  }
}

TEST(FootprintPrediction, FollowsAnalyticCircleAcrossWrappedYaw) {
  pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
  interfaces::Objects objects;
  pp::PredictedObjects predictions;
  for (int i = 0; i <= 10; ++i) {
    const double elapsed = 0.1 * i;
    const double angle = kPi / 2.0 - 0.18 + 0.6 * elapsed;
    const auto ego = stationaryEgo(100.0 + elapsed);
    objects = observation(100.0 + elapsed, {object(7, ego,
        {-20.0 + 12.0 * std::cos(angle), 30.0 + 12.0 * std::sin(angle), 0.8}, rotation(angle + kPi / 2.0))});
    predictions = predict(core, ego, objects);
  }
  expectMetadata(predictions, objects);
  for (std::size_t k = 0; k < predictions.size(); ++k) {
    const double angle = kPi / 2.0 - 0.18 + 0.6 * (1.0 + 0.5 * (k + 1));
    expectPose(predictions[k].objects.front(), stationaryEgo(101.0),
        {-20.0 + 12.0 * std::cos(angle), 30.0 + 12.0 * std::sin(angle), 0.8}, rotation(angle + kPi / 2.0));
    expectVelocity(predictions[k].objects.front(), stationaryEgo(101.0),
        {-7.2 * std::sin(angle), 7.2 * std::cos(angle), 0.0});
  }
}

TEST(FootprintPrediction, IgnoresEgoMotionAndStampSkewForMapObservations) {
  pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
  interfaces::Objects objects;
  pp::FootprintPrediction::Result result;
  interfaces::CompetitionVehicleStatus competition;
  competition.gear = 4;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms.front().child_frame_id = "base_link";
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.3;
  for (int i = 0; i <= 10; ++i) {
    const double elapsed = 0.1 * i;
    const double time = 100.0 + elapsed;
    objects = observation(time, {object(7, movingEgo(time),
        {20.0 + 2.0 * elapsed, 5.0 + 0.5 * elapsed, 0.8 + 0.1 * elapsed}, rotation(0.2))});
    const auto original = objects;
    const auto ego = movingEgo(time + 0.1);
    result = core.process(ego, competition, objects, tf, imu, pp::Route{});
    EXPECT_EQ(objects, original);
    EXPECT_EQ(std::get<0>(result), ego);
  }
  EXPECT_EQ(std::get<1>(result), competition);
  EXPECT_EQ(std::get<2>(result), objects);
  EXPECT_EQ(std::get<3>(result), tf);
  EXPECT_EQ(std::get<4>(result), imu);
  EXPECT_FALSE(std::get<5>(result));
  const auto& predictions = std::get<6>(result);
  expectMetadata(predictions, objects);
  for (std::size_t k = 0; k < predictions.size(); ++k) {
    const double horizon = 0.5 * (k + 1);
    expectPose(predictions[k].objects.front(), movingEgo(101.0),
        {22.0 + 2.0 * horizon, 5.5 + 0.5 * horizon, 0.9 + 0.1 * horizon}, rotation(0.2));
    expectVelocity(predictions[k].objects.front(), movingEgo(101.0), {2.0, 0.5, 0.1});
  }
}

TEST(FootprintPrediction, SeparatesNpcHistoryWhenObjectOrderChanges) {
  pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
  interfaces::Objects objects;
  pp::PredictedObjects predictions;
  for (int i = 0; i <= 10; ++i) {
    const double elapsed = 0.1 * i;
    const auto ego = stationaryEgo(100.0 + elapsed);
    const auto first = object(7, ego, {5.0 + 2.0 * elapsed, -10.0, 0.8});
    const double angle = -0.4 * elapsed;
    const auto second = object(8, ego, {30.0 + 8.0 * std::cos(angle), 8.0 * std::sin(angle), 0.8},
        rotation(angle - kPi / 2.0));
    objects = i % 2 ? observation(100.0 + elapsed, {second, first}) :
                     observation(100.0 + elapsed, {first, second});
    predictions = predict(core, ego, objects);
  }
  expectMetadata(predictions, objects);
  for (std::size_t k = 0; k < predictions.size(); ++k) {
    const double horizon = 0.5 * (k + 1);
    expectPose(predictions[k].objects[0], stationaryEgo(101.0), {7.0 + 2.0 * horizon, -10.0, 0.8}, rotation(0.0));
    const double angle = -0.4 * (1.0 + horizon);
    expectPose(predictions[k].objects[1], stationaryEgo(101.0),
        {30.0 + 8.0 * std::cos(angle), 8.0 * std::sin(angle), 0.8}, rotation(angle - kPi / 2.0));
  }
}

TEST(FootprintPrediction, DuplicateObservationEpochDoesNotDistortSpeed) {
  pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
  interfaces::Objects objects;
  pp::PredictedObjects predictions;
  for (int i = 0; i <= 2; ++i) {
    const double time = 100.0 + 0.1 * i;
    objects = observation(time, {object(7, stationaryEgo(time), {1.0 + 0.3 * i, 2.0, 0.8})});
    predictions = predict(core, stationaryEgo(time), objects);
  }
  for (int i = 0; i < 8; ++i) EXPECT_EQ(predict(core, stationaryEgo(100.2), objects), predictions);
  objects = observation(100.3, {object(7, stationaryEgo(100.3), {1.9, 2.0, 0.8})});
  predictions = predict(core, stationaryEgo(100.3), objects);
  for (std::size_t k = 0; k < predictions.size(); ++k) {
    expectPose(predictions[k].objects.front(), stationaryEgo(100.3),
        {1.9 + 1.5 * (k + 1), 2.0, 0.8}, rotation(0.0));
    expectVelocity(predictions[k].objects.front(), stationaryEgo(100.3), {3.0, 0.0, 0.0});
  }
}

TEST(FootprintPrediction, RestartsTrackAfterLongAbsenceOrTimeRewind) {
  for (double restart_time : {101.0, 100.05}) {
    pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
    predict(core, stationaryEgo(100.0), observation(100.0, {object(7, stationaryEgo(100.0), {0.0, 0.0, 0.8})}));
    predict(core, stationaryEgo(100.1), observation(100.1, {object(7, stationaryEgo(100.1), {1.0, 0.0, 0.8})}));
    predict(core, stationaryEgo(100.2), observation(100.2, {}));
    const auto restarted = observation(restart_time, {object(7, stationaryEgo(restart_time), {30.0, 5.0, 0.8})});
    const auto predictions = predict(core, stationaryEgo(restart_time), restarted);
    expectMetadata(predictions, restarted);
    for (const auto& sample : predictions) {
      const auto& predicted = sample.objects.front();
      expectPose(predicted, stationaryEgo(restart_time), {30.0, 5.0, 0.8}, rotation(0.0));
      EXPECT_FALSE(predicted.velocity_available);
      EXPECT_FALSE(predicted.acceleration_available);
      EXPECT_EQ(predicted.velocity, geometry_msgs::Vector3{});
      EXPECT_EQ(predicted.acceleration, geometry_msgs::Vector3{});
    }
    const auto next = observation(restart_time + 0.1,
        {object(7, stationaryEgo(restart_time + 0.1), {30.4, 5.0, 0.8})});
    const auto recovered = predict(core, stationaryEgo(restart_time + 0.1), next);
    expectPose(recovered.back().objects.front(), stationaryEgo(restart_time + 0.1), {50.4, 5.0, 0.8}, rotation(0.0));
  }
}

TEST(FootprintPrediction, ReturnsTenEmptyListsAfterAnObjectDisappears) {
  pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
  predict(core, stationaryEgo(100.0), observation(100.0, {object(7, stationaryEgo(100.0), {5.0, 2.0, 0.8})}));
  const auto empty = observation(100.1, {});
  const auto predictions = predict(core, stationaryEgo(100.1), empty);
  ASSERT_EQ(predictions.size(), 10u);
  for (const auto& sample : predictions) EXPECT_EQ(sample, empty);
}

TEST(FootprintPrediction, ReconstructsTiltedBboxCenterFromItsTurningBottomFootprint) {
  pp::FootprintPrediction core(pp::FootprintPredictionParameters{});
  interfaces::Objects objects;
  pp::PredictedObjects predictions;
  for (int i = 0; i <= 10; ++i) {
    const double elapsed = 0.1 * i;
    const double angle = 0.3 + 0.35 * elapsed;
    const auto q = rotation(angle + kPi / 2.0, 0.2, -0.25);
    const Eigen::Vector3d footprint_center(10.0 * std::cos(angle), 10.0 * std::sin(angle), 0.0);
    const auto ego = stationaryEgo(100.0 + elapsed);
    objects = observation(100.0 + elapsed,
        {object(7, ego, footprint_center + q * Eigen::Vector3d(0.0, 0.0, 0.8), q)});
    predictions = predict(core, ego, objects);
  }
  expectMetadata(predictions, objects);
  for (std::size_t k = 0; k < predictions.size(); ++k) {
    const double angle = 0.3 + 0.35 * (1.0 + 0.5 * (k + 1));
    const auto q = rotation(angle + kPi / 2.0, 0.2, -0.25);
    const Eigen::Vector3d footprint_center(10.0 * std::cos(angle), 10.0 * std::sin(angle), 0.0);
    expectPose(predictions[k].objects.front(), stationaryEgo(101.0),
        footprint_center + q * Eigen::Vector3d(0.0, 0.0, 0.8), q);
  }
}
