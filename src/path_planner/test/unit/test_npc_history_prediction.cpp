#include <path_planner/cores/npc_history_prediction.hpp>
#include <path_planner/parameters/npc_history_prediction.hpp>
#include <ros/ros.h>
#include <ros/master.h>
#include <chrono>
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <tuple>
#include <utility>

namespace pp = path_planner;
namespace {
constexpr double kPi = 3.14159265358979323846;
const Eigen::Vector3d kCircleCenter(-102.0, 343.0, 0.7);

pp::NpcHistoryPredictionParameters circleModel() {
  pp::NpcHistoryPredictionParameters parameters;
  pp::NpcHistoryPredictionParameters::Curve curve;
  curve.model_label = "test_npc";
  curve.actor_label = "test_actor";
  curve.length_m = 0.0;
  curve.history_sec = 30.0;
  for (int i = 0; i < 512; ++i) {
    const double angle = 2.0 * kPi * i / 512;
    curve.points.emplace_back(kCircleCenter.x() + 18.0 * std::cos(angle),
                              kCircleCenter.y() + 18.0 * std::sin(angle), 37.0);
  }
  for (std::size_t i = 0; i < curve.points.size(); ++i)
    curve.length_m += (curve.points[(i + 1) % curve.points.size()] - curve.points[i]).head<2>().norm();
  parameters.curves.push_back(std::move(curve));
  return parameters;
}

interfaces::Objects observation(double stamp_sec, const Eigen::Vector3d& position,
                                const std::string& label = "unknown") {
  interfaces::Objects objects;
  objects.header.seq = 42;
  objects.header.stamp.fromSec(stamp_sec);
  objects.header.frame_id = "base_link";
  interfaces::Object object;
  object.id = 7;
  object.label = label;
  object.position.x = position.x();
  object.position.y = position.y();
  object.position.z = position.z();
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1.0;
  object.bbox_size.x = 4.5;
  object.bbox_size.y = 1.8;
  object.bbox_size.z = 1.4;
  object.velocity.x = 1e8;
  object.acceleration.y = -1e8;
  object.velocity_available = true;
  object.acceleration_available = true;
  objects.objects.push_back(object);
  return objects;
}

nav_msgs::Odometry stationaryEgo(double stamp_sec) {
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(stamp_sec);
  ego.header.frame_id = "map";
  ego.child_frame_id = "base_link";
  ego.pose.pose.orientation.w = 1.0;
  return ego;
}

nav_msgs::Odometry movingEgo(double stamp_sec) {
  auto ego = stationaryEgo(stamp_sec);
  const double duration_sec = stamp_sec - 100.0;
  const double angle = 0.4 * duration_sec;
  const Eigen::Vector3d local_displacement(
      (1.1 * std::sin(angle) - 0.3 * (1.0 - std::cos(angle))) / 0.4,
      (1.1 * (1.0 - std::cos(angle)) + 0.3 * std::sin(angle)) / 0.4,
      0.2 * duration_sec);
  const Eigen::Vector3d position = Eigen::Vector3d(5.0, -2.0, 0.0) +
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ()) * local_displacement;
  ego.pose.pose.position.x = position.x();
  ego.pose.pose.position.y = position.y();
  ego.pose.pose.position.z = position.z();
  ego.pose.pose.orientation.z = std::sin(0.5 * (0.35 + angle));
  ego.pose.pose.orientation.w = std::cos(0.5 * (0.35 + angle));
  ego.twist.twist.linear.x = 1.1;
  ego.twist.twist.linear.y = 0.3;
  ego.twist.twist.linear.z = 0.2;
  ego.twist.twist.angular.z = 0.4;
  return ego;
}

Eigen::Vector3d inBase(const nav_msgs::Odometry& ego, const Eigen::Vector3d& point_map) {
  const auto& position = ego.pose.pose.position;
  const auto& q = ego.pose.pose.orientation;
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).conjugate() *
      (point_map - Eigen::Vector3d(position.x, position.y, position.z));
}

void expectPosition(const interfaces::Object& object, const Eigen::Vector3d& expected, double tolerance_m) {
  EXPECT_NEAR(object.position.x, expected.x(), tolerance_m);
  EXPECT_NEAR(object.position.y, expected.y(), tolerance_m);
  EXPECT_NEAR(object.position.z, expected.z(), tolerance_m);
  EXPECT_EQ(object.bbox_pose.position, object.position);
}
}  // namespace

TEST(NpcHistoryPrediction, PredictsAllTenCirclePositionsFromOneSecondForBothAliases) {
  for (const std::string label : {"test_actor", "test_npc"}) {
    pp::NpcHistoryPrediction core(circleModel());
    interfaces::CompetitionVehicleStatus competition;
    competition.gear = 4;
    tf2_msgs::TFMessage tf;
    tf.transforms.resize(1);
    tf.transforms.front().header.frame_id = "map";
    tf.transforms.front().child_frame_id = "base_link";
    tf.transforms.front().transform.rotation.w = 1.0;
    sensor_msgs::Imu imu;
    imu.header.frame_id = "imu_link";
    imu.orientation.w = 1.0;
    imu.linear_acceleration.x = 0.3;
    const pp::Route route;
    for (int i = 0; i <= 20; ++i) {
      const double time_sec = 99.0 + 0.1 * i;
      const double angle = (time_sec - 100.0) * 3.0 / 18.0;
      const auto objects = observation(time_sec, kCircleCenter +
          Eigen::Vector3d(18.0 * std::cos(angle), 18.0 * std::sin(angle), 0.0), label);
      const auto ego = stationaryEgo(time_sec);
      const auto result = core.process(ego, competition, objects, tf, imu, route);
      if (i != 10 && i != 20) continue;
      EXPECT_EQ(std::get<0>(result), ego);
      EXPECT_EQ(std::get<1>(result), competition);
      EXPECT_EQ(std::get<2>(result), objects);
      EXPECT_EQ(std::get<3>(result), tf);
      EXPECT_EQ(std::get<4>(result), imu);
      EXPECT_FALSE(std::get<5>(result));
      const auto& predictions = std::get<6>(result);
      ASSERT_EQ(predictions.size(), 10u);
      for (std::size_t j = 0; j < predictions.size(); ++j) {
        const double turn_angle = 0.5 * (j + 1) * 3.0 / 18.0;
        const double angle_future = angle + turn_angle;
        ASSERT_EQ(predictions[j].objects.size(), 1u);
        EXPECT_EQ(predictions[j].header, objects.header);
        const auto& predicted = predictions[j].objects.front();
        expectPosition(predicted, kCircleCenter + Eigen::Vector3d(
            18.0 * std::cos(angle_future), 18.0 * std::sin(angle_future), 0.0), 0.015);
        EXPECT_EQ(predicted.id, objects.objects.front().id);
        EXPECT_EQ(predicted.label, label);
        EXPECT_EQ(predicted.bbox_size, objects.objects.front().bbox_size);
        EXPECT_NEAR(predicted.bbox_pose.orientation.z, std::sin(0.5 * turn_angle), 0.005);
        EXPECT_NEAR(predicted.bbox_pose.orientation.w, std::cos(0.5 * turn_angle), 0.005);
      }
    }
  }
}

TEST(NpcHistoryPrediction, UsesOnlyLastSecondForUnknownAndKnownObjectsOutsideLearnedPath) {
  pp::NpcHistoryPrediction core(circleModel());
  const interfaces::CompetitionVehicleStatus competition;
  const tf2_msgs::TFMessage tf;
  const sensor_msgs::Imu imu;
  const pp::Route route;
  for (int i = 0; i <= 20; ++i) {
    const double time_sec = 98.0 + 0.1 * i;
    const double x_m = (time_sec - 99.0) * (time_sec < 99.0 ? 20.0 : 2.0);
    auto objects = observation(time_sec, {x_m, 5.0, 0.7});
    auto second = objects.objects.front();
    second.id = 8;
    second.label = "test_actor";
    second.position.y = -5.0;
    second.bbox_pose.position = second.position;
    objects.objects.push_back(second);
    const auto original = objects;
    const auto ego = stationaryEgo(time_sec);
    const auto result = core.process(ego, competition, objects, tf, imu, route);
    EXPECT_EQ(objects, original);
    if (i != 20) continue;
    const auto& predictions = std::get<6>(result);
    for (std::size_t j = 0; j < predictions.size(); ++j) {
      ASSERT_EQ(predictions[j].objects.size(), 2u);
      EXPECT_EQ(predictions[j].header, objects.header);
      expectPosition(predictions[j].objects[0], {2.0 + (j + 1), 5.0, 0.7}, 1e-7);
      expectPosition(predictions[j].objects[1], {2.0 + (j + 1), -5.0, 0.7}, 1e-7);
    }
    const auto repeated = core.process(ego, competition, objects, tf, imu, route);
    EXPECT_EQ(std::get<6>(repeated), predictions);
  }
}

TEST(NpcHistoryPrediction, CorrectsEgoTimeWithBodyTwistAndKeepsObservationFrameForDuplicateEpoch) {
  pp::NpcHistoryPrediction core(circleModel());
  const interfaces::CompetitionVehicleStatus competition;
  const tf2_msgs::TFMessage tf;
  const sensor_msgs::Imu imu;
  const pp::Route route;
  for (int i = 0; i <= 10; ++i) {
    const double time_sec = 100.0 + 0.1 * i;
    const auto ego_at_observation = movingEgo(time_sec);
    const Eigen::Vector3d point_map(20.0 + 2.0 * (time_sec - 100.0),
        5.0 + 0.5 * (time_sec - 100.0), 0.7 + 0.1 * (time_sec - 100.0));
    auto objects = observation(time_sec, inBase(ego_at_observation, point_map));
    const double local_yaw = 0.2 - (0.35 + 0.4 * (time_sec - 100.0));
    objects.objects.front().bbox_pose.orientation.z = std::sin(0.5 * local_yaw);
    objects.objects.front().bbox_pose.orientation.w = std::cos(0.5 * local_yaw);
    const auto ego = movingEgo(time_sec + 0.1);
    const auto result = core.process(ego, competition, objects, tf, imu, route);
    if (i != 10) continue;
    const auto& predictions = std::get<6>(result);
    for (std::size_t j = 0; j < predictions.size(); ++j) {
      const double horizon_sec = 0.5 * (j + 1);
      const auto expected = inBase(ego_at_observation,
          point_map + horizon_sec * Eigen::Vector3d(2.0, 0.5, 0.1));
      ASSERT_EQ(predictions[j].objects.size(), 1u);
      EXPECT_EQ(predictions[j].header, objects.header);
      const auto& predicted = predictions[j].objects.front();
      expectPosition(predicted, expected, 1e-7);
      EXPECT_NEAR(predicted.bbox_pose.orientation.z, std::sin(0.5 * local_yaw), 1e-7);
      EXPECT_NEAR(predicted.bbox_pose.orientation.w, std::cos(0.5 * local_yaw), 1e-7);
    }
    auto revised_ego = movingEgo(time_sec + 0.15);
    revised_ego.pose.pose.position.x += 0.01;
    const auto repeated = core.process(revised_ego, competition, objects, tf, imu, route);
    EXPECT_EQ(std::get<0>(repeated), revised_ego);
    EXPECT_EQ(std::get<6>(repeated), predictions);
  }
}

TEST(NpcHistoryPrediction, ReturnsTenEmptyListsForEmptyObservation) {
  pp::NpcHistoryPrediction core(circleModel());
  auto objects = observation(100.0, {0.0, 0.0, 0.7});
  objects.objects.clear();
  const auto result = core.process(stationaryEgo(100.0), interfaces::CompetitionVehicleStatus{},
      objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{});
  ASSERT_EQ(std::get<6>(result).size(), 10u);
  for (const auto& prediction : std::get<6>(result)) EXPECT_EQ(prediction, objects);
}

TEST(NpcHistoryPrediction, LoadsDeployedTrajectoryAssetThroughItsParameterReader) {
  const std::string source = __FILE__;
  const std::string package = source.substr(0, source.find("/test/unit/"));
  const auto parameters = pp::loadNpcHistoryPredictionParameters(node_pipeline::ParamReader::fromFile(
      package + "/config/path_planner/parameters/npc_history_prediction.yaml", package));
  ASSERT_EQ(parameters.curves.size(), 774u);
  EXPECT_EQ(parameters.nominal_fit_slack_m, 0.02);
  pp::NpcHistoryPrediction core(parameters);
  auto objects = observation(100.0, {0.0, 0.0, 0.7});
  objects.objects.clear();
  const auto result = core.process(stationaryEgo(100.0), interfaces::CompetitionVehicleStatus{},
      objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{});
  for (const auto& prediction : std::get<6>(result)) EXPECT_EQ(prediction, objects);
}

TEST(NpcHistoryPrediction, DoesNotInterpolateEgoAcrossAnInactiveSectionGap) {
  auto parameters = circleModel();
  parameters.curves.front().nominal_rate_mps = parameters.curves.front().length_m * 3.0 / (36.0*kPi);
  pp::NpcHistoryPrediction core(parameters);
  for (double time_sec : {100.0, 120.0, 120.1, 120.2, 120.3, 120.4, 120.5,
                         120.6, 120.7, 120.8, 120.9, 121.0}) {
    const double angle = (time_sec-100.0)*3.0/18.0;
    const Eigen::Vector3d point = kCircleCenter +
        Eigen::Vector3d(18.0*std::cos(angle), 18.0*std::sin(angle), 0.0);
    const auto ego_at_observation = movingEgo(time_sec);
    const auto objects = observation(time_sec, inBase(ego_at_observation, point), "test_actor");
    const auto result = core.process(movingEgo(time_sec+0.1), interfaces::CompetitionVehicleStatus{},
        objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{});
    if (time_sec != 120.0 && time_sec != 121.0) continue;
    for (std::size_t i = 0; i < std::get<6>(result).size(); ++i) {
      const double future_angle = angle+0.5*(i+1)*3.0/18.0;
      const auto expected = inBase(ego_at_observation, kCircleCenter +
          Eigen::Vector3d(18.0*std::cos(future_angle), 18.0*std::sin(future_angle), 0.0));
      expectPosition(std::get<6>(result)[i].objects.front(), expected, 0.015);
    }
  }
}

TEST(NpcHistoryPrediction, PublishesAllTenObservationFrameMessagesIncludingCachedResults) {
  const std::array<std::string, 10> names{{"0_5s", "1_0s", "1_5s", "2_0s", "2_5s",
      "3_0s", "3_5s", "4_0s", "4_5s", "5_0s"}};
  ros::M_string remappings;
  for (const auto& name : names)
    remappings["/visualizer/predicted_objects_"+name] = "/npc_prediction_test/predicted_objects_"+name;
  ros::init(remappings, "npc_prediction_visualization_test", ros::init_options::AnonymousName);
  if (!ros::master::check()) GTEST_SKIP() << "publisher integration requires a ROS master";
  ros::NodeHandle node;
  std::array<interfaces::Objects, 10> received;
  std::array<unsigned, 10> counts{};
  std::array<ros::Subscriber, 10> subscribers;
  pp::NpcHistoryPrediction core(circleModel());
  interfaces::Objects objects;
  pp::PredictedObjects expected;
  for (int i = 0; i <= 10; ++i) {
    const double time = 99.0+0.1*i;
    objects = observation(time, {3.0+0.2*i, 5.0, 0.7});
    expected = std::get<6>(core.process(stationaryEgo(time), interfaces::CompetitionVehicleStatus{}, objects,
        tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  }
  const auto ego = stationaryEgo(100.0);
  for (std::size_t i = 0; i < names.size(); ++i)
    subscribers[i] = node.subscribe<interfaces::Objects>("/npc_prediction_test/predicted_objects_"+names[i], 1,
        [&, i](const interfaces::Objects::ConstPtr& message) { received[i] = *message; ++counts[i]; });
  const auto wait_for_messages = [&](unsigned count) {
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      ros::spinOnce();
      if (std::all_of(counts.begin(), counts.end(), [=](unsigned actual) { return actual >= count; })) return true;
      ros::WallDuration(0.01).sleep();
    }
    return false;
  };
  ASSERT_TRUE(wait_for_messages(1));
  const auto first_received = received;
  for (std::size_t i = 0; i < names.size(); ++i) {
    // ROS1 assigns the transport sequence; the observation stamp/frame and payload stay intact.
    auto transported = received[i];
    transported.header.seq = expected[i].header.seq;
    EXPECT_EQ(transported, expected[i]);
    EXPECT_EQ(received[i].header.stamp, objects.header.stamp);
    EXPECT_EQ(received[i].header.frame_id, objects.header.frame_id);
    EXPECT_EQ(subscribers[i].getTopic(), "/npc_prediction_test/predicted_objects_"+names[i]);
  }
  const auto cached = std::get<6>(core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
      tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  EXPECT_EQ(cached, expected);
  ASSERT_TRUE(wait_for_messages(2));
  for (std::size_t i = 0; i < names.size(); ++i) {
    auto transported = received[i];
    transported.header.seq = cached[i].header.seq;
    EXPECT_EQ(transported, cached[i]);
    EXPECT_GT(received[i].header.seq, first_received[i].header.seq);
  }
}
