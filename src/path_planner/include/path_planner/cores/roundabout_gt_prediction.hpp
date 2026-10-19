#pragma once
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/Imu.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

namespace path_planner {
struct RoundaboutGtPredictionParameters {
  double acceleration_persistence_sec = 1.0;
  double max_longitudinal_acceleration_mps2 = 3.0;
  double max_turn_rate_radps = 0.8;
  double turning_speed_threshold_mps = 0.5;
};

// GT absolute map velocity is used immediately, without a position-history warmup.
class RoundaboutGtPrediction {
 public:
  explicit RoundaboutGtPrediction(RoundaboutGtPredictionParameters parameters) : p_(parameters) {}
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
      interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu, Route, PredictedObjects>;
  Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) {
    PredictedObjects predictions;
    predictions.fill(objects);
    for (std::size_t index = 0; index < objects.objects.size(); ++index) {
      const auto& object = objects.objects[index];
      const auto& q = object.bbox_pose.orientation;
      const double speed = std::hypot(object.velocity.x, object.velocity.y);
      const double heading = speed > 0.0 ? std::atan2(object.velocity.y, object.velocity.x) :
          std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
      const double ux = std::cos(heading), uy = std::sin(heading);
      const double longitudinal = std::max(-p_.max_longitudinal_acceleration_mps2,
          std::min(p_.max_longitudinal_acceleration_mps2,
                   ux*object.acceleration.x + uy*object.acceleration.y));
      double curvature = 0.0;
      if (object.label == "vehicle" && speed >= p_.turning_speed_threshold_mps) {
        const double turn_rate = (ux*object.acceleration.y - uy*object.acceleration.x)/speed;
        curvature = std::max(-p_.max_turn_rate_radps, std::min(p_.max_turn_rate_radps, turn_rate))/speed;
      }
      for (std::size_t i = 0; i < predictions.size(); ++i) {
        const double time = 0.5*(i + 1);
        // Positive acceleration is a short observation, not a five-second promise.
        // Braking continues to rest and never predicts an unobserved reversal.
        const double duration = longitudinal < 0.0 ? std::min(time, speed/-longitudinal) :
            std::min(time, p_.acceleration_persistence_sec);
        const double future_speed = std::max(0.0, speed + longitudinal*duration);
        const double distance = speed*duration + longitudinal*duration*duration/2 +
                                future_speed*(time - duration);
        const double turn = curvature*distance;
        const double half_sine = std::sin(turn/2), half_cosine = std::cos(turn/2);
        const double sinc = turn == 0.0 ? 1.0 : std::sin(turn)/turn;
        const double cosc = turn == 0.0 ? 0.0 : 2*half_sine*half_sine/turn;
        auto& future = predictions[i].objects[index];
        future.position.x += distance*(sinc*ux - cosc*uy);
        future.position.y += distance*(cosc*ux + sinc*uy);
        future.position.z += time*object.velocity.z;
        future.bbox_pose.position = future.position;
        const double fx = std::cos(turn)*ux - std::sin(turn)*uy;
        const double fy = std::sin(turn)*ux + std::cos(turn)*uy;
        future.velocity.x = future_speed*fx;
        future.velocity.y = future_speed*fy;
        const double tangent = future_speed > 0.0 && (longitudinal < 0.0 ||
            time < p_.acceleration_persistence_sec) ? longitudinal : 0.0;
        future.acceleration.x = tangent*fx - curvature*future_speed*future_speed*fy;
        future.acceleration.y = tangent*fy + curvature*future_speed*future_speed*fx;
        future.acceleration.z = 0.0;
        auto& rotation = future.bbox_pose.orientation;
        rotation.x = half_cosine*q.x - half_sine*q.y;
        rotation.y = half_sine*q.x + half_cosine*q.y;
        rotation.z = half_cosine*q.z + half_sine*q.w;
        rotation.w = half_cosine*q.w - half_sine*q.z;
      }
    }
    for (std::size_t i = 0; i < predictions.size(); ++i) visualizers_[i].publish(predictions[i]);
    return std::make_tuple(ego, competition, objects, tf, imu, route, predictions);
  }
 private:
  RoundaboutGtPredictionParameters p_;
  using View = node_pipeline::Visualizer<interfaces::Objects>;
  std::array<View, 10> visualizers_{{View("predicted_objects_0_5s"), View("predicted_objects_1_0s"),
      View("predicted_objects_1_5s"), View("predicted_objects_2_0s"), View("predicted_objects_2_5s"),
      View("predicted_objects_3_0s"), View("predicted_objects_3_5s"), View("predicted_objects_4_0s"),
      View("predicted_objects_4_5s"), View("predicted_objects_5_0s")}};
};
}  // namespace path_planner
