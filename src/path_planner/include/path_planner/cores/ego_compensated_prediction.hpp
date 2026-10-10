#pragma once
#include <sensor_msgs/Imu.h>
#include <path_planner/libraries/types.hpp>
#include <cmath>
#include <tuple>

namespace path_planner {
class EgoCompensatedPrediction {
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
                            interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu, Route, PredictedObjects>;

  // Uniform map-plane turn using absolute motion; tangential acceleration is ignored.
  Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& status,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) const {
    PredictedObjects predictions;
    predictions.fill(objects);
    for (std::size_t object_index = 0; object_index < objects.objects.size(); ++object_index) {
      const auto& object = objects.objects[object_index];
      const double vx_mps = object.velocity.x;
      const double vy_mps = object.velocity.y;
      const double speed_squared_mps2 = vx_mps * vx_mps + vy_mps * vy_mps;
      const double turn_rate_radps = speed_squared_mps2 == 0.0 ? 0.0 :
          (vx_mps * object.acceleration.y - vy_mps * object.acceleration.x) / speed_squared_mps2;
      for (std::size_t i = 0; i < predictions.size(); ++i) {
        const double time_sec = 0.5 * (i + 1);
        const double angle_rad = turn_rate_radps * time_sec;
        const double sin_angle = std::sin(angle_rad);
        const double cos_angle = std::cos(angle_rad);
        const double sin_half_angle = std::sin(0.5 * angle_rad);
        const double cos_half_angle = std::cos(0.5 * angle_rad);
        // Integrals of the rotating velocity, including their zero-turn limits.
        // The half-angle expression avoids cancellation in 1-cos(angle).
        const double sinc = angle_rad == 0.0 ? 1.0 : sin_angle / angle_rad;
        const double cosc = angle_rad == 0.0 ? 0.0 :
            2.0 * sin_half_angle * sin_half_angle / angle_rad;
        auto& predicted = predictions[i].objects[object_index];
        predicted.position.x += time_sec * (sinc * vx_mps - cosc * vy_mps);
        predicted.position.y += time_sec * (cosc * vx_mps + sinc * vy_mps);
        predicted.position.z += time_sec * object.velocity.z;
        predicted.velocity.x = cos_angle * vx_mps - sin_angle * vy_mps;
        predicted.velocity.y = sin_angle * vx_mps + cos_angle * vy_mps;
        predicted.bbox_pose.position = predicted.position;
        const auto& q = object.bbox_pose.orientation;
        auto& predicted_q = predicted.bbox_pose.orientation;
        predicted_q.x = cos_half_angle * q.x - sin_half_angle * q.y;
        predicted_q.y = sin_half_angle * q.x + cos_half_angle * q.y;
        predicted_q.z = cos_half_angle * q.z + sin_half_angle * q.w;
        predicted_q.w = cos_half_angle * q.w - sin_half_angle * q.z;
      }
    }
    return std::make_tuple(ego, status, objects, tf, imu, route, predictions);
  }

};
}  // namespace path_planner
