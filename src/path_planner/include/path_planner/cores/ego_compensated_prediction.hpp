#pragma once
#include <sensor_msgs/Imu.h>
#include <path_planner/libraries/types.hpp>
#include <cmath>
#include <tuple>

namespace path_planner {
struct EgoCompensatedPredictionParameters {
  double yaw_offset_rad = -0.023509;
};

class EgoCompensatedPrediction {
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
                            interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu, Route, PredictedObjects>;

  explicit EgoCompensatedPrediction(EgoCompensatedPredictionParameters parameters = {}) : parameters_(parameters) {}

  // Uniform map-plane turn; geometry stays in the observation's fixed base_link.
  // Available map vectors supply the motion; tangential acceleration is ignored.
  Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& status,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) const {
    PredictedObjects predictions;
    predictions.fill(objects);
    const double yaw_rad = status.yaw_rad + parameters_.yaw_offset_rad;
    const double sin_yaw = std::sin(yaw_rad), cos_yaw = std::cos(yaw_rad);
    const double sin_pitch = std::sin(status.pitch_rad), cos_pitch = std::cos(status.pitch_rad);
    const double sin_roll = std::sin(status.roll_rad), cos_roll = std::cos(status.roll_rad);
    const double wx_radps = status.angular_velocity_x_radps;
    const double wy_radps = status.angular_velocity_y_radps;
    const double wz_radps = status.angular_velocity_z_radps;
    const double angular_speed_radps = std::sqrt(wx_radps * wx_radps + wy_radps * wy_radps + wz_radps * wz_radps);
    const double time_offset_sec = (objects.header.stamp - status.header.stamp).toSec();
    const double angle_back_rad = -angular_speed_radps * time_offset_sec;
    const double sin_back = std::sin(angle_back_rad), cos_back = std::cos(angle_back_rad);
    geometry_msgs::Vector3 angular_axis_body;
    if (angular_speed_radps != 0.0) {
      angular_axis_body.x = wx_radps / angular_speed_radps;
      angular_axis_body.y = wy_radps / angular_speed_radps;
      angular_axis_body.z = wz_radps / angular_speed_radps;
    }
    const auto map_to_base = [&](double x, double y, double z) {
      const double yaw_x = cos_yaw * x + sin_yaw * y;
      const double yaw_y = -sin_yaw * x + cos_yaw * y;
      const double pitch_x = cos_pitch * yaw_x - sin_pitch * z;
      const double pitch_z = sin_pitch * yaw_x + cos_pitch * z;
      geometry_msgs::Vector3 base;
      base.x = pitch_x;
      base.y = cos_roll * yaw_y + sin_roll * pitch_z;
      base.z = -sin_roll * yaw_y + cos_roll * pitch_z;
      // R_obs = R_status * exp(omega_body * dt); apply its inverse to the vector.
      const auto status_base = base;
      const auto& axis = angular_axis_body;
      const double projection = axis.x * status_base.x + axis.y * status_base.y + axis.z * status_base.z;
      base.x = cos_back * status_base.x + sin_back * (axis.y * status_base.z - axis.z * status_base.y) +
          (1.0 - cos_back) * projection * axis.x;
      base.y = cos_back * status_base.y + sin_back * (axis.z * status_base.x - axis.x * status_base.z) +
          (1.0 - cos_back) * projection * axis.y;
      base.z = cos_back * status_base.z + sin_back * (axis.x * status_base.y - axis.y * status_base.x) +
          (1.0 - cos_back) * projection * axis.z;
      return base;
    };
    const auto turn_axis_base = map_to_base(0.0, 0.0, 1.0);
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
        const auto displacement_base = map_to_base(
            time_sec * (sinc * vx_mps - cosc * vy_mps),
            time_sec * (cosc * vx_mps + sinc * vy_mps), time_sec * object.velocity.z);
        predicted.position.x += displacement_base.x;
        predicted.position.y += displacement_base.y;
        predicted.position.z += displacement_base.z;
        predicted.velocity.x = cos_angle * vx_mps - sin_angle * vy_mps;
        predicted.velocity.y = sin_angle * vx_mps + cos_angle * vy_mps;
        predicted.bbox_pose.position = predicted.position;
        const auto& q = object.bbox_pose.orientation;
        auto& predicted_q = predicted.bbox_pose.orientation;
        const double turn_x = sin_half_angle * turn_axis_base.x;
        const double turn_y = sin_half_angle * turn_axis_base.y;
        const double turn_z = sin_half_angle * turn_axis_base.z;
        predicted_q.x = cos_half_angle * q.x + turn_x * q.w + turn_y * q.z - turn_z * q.y;
        predicted_q.y = cos_half_angle * q.y - turn_x * q.z + turn_y * q.w + turn_z * q.x;
        predicted_q.z = cos_half_angle * q.z + turn_x * q.y - turn_y * q.x + turn_z * q.w;
        predicted_q.w = cos_half_angle * q.w - turn_x * q.x - turn_y * q.y - turn_z * q.z;
      }
    }
    return std::make_tuple(ego, status, objects, tf, imu, route, predictions);
  }

 private:
  EgoCompensatedPredictionParameters parameters_;
};
}  // namespace path_planner
