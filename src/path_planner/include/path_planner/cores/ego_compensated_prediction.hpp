#pragma once
#include <path_planner/libraries/types.hpp>
#include <cmath>
#include <tuple>

namespace path_planner {
class EgoCompensatedPrediction {
 public:
  using Result = std::tuple<PredictedObjects, Route, nav_msgs::Odometry>;

  // Uniform map-plane turn; geometry stays in the observation's fixed base_link.
  // Base_link velocity/acceleration are turned in map axes and returned in base_link axes.
  // Tangential acceleration is ignored.
  Result process(const interfaces::Objects& objects, const Route& route, const nav_msgs::Odometry& ego) const {
    PredictedObjects predictions;
    predictions.fill(objects);
    const auto& attitude = ego.pose.pose.orientation;
    const double r00 = 1.0 - 2.0 * (attitude.y * attitude.y + attitude.z * attitude.z);
    const double r01 = 2.0 * (attitude.x * attitude.y - attitude.z * attitude.w);
    const double r02 = 2.0 * (attitude.x * attitude.z + attitude.y * attitude.w);
    const double r10 = 2.0 * (attitude.x * attitude.y + attitude.z * attitude.w);
    const double r11 = 1.0 - 2.0 * (attitude.x * attitude.x + attitude.z * attitude.z);
    const double r12 = 2.0 * (attitude.y * attitude.z - attitude.x * attitude.w);
    const double r20 = 2.0 * (attitude.x * attitude.z - attitude.y * attitude.w);
    const double r21 = 2.0 * (attitude.y * attitude.z + attitude.x * attitude.w);
    const double r22 = 1.0 - 2.0 * (attitude.x * attitude.x + attitude.y * attitude.y);
    const double wx_radps = ego.twist.twist.angular.x;
    const double wy_radps = ego.twist.twist.angular.y;
    const double wz_radps = ego.twist.twist.angular.z;
    const double angular_speed_radps = std::sqrt(wx_radps * wx_radps + wy_radps * wy_radps + wz_radps * wz_radps);
    const double time_offset_sec = (objects.header.stamp - ego.header.stamp).toSec();
    const double angle_back_rad = -angular_speed_radps * time_offset_sec;
    const double sin_back = std::sin(angle_back_rad), cos_back = std::cos(angle_back_rad);
    geometry_msgs::Vector3 angular_axis_body;
    if (angular_speed_radps != 0.0) {
      angular_axis_body.x = wx_radps / angular_speed_radps;
      angular_axis_body.y = wy_radps / angular_speed_radps;
      angular_axis_body.z = wz_radps / angular_speed_radps;
    }
    // Rotates a body vector about the angular-velocity axis by the angle with the given sine/cosine.
    const auto turn_body = [&](const geometry_msgs::Vector3& body, double sin_turn, double cos_turn) {
      const auto& axis = angular_axis_body;
      const double projection = axis.x * body.x + axis.y * body.y + axis.z * body.z;
      geometry_msgs::Vector3 turned;
      turned.x = cos_turn * body.x + sin_turn * (axis.y * body.z - axis.z * body.y) +
          (1.0 - cos_turn) * projection * axis.x;
      turned.y = cos_turn * body.y + sin_turn * (axis.z * body.x - axis.x * body.z) +
          (1.0 - cos_turn) * projection * axis.y;
      turned.z = cos_turn * body.z + sin_turn * (axis.x * body.y - axis.y * body.x) +
          (1.0 - cos_turn) * projection * axis.z;
      return turned;
    };
    const auto map_to_base = [&](double x, double y, double z) {
      geometry_msgs::Vector3 ego_base;
      ego_base.x = r00 * x + r10 * y + r20 * z;
      ego_base.y = r01 * x + r11 * y + r21 * z;
      ego_base.z = r02 * x + r12 * y + r22 * z;
      // R_obs = R_ego * exp(omega_body * dt); apply its inverse to the vector.
      return turn_body(ego_base, sin_back, cos_back);
    };
    const auto base_to_map = [&](const geometry_msgs::Vector3& base) {
      // R_obs = R_ego * exp(omega_body * dt); apply it to the vector.
      const auto ego_base = turn_body(base, -sin_back, cos_back);
      geometry_msgs::Vector3 map;
      map.x = r00 * ego_base.x + r01 * ego_base.y + r02 * ego_base.z;
      map.y = r10 * ego_base.x + r11 * ego_base.y + r12 * ego_base.z;
      map.z = r20 * ego_base.x + r21 * ego_base.y + r22 * ego_base.z;
      return map;
    };
    const auto turn_axis_base = map_to_base(0.0, 0.0, 1.0);
    for (std::size_t object_index = 0; object_index < objects.objects.size(); ++object_index) {
      const auto& object = objects.objects[object_index];
      if (!object.velocity_available) continue;
      const auto velocity_map = base_to_map(object.velocity);
      const auto acceleration_map = base_to_map(object.acceleration);
      const double vx_mps = velocity_map.x;
      const double vy_mps = velocity_map.y;
      const double speed_squared_mps2 = vx_mps * vx_mps + vy_mps * vy_mps;
      const double turn_rate_radps = !object.acceleration_available || speed_squared_mps2 == 0.0 ? 0.0 :
          (vx_mps * acceleration_map.y - vy_mps * acceleration_map.x) / speed_squared_mps2;
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
            time_sec * (cosc * vx_mps + sinc * vy_mps), time_sec * velocity_map.z);
        predicted.position.x += displacement_base.x;
        predicted.position.y += displacement_base.y;
        predicted.position.z += displacement_base.z;
        predicted.velocity = map_to_base(cos_angle * vx_mps - sin_angle * vy_mps,
                                         sin_angle * vx_mps + cos_angle * vy_mps, velocity_map.z);
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
    return std::make_tuple(predictions, route, ego);
  }
};
}  // namespace path_planner
