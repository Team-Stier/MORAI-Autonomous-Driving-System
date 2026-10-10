#pragma once
#include <path_planner/libraries/types.hpp>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <tuple>
#include <unordered_map>

namespace path_planner {
struct FootprintPredictionParameters {
  double history_sec = 1.0;
};

class FootprintPrediction {
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
                            interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu,
                            Route, PredictedObjects>;

  explicit FootprintPrediction(FootprintPredictionParameters parameters = {})
      : parameters_(parameters) {}

  Result process(const nav_msgs::Odometry& ego,
                 const interfaces::CompetitionVehicleStatus& competition,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) {
    const double now_sec = objects.header.stamp.toSec();
    if (now_sec < last_stamp_sec_) tracks_.clear();
    last_stamp_sec_ = now_sec;
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (now_sec - it->second.back().time_sec > 0.5) it = tracks_.erase(it);
      else ++it;
    }

    PredictedObjects predictions;
    predictions.fill(objects);
    for (std::size_t i = 0; i < objects.objects.size(); ++i) {
      const auto& input = objects.objects[i];
      auto& history = tracks_[input.id];
      const Footprint observed = footprint(input, now_sec);
      if (!history.empty() && history.back().time_sec == now_sec) history.back() = observed;
      else history.push_back(observed);
      // shortcut: keep at most 128 footprints per NPC; raise the cap for longer windows at high input rates.
      while (history.size() > 128 || (history.size() > 1 &&
             history.front().time_sec < now_sec - parameters_.history_sec - 1e-9))
        history.pop_front();
      const Motion motion = estimateMotion(history);
      const Eigen::Vector3d center_m = center(observed);
      const auto& q = input.bbox_pose.orientation;
      const Eigen::Quaterniond map_rotation(q.w, q.x, q.y, q.z);
      const Eigen::AngleAxisd current_yaw(observed.yaw_rad, Eigen::Vector3d::UnitZ());
      for (std::size_t k = 0; k < predictions.size(); ++k) {
        const double dt_sec = 0.5 * (k + 1);
        const double angle_rad = motion.turn_rate_radps * dt_sec;
        const double cosc = std::abs(angle_rad) < 1e-6 ? angle_rad / 2.0 :
            2.0 * std::pow(std::sin(angle_rad / 2.0), 2) / angle_rad;
        const auto& v = motion.body_velocity_mps;
        const Eigen::Vector3d displacement_m = dt_sec * Eigen::Vector3d(
            sinc(angle_rad) * v.x() - cosc * v.y(),
            cosc * v.x() + sinc(angle_rad) * v.y(), v.z());
        const Eigen::AngleAxisd turn(angle_rad, Eigen::Vector3d::UnitZ());
        const Eigen::Vector3d point_m = center_m + current_yaw * displacement_m;
        const Eigen::Vector3d velocity_map = current_yaw * (turn * v);
        const Eigen::Vector3d acceleration_map(
            -motion.turn_rate_radps * velocity_map.y(), motion.turn_rate_radps * velocity_map.x(), 0.0);
        const Eigen::Quaterniond rotation = (turn * map_rotation).normalized();
        auto& output = predictions[k].objects[i];
        output.position.x = point_m.x(); output.position.y = point_m.y(); output.position.z = point_m.z();
        output.bbox_pose.position = output.position;
        output.bbox_pose.orientation.x = rotation.x(); output.bbox_pose.orientation.y = rotation.y();
        output.bbox_pose.orientation.z = rotation.z(); output.bbox_pose.orientation.w = rotation.w();
        output.velocity.x = velocity_map.x(); output.velocity.y = velocity_map.y(); output.velocity.z = velocity_map.z();
        output.acceleration.x = acceleration_map.x(); output.acceleration.y = acceleration_map.y();
        output.acceleration.z = acceleration_map.z();
        output.velocity_available = history.size() > 1;
        output.acceleration_available = history.size() > 1;
      }
    }
    return std::make_tuple(ego, competition, objects, tf, imu, route, predictions);
  }

 private:
  struct Footprint {
    double time_sec;
    std::array<Eigen::Vector3d, 4> corners_m;
    Eigen::Vector3d center_offset_m;
    double yaw_rad;
  };
  struct Motion {
    Eigen::Vector3d body_velocity_mps = Eigen::Vector3d::Zero();
    double turn_rate_radps = 0.0;
  };

  static double sinc(double angle_rad) {
    return std::abs(angle_rad) < 1e-6 ? 1.0 - angle_rad * angle_rad / 6.0 :
           std::sin(angle_rad) / angle_rad;
  }

  static Footprint footprint(const interfaces::Object& object, double time_sec) {
    const auto& p = object.bbox_pose.position;
    const auto& q = object.bbox_pose.orientation;
    const Eigen::Quaterniond rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
    const Eigen::Vector3d position_m(p.x, p.y, p.z);
    const Eigen::Vector3d half = 0.5 * Eigen::Vector3d(object.bbox_size.x, object.bbox_size.y, object.bbox_size.z);
    Footprint result;
    result.time_sec = time_sec;
    result.corners_m = {{position_m + rotation * Eigen::Vector3d(-half.x(), -half.y(), -half.z()),
                        position_m + rotation * Eigen::Vector3d( half.x(), -half.y(), -half.z()),
                        position_m + rotation * Eigen::Vector3d( half.x(),  half.y(), -half.z()),
                        position_m + rotation * Eigen::Vector3d(-half.x(),  half.y(), -half.z())}};
    result.center_offset_m = rotation * Eigen::Vector3d(0.0, 0.0, half.z());
    const Eigen::Vector3d forward = rotation * Eigen::Vector3d::UnitX();
    result.yaw_rad = std::atan2(forward.y(), forward.x());
    return result;
  }

  static Eigen::Vector3d center(const Footprint& footprint) {
    return 0.25 * (footprint.corners_m[0] + footprint.corners_m[1] +
                   footprint.corners_m[2] + footprint.corners_m[3]) + footprint.center_offset_m;
  }

  static Motion estimateMotion(const std::deque<Footprint>& history) {
    Motion result;
    if (history.size() < 2) return result;
    for (std::size_t i = 1; i < history.size(); ++i) {
      const auto& before = history[i - 1];
      const auto& after = history[i];
      const double delta_yaw_rad = std::atan2(std::sin(after.yaw_rad - before.yaw_rad),
                                            std::cos(after.yaw_rad - before.yaw_rad));
      const double middle_yaw_rad = before.yaw_rad + 0.5 * delta_yaw_rad;
      const double c = std::cos(middle_yaw_rad), s = std::sin(middle_yaw_rad);
      const Eigen::Vector3d delta_m = center(after) - center(before);
      result.body_velocity_mps.x() += (c * delta_m.x() + s * delta_m.y()) / sinc(0.5 * delta_yaw_rad);
      result.body_velocity_mps.y() += (-s * delta_m.x() + c * delta_m.y()) / sinc(0.5 * delta_yaw_rad);
      result.body_velocity_mps.z() += delta_m.z();
      result.turn_rate_radps += delta_yaw_rad;
    }
    const double span_sec = history.back().time_sec - history.front().time_sec;
    result.body_velocity_mps /= span_sec;
    result.turn_rate_radps /= span_sec;
    // shortcut: extrapolate recent constant body twist; use a varying-curvature model for transition curves.
    return result;
  }

  FootprintPredictionParameters parameters_;
  std::unordered_map<std::uint32_t, std::deque<Footprint>> tracks_;
  double last_stamp_sec_ = -std::numeric_limits<double>::infinity();
};
}  // namespace path_planner
