#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Path.h>
#include <interfaces/TrafficLight.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace velocity_planner {
struct CurvatureVelocity60Parameters {
  double configured_target_speed_mps = 60.0 / 3.6;
  double minimum_curve_speed_mps = 0.0;
  double maximum_lateral_acceleration_mps2 = 1.5;
  double curvature_speed_reduction_gain_m = 0.0;
  double preview_distance_m = 20.0;
  double lookahead_curvature_preview_distance_m = 8.0;
  double curvature_sample_spacing_m = 2.0;
  double curve_approach_deceleration_mps2 = 3.0;
  double curvature_epsilon_m_inv = 0.001;
  double target_speed_filter_time_constant_sec = 0.0;
  double target_speed_acceleration_limit_mps2 = 1.0;
  double curve_target_speed_acceleration_limit_mps2 = 1.0;
  double target_speed_deceleration_limit_mps2 = 2.0;
};

// Adapted from Team-Stier/MOLIT2026-stier1 (MIT), d71cadc7f1bb040ae026f95d0a19292d16ca8978,
// src/morai_path_tracking/src/planning/curvature_speed_planner.cpp.
class CurvatureVelocity60 {
 public:
  explicit CurvatureVelocity60(CurvatureVelocity60Parameters parameters) : parameters_(parameters) {}

  std::tuple<interfaces::Path, interfaces::TrafficLight,
             interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const interfaces::Path& path, const interfaces::TrafficLight& light,
      const interfaces::CompetitionVehicleStatus& competition, const tf2_msgs::TFMessage& tf) {
    if (std::none_of(path.points.begin(), path.points.end(),
        [](const interfaces::PathPoint& point) { return std::isnan(point.velocity); }))
      return {path, light, competition, tf};
    const auto match = std::find_if(tf.transforms.begin(), tf.transforms.end(),
      [](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
      });
    const auto& ego = match->transform.translation;
    std::size_t closest_index = 0;
    double closest_distance_m = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path.points.size(); ++index) {
      const auto& point = path.points[index].pose.pose.position;
      const double distance_m = std::hypot(point.x - ego.x, point.y - ego.y);
      if (distance_m < closest_distance_m) {
        closest_distance_m = distance_m;
        closest_index = index;
      }
    }

    const double spacing_m = parameters_.curvature_sample_spacing_m;
    const double maximum_distance_m = std::max(parameters_.preview_distance_m,
        parameters_.lookahead_curvature_preview_distance_m) + spacing_m;
    const auto& closest = path.points[closest_index].pose.pose.position;
    std::vector<DistancePoint> samples{{closest.x, closest.y, 0.0}};
    double next_distance_m = spacing_m;
    double traversed_distance_m = 0.0;
    for (std::size_t index = closest_index; index + 1 < path.points.size() &&
         next_distance_m <= maximum_distance_m + 1e-12; ++index) {
      const auto& start = path.points[index].pose.pose.position;
      const auto& end = path.points[index + 1].pose.pose.position;
      const double segment_length_m = std::hypot(end.x - start.x, end.y - start.y);
      if (segment_length_m <= std::numeric_limits<double>::epsilon()) continue;
      while (next_distance_m <= traversed_distance_m + segment_length_m + 1e-12 &&
             next_distance_m <= maximum_distance_m + 1e-12) {
        const double ratio = clamp((next_distance_m - traversed_distance_m) / segment_length_m, 0.0, 1.0);
        samples.push_back({start.x + ratio * (end.x - start.x),
                           start.y + ratio * (end.y - start.y), next_distance_m});
        next_distance_m += spacing_m;
      }
      traversed_distance_m += segment_length_m;
    }

    double raw_target_mps = parameters_.configured_target_speed_mps;
    for (std::size_t index = 0; index + 2 < samples.size(); ++index) {
      const auto& first = samples[index];
      const auto& center = samples[index + 1];
      const auto& third = samples[index + 2];
      if (center.distance_m > parameters_.preview_distance_m + 1e-12) continue;
      const double denominator = distance(first, center) * distance(center, third) * distance(first, third);
      if (!std::isfinite(denominator) || denominator <= std::numeric_limits<double>::epsilon()) continue;
      const double curvature_m_inv = 2.0 * std::abs(
          (center.x - first.x) * (third.y - first.y) -
          (center.y - first.y) * (third.x - first.x)) / denominator;
      if (!std::isfinite(curvature_m_inv) || curvature_m_inv <= parameters_.curvature_epsilon_m_inv) continue;
      const double curve_limit_mps = std::sqrt(parameters_.maximum_lateral_acceleration_mps2 / curvature_m_inv) /
          (1.0 + parameters_.curvature_speed_reduction_gain_m * curvature_m_inv);
      const double curve_target_mps = clamp(curve_limit_mps, parameters_.minimum_curve_speed_mps,
                                            parameters_.configured_target_speed_mps);
      const double allowed_now_mps = std::sqrt(curve_target_mps * curve_target_mps +
          2.0 * parameters_.curve_approach_deceleration_mps2 * center.distance_m);
      if (allowed_now_mps + 1e-12 < raw_target_mps) raw_target_mps = allowed_now_mps;
    }

    const double stamp_sec = path.header.stamp.toSec();
    double target_mps = raw_target_mps;
    if (has_last_target_) {
      // Repeated or backward path time contributes no elapsed integration time.
      const double dt_sec = std::max(0.0, stamp_sec - last_stamp_sec_);
      if (parameters_.target_speed_filter_time_constant_sec > 0.0) {
        const double alpha = 1.0 - std::exp(-dt_sec / parameters_.target_speed_filter_time_constant_sec);
        filtered_target_mps_ += alpha * (raw_target_mps - filtered_target_mps_);
      } else {
        filtered_target_mps_ = raw_target_mps;
      }
      const double acceleration_limit_mps2 = raw_target_mps + 1e-12 < parameters_.configured_target_speed_mps
          ? parameters_.curve_target_speed_acceleration_limit_mps2 : parameters_.target_speed_acceleration_limit_mps2;
      target_mps = clamp(filtered_target_mps_,
          last_target_mps_ - parameters_.target_speed_deceleration_limit_mps2 * dt_sec,
          last_target_mps_ + acceleration_limit_mps2 * dt_sec);
    } else {
      filtered_target_mps_ = raw_target_mps;
      has_last_target_ = true;
    }
    target_mps = std::min(target_mps, parameters_.configured_target_speed_mps);
    filtered_target_mps_ = std::min(filtered_target_mps_, parameters_.configured_target_speed_mps);
    last_target_mps_ = target_mps;
    last_stamp_sec_ = std::max(last_stamp_sec_, stamp_sec);

    interfaces::Path message = path;
    for (auto& point : message.points) {
      if (std::isnan(point.velocity)) {
        point.velocity = target_mps;
        point.acceleration = 0.0;
      }
    }
    return {std::move(message), light, competition, tf};
  }

 private:
  struct DistancePoint { double x; double y; double distance_m; };
  static double distance(const DistancePoint& first, const DistancePoint& second) {
    return std::hypot(second.x - first.x, second.y - first.y);
  }
  static double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
  }
  CurvatureVelocity60Parameters parameters_;
  bool has_last_target_ = false;
  double filtered_target_mps_ = 0.0;
  double last_target_mps_ = 0.0;
  double last_stamp_sec_ = 0.0;
};
}  // namespace velocity_planner
