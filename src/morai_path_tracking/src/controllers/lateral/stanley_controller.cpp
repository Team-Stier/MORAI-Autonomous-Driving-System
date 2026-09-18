#include "morai_path_tracking/controllers/lateral/stanley_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace morai_path_tracking {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct IndexedPoint {
  Point2d point;
  std::size_t original_index{0U};
};

bool positiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

double normalizeAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

Point2d interpolateAtArcLength(const std::vector<IndexedPoint>& path,
                               const std::vector<double>& arc_lengths_m,
                               double arc_length_m) {
  if (arc_length_m <= 0.0) {
    return path.front().point;
  }
  if (arc_length_m >= arc_lengths_m.back()) {
    return path.back().point;
  }

  for (std::size_t index = 0U; index + 1U < path.size(); ++index) {
    const double segment_length_m =
        arc_lengths_m[index + 1U] - arc_lengths_m[index];
    if (!positiveFinite(segment_length_m) ||
        arc_length_m > arc_lengths_m[index + 1U]) {
      continue;
    }
    const double ratio =
        clamp((arc_length_m - arc_lengths_m[index]) / segment_length_m,
              0.0, 1.0);
    return {
        path[index].point.x +
            ratio * (path[index + 1U].point.x - path[index].point.x),
        path[index].point.y +
            ratio * (path[index + 1U].point.y - path[index].point.y),
    };
  }
  return path.back().point;
}

double signedCurvature(const Point2d& first, const Point2d& second,
                       const Point2d& third) {
  const double first_second_m =
      std::hypot(second.x - first.x, second.y - first.y);
  const double second_third_m =
      std::hypot(third.x - second.x, third.y - second.y);
  const double first_third_m =
      std::hypot(third.x - first.x, third.y - first.y);
  const double denominator =
      first_second_m * second_third_m * first_third_m;
  if (!positiveFinite(denominator)) {
    return 0.0;
  }
  const double twice_signed_area =
      2.0 * ((second.x - first.x) * (third.y - first.y) -
             (second.y - first.y) * (third.x - first.x));
  return twice_signed_area / denominator;
}

std::string configurationError(const StanleyConfig& config) {
  if (!positiveFinite(config.wheelbase_m)) {
    return "wheelbase_m must be finite and positive";
  }
  if (!positiveFinite(config.gain)) {
    return "gain must be finite and positive";
  }
  if (!positiveFinite(config.softening_speed_mps)) {
    return "softening_speed_mps must be finite and positive";
  }
  if (!positiveFinite(config.minimum_control_speed_mps)) {
    return "minimum_control_speed_mps must be finite and positive";
  }
  if (!positiveFinite(config.heading_window_m)) {
    return "heading_window_m must be finite and positive";
  }
  if (!std::isfinite(config.heading_error_gain) ||
      config.heading_error_gain < 0.0) {
    return "heading_error_gain must be finite and non-negative";
  }
  if (!std::isfinite(config.curvature_feedforward_gain) ||
      config.curvature_feedforward_gain < 0.0) {
    return "curvature_feedforward_gain must be finite and non-negative";
  }
  if (!positiveFinite(config.curvature_preview_distance_m)) {
    return "curvature_preview_distance_m must be finite and positive";
  }
  if (!std::isfinite(config.yaw_rate_damping_gain_sec) ||
      config.yaw_rate_damping_gain_sec < 0.0) {
    return "yaw_rate_damping_gain_sec must be finite and non-negative";
  }
  if (!std::isfinite(config.yaw_rate_damping_nonlinear_gain_sec2) ||
      config.yaw_rate_damping_nonlinear_gain_sec2 < 0.0) {
    return "yaw_rate_damping_nonlinear_gain_sec2 must be finite and "
           "non-negative";
  }
  if (!positiveFinite(config.maximum_steering_angle_rad)) {
    return "maximum_steering_angle_rad must be finite and positive";
  }
  if (!positiveFinite(config.maximum_steering_rate_rad_per_sec)) {
    return "maximum_steering_rate_rad_per_sec must be finite and positive";
  }
  return {};
}

StanleyResult invalidResult(const std::string& error) {
  StanleyResult result;
  result.error = error;
  return result;
}

}  // namespace

StanleyController::StanleyController(const StanleyConfig& config)
    : config_(config), config_error_(configurationError(config)) {}

StanleyResult StanleyController::calculate(
    const std::vector<Point2d>& path_in_vehicle_frame, double speed_mps,
    double measured_yaw_rate_radps, double previous_steering_angle_rad,
    double dt_sec) const {
  if (!config_error_.empty()) {
    return invalidResult("invalid Stanley configuration: " + config_error_);
  }
  if (!std::isfinite(speed_mps) || speed_mps < 0.0) {
    return invalidResult("speed_mps must be finite and non-negative");
  }
  if (!std::isfinite(measured_yaw_rate_radps)) {
    return invalidResult("measured yaw rate must be finite");
  }
  if (!std::isfinite(previous_steering_angle_rad)) {
    return invalidResult("previous steering must be finite");
  }
  if (!positiveFinite(dt_sec)) {
    return invalidResult("dt_sec must be finite and positive");
  }

  std::vector<IndexedPoint> finite_path;
  finite_path.reserve(path_in_vehicle_frame.size());
  for (std::size_t index = 0U; index < path_in_vehicle_frame.size();
       ++index) {
    const Point2d& point = path_in_vehicle_frame[index];
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      finite_path.push_back({point, index});
    }
  }
  if (finite_path.size() < 2U) {
    return invalidResult("path requires at least two finite points");
  }

  std::vector<double> arc_lengths_m(finite_path.size(), 0.0);
  for (std::size_t index = 1U; index < finite_path.size(); ++index) {
    arc_lengths_m[index] =
        arc_lengths_m[index - 1U] +
        std::hypot(finite_path[index].point.x -
                       finite_path[index - 1U].point.x,
                   finite_path[index].point.y -
                       finite_path[index - 1U].point.y);
  }

  const Point2d front_axle{config_.wheelbase_m, 0.0};
  double nearest_distance_m = std::numeric_limits<double>::infinity();
  Point2d target;
  double nearest_segment_heading_rad = 0.0;
  double target_arc_length_m = 0.0;
  std::size_t target_segment_index = 0U;
  bool found_segment = false;

  for (std::size_t index = 0U; index + 1U < finite_path.size(); ++index) {
    const Point2d& start = finite_path[index].point;
    const Point2d& end = finite_path[index + 1U].point;
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_length_squared =
        segment_x * segment_x + segment_y * segment_y;
    if (!positiveFinite(segment_length_squared)) {
      continue;
    }

    const double projection =
        ((front_axle.x - start.x) * segment_x +
         (front_axle.y - start.y) * segment_y) /
        segment_length_squared;
    const double clamped_projection = clamp(projection, 0.0, 1.0);
    const Point2d candidate{
        start.x + clamped_projection * segment_x,
        start.y + clamped_projection * segment_y,
    };
    const double distance_m =
        std::hypot(candidate.x - front_axle.x,
                   candidate.y - front_axle.y);
    if (std::isfinite(distance_m) && distance_m < nearest_distance_m) {
      nearest_distance_m = distance_m;
      target = candidate;
      nearest_segment_heading_rad = std::atan2(segment_y, segment_x);
      target_arc_length_m =
          arc_lengths_m[index] +
          clamped_projection * std::sqrt(segment_length_squared);
      target_segment_index = finite_path[index].original_index;
      found_segment = true;
    }
  }
  if (!found_segment) {
    return invalidResult("path requires a finite nonzero-length segment");
  }

  const double half_window_m = 0.5 * config_.heading_window_m;
  double heading_start_arc_m =
      std::max(0.0, target_arc_length_m - half_window_m);
  double heading_end_arc_m =
      std::min(arc_lengths_m.back(), target_arc_length_m + half_window_m);
  const double missing_before_m =
      std::max(0.0, half_window_m - target_arc_length_m);
  heading_end_arc_m =
      std::min(arc_lengths_m.back(), heading_end_arc_m + missing_before_m);
  const double missing_after_m =
      std::max(0.0, target_arc_length_m + half_window_m -
                        arc_lengths_m.back());
  heading_start_arc_m =
      std::max(0.0, heading_start_arc_m - missing_after_m);

  const Point2d heading_start = interpolateAtArcLength(
      finite_path, arc_lengths_m, heading_start_arc_m);
  const Point2d heading_end = interpolateAtArcLength(
      finite_path, arc_lengths_m, heading_end_arc_m);
  const double heading_dx = heading_end.x - heading_start.x;
  const double heading_dy = heading_end.y - heading_start.y;
  const double heading_chord_length_squared =
      heading_dx * heading_dx + heading_dy * heading_dy;
  const double path_heading_rad =
      positiveFinite(heading_chord_length_squared)
          ? std::atan2(heading_dy, heading_dx)
          : nearest_segment_heading_rad;

  const double left_normal_x = -std::sin(path_heading_rad);
  const double left_normal_y = std::cos(path_heading_rad);
  const double cross_track_error_m =
      left_normal_x * (target.x - front_axle.x) +
      left_normal_y * (target.y - front_axle.y);
  const double heading_error_rad = normalizeAngle(path_heading_rad);
  const double control_speed_mps =
      std::max(speed_mps, config_.minimum_control_speed_mps);

  double reference_curvature_m_inv = 0.0;
  const double remaining_path_m =
      arc_lengths_m.back() - target_arc_length_m;
  if (remaining_path_m >= config_.curvature_preview_distance_m) {
    const Point2d curvature_start = interpolateAtArcLength(
        finite_path, arc_lengths_m, target_arc_length_m);
    const Point2d curvature_middle = interpolateAtArcLength(
        finite_path, arc_lengths_m,
        target_arc_length_m + 0.5 * config_.curvature_preview_distance_m);
    const Point2d curvature_end = interpolateAtArcLength(
        finite_path, arc_lengths_m,
        target_arc_length_m + config_.curvature_preview_distance_m);
    reference_curvature_m_inv =
        signedCurvature(curvature_start, curvature_middle, curvature_end);
  }
  const double reference_yaw_rate_radps =
      speed_mps * reference_curvature_m_inv;
  const double yaw_rate_error_radps =
      measured_yaw_rate_radps - reference_yaw_rate_radps;
  const double curvature_feedforward_rad =
      config_.curvature_feedforward_gain *
      std::atan(config_.wheelbase_m * reference_curvature_m_inv);
  const double heading_feedback_rad =
      config_.heading_error_gain * heading_error_rad;
  const double cross_track_feedback_rad =
      std::atan2(config_.gain * cross_track_error_m,
                 control_speed_mps + config_.softening_speed_mps);
  const double applied_yaw_rate_damping_gain_sec =
      config_.yaw_rate_damping_gain_sec +
      config_.yaw_rate_damping_nonlinear_gain_sec2 *
          std::abs(measured_yaw_rate_radps);
  const double yaw_rate_damping_steering_rad =
      -applied_yaw_rate_damping_gain_sec * yaw_rate_error_radps;
  const double requested_steering_rad =
      curvature_feedforward_rad + heading_feedback_rad +
      cross_track_feedback_rad + yaw_rate_damping_steering_rad;
  const double saturated_steering_rad =
      clamp(requested_steering_rad, -config_.maximum_steering_angle_rad,
            config_.maximum_steering_angle_rad);
  const double maximum_change_rad =
      config_.maximum_steering_rate_rad_per_sec * dt_sec;
  const double rate_limited_steering_rad =
      clamp(saturated_steering_rad,
            previous_steering_angle_rad - maximum_change_rad,
            previous_steering_angle_rad + maximum_change_rad);
  const double steering_angle_rad =
      clamp(rate_limited_steering_rad, -config_.maximum_steering_angle_rad,
            config_.maximum_steering_angle_rad);

  if (!std::isfinite(cross_track_error_m) ||
      !std::isfinite(heading_error_rad) ||
      !std::isfinite(reference_curvature_m_inv) ||
      !std::isfinite(reference_yaw_rate_radps) ||
      !std::isfinite(yaw_rate_error_radps) ||
      !std::isfinite(applied_yaw_rate_damping_gain_sec) ||
      !std::isfinite(yaw_rate_damping_steering_rad) ||
      !std::isfinite(requested_steering_rad) ||
      !std::isfinite(steering_angle_rad)) {
    return invalidResult("Stanley calculation produced a non-finite result");
  }

  StanleyResult result;
  result.valid = true;
  result.target = target;
  result.steering_angle_rad = steering_angle_rad;
  result.cross_track_error_m = cross_track_error_m;
  result.heading_error_rad = heading_error_rad;
  result.reference_curvature_m_inv = reference_curvature_m_inv;
  result.reference_yaw_rate_radps = reference_yaw_rate_radps;
  result.yaw_rate_error_radps = yaw_rate_error_radps;
  result.curvature_feedforward_steering_rad =
      curvature_feedforward_rad;
  result.heading_feedback_steering_rad = heading_feedback_rad;
  result.cross_track_feedback_steering_rad =
      cross_track_feedback_rad;
  result.applied_yaw_rate_damping_gain_sec =
      applied_yaw_rate_damping_gain_sec;
  result.yaw_rate_damping_steering_rad =
      yaw_rate_damping_steering_rad;
  result.requested_steering_angle_rad = requested_steering_rad;
  result.target_segment_index = target_segment_index;
  return result;
}

}  // namespace morai_path_tracking
