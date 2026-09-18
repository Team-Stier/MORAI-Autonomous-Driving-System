#include "morai_path_tracking/controllers/lateral/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace morai_path_tracking {
namespace {

bool isFinitePoint(const Point2d& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

void requireFiniteNonNegative(const char* name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

void requireFinitePositive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

void validateConfig(const PurePursuitConfig& config) {
  requireFinitePositive("wheelbase_m", config.wheelbase_m);
  requireFiniteNonNegative("lookahead_base_m", config.lookahead_base_m);
  requireFiniteNonNegative("lookahead_speed_gain_sec",
                           config.lookahead_speed_gain_sec);
  requireFiniteNonNegative("lookahead_curvature_gain_m",
                           config.lookahead_curvature_gain_m);
  requireFinitePositive("lookahead_min_m", config.lookahead_min_m);
  requireFinitePositive("lookahead_max_m", config.lookahead_max_m);
  if (config.lookahead_max_m < config.lookahead_min_m) {
    throw std::invalid_argument(
        "lookahead_max_m must be at least lookahead_min_m");
  }
  requireFinitePositive("minimum_target_distance_m",
                        config.minimum_target_distance_m);
  requireFinitePositive("maximum_steering_angle_rad",
                        config.maximum_steering_angle_rad);
}

bool firstForwardIntersection(const Point2d& start, const Point2d& end,
                              double radius, Point2d* target) {
  if (!isFinitePoint(start) || !isFinitePoint(end)) {
    return false;
  }

  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double a = dx * dx + dy * dy;
  if (a == 0.0) {
    return false;
  }

  const double b = 2.0 * (start.x * dx + start.y * dy);
  const double c = start.x * start.x + start.y * start.y - radius * radius;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0 || !std::isfinite(discriminant)) {
    return false;
  }

  const double root = std::sqrt(discriminant);
  const double first_t = (-b - root) / (2.0 * a);
  const double second_t = (-b + root) / (2.0 * a);
  const double roots[] = {first_t, second_t};
  for (const double t : roots) {
    if (t < 0.0 || t > 1.0 || !std::isfinite(t)) {
      continue;
    }
    Point2d candidate{start.x + t * dx, start.y + t * dy};
    if (candidate.x > 0.0 && isFinitePoint(candidate)) {
      *target = candidate;
      return true;
    }
  }
  return false;
}

}  // namespace

PurePursuitResult computePurePursuit(
    const std::vector<Point2d>& path_in_vehicle_frame,
    double longitudinal_speed_mps, double preview_curvature_m_inv,
    const PurePursuitConfig& config) {
  validateConfig(config);
  if (!std::isfinite(longitudinal_speed_mps) ||
      !std::isfinite(preview_curvature_m_inv) ||
      preview_curvature_m_inv < 0.0) {
    return {};
  }

  PurePursuitResult result;
  const double speed_lookahead_m =
      config.lookahead_base_m +
      config.lookahead_speed_gain_sec * std::abs(longitudinal_speed_mps);
  const double curvature_scale =
      1.0 + config.lookahead_curvature_gain_m * preview_curvature_m_inv;
  result.lookahead_m = std::max(
      config.lookahead_min_m,
      std::min(config.lookahead_max_m, speed_lookahead_m / curvature_scale));

  Point2d target;
  bool found_target = false;
  for (std::size_t index = 1; index < path_in_vehicle_frame.size(); ++index) {
    if (firstForwardIntersection(path_in_vehicle_frame[index - 1],
                                 path_in_vehicle_frame[index],
                                 result.lookahead_m, &target)) {
      found_target = true;
      break;
    }
  }

  if (!found_target) {
    double farthest_distance = -1.0;
    for (const Point2d& point : path_in_vehicle_frame) {
      if (!isFinitePoint(point) || point.x <= 0.0) {
        continue;
      }
      const double distance = std::hypot(point.x, point.y);
      if (distance > farthest_distance) {
        farthest_distance = distance;
        target = point;
      }
    }
    if (farthest_distance < config.minimum_target_distance_m) {
      return result;
    }
  }

  const double distance_squared = target.x * target.x + target.y * target.y;
  const double raw =
      std::atan2(2.0 * config.wheelbase_m * target.y, distance_squared);
  result.valid = true;
  result.target = target;
  result.steering_angle_rad = std::max(
      -config.maximum_steering_angle_rad,
      std::min(config.maximum_steering_angle_rad, raw));
  return result;
}

}  // namespace morai_path_tracking
