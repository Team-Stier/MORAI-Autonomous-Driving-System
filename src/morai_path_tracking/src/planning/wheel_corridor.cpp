#include "morai_path_tracking/planning/wheel_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace morai_path_tracking {
namespace {

bool positiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

struct Projection {
  bool valid{false};
  double signed_offset_m{0.0};
  double distance_m{0.0};
};

Projection projectToPath(const Point2d& query,
                         const std::vector<Point2d>& path) {
  Projection best;
  best.distance_m = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index + 1U < path.size(); ++index) {
    const Point2d& start = path[index];
    const Point2d& end = path[index + 1U];
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(end.x) || !std::isfinite(end.y)) {
      continue;
    }
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (!positiveFinite(length_squared)) {
      continue;
    }
    const double ratio = clamp(((query.x - start.x) * dx +
                                (query.y - start.y) * dy) /
                                   length_squared,
                               0.0, 1.0);
    const double projected_x = start.x + ratio * dx;
    const double projected_y = start.y + ratio * dy;
    const double offset_x = query.x - projected_x;
    const double offset_y = query.y - projected_y;
    const double distance_m = std::hypot(offset_x, offset_y);
    if (distance_m < best.distance_m) {
      const double segment_length_m = std::sqrt(length_squared);
      best.valid = true;
      best.distance_m = distance_m;
      best.signed_offset_m =
          (dx * offset_y - dy * offset_x) / segment_length_m;
    }
  }
  return best;
}

WheelCorridorResult invalidResult(const std::string& error) {
  WheelCorridorResult result;
  result.error = error;
  return result;
}

}  // namespace

WheelCorridorResult estimateWheelCorridor(
    const std::vector<Point2d>& path_in_vehicle_frame,
    const WheelCorridorConfig& config) {
  if (!positiveFinite(config.lane_half_width_m)) {
    return invalidResult("lane_half_width_m must be finite and positive");
  }
  if (!positiveFinite(config.vehicle_width_m)) {
    return invalidResult("vehicle_width_m must be finite and positive");
  }
  if (!positiveFinite(config.wheelbase_m)) {
    return invalidResult("wheelbase_m must be finite and positive");
  }

  const double half_width_m = 0.5 * config.vehicle_width_m;
  const std::array<Point2d, 4U> wheel_points{{
      {0.0, half_width_m},
      {0.0, -half_width_m},
      {config.wheelbase_m, half_width_m},
      {config.wheelbase_m, -half_width_m},
  }};

  WheelCorridorResult result;
  for (std::size_t index = 0U; index < wheel_points.size(); ++index) {
    const Projection projection =
        projectToPath(wheel_points[index], path_in_vehicle_frame);
    if (!projection.valid || !std::isfinite(projection.signed_offset_m)) {
      return invalidResult(
          "path requires at least one finite nonzero-length segment");
    }
    result.signed_wheel_offsets_m[index] = projection.signed_offset_m;
    result.maximum_wheel_offset_m =
        std::max(result.maximum_wheel_offset_m,
                 std::abs(projection.signed_offset_m));
  }
  result.minimum_clearance_m =
      config.lane_half_width_m - result.maximum_wheel_offset_m;
  result.valid = std::isfinite(result.minimum_clearance_m);
  if (!result.valid) {
    result.error = "wheel corridor produced a non-finite result";
  }
  return result;
}

LaneClearanceSpeedLimit computeLaneClearanceSpeedLimit(
    double minimum_wheel_clearance_m, double configured_target_speed_mps,
    const LaneClearanceSpeedConfig& config) {
  LaneClearanceSpeedLimit result;
  if (!std::isfinite(minimum_wheel_clearance_m)) {
    result.error = "minimum_wheel_clearance_m must be finite";
    return result;
  }
  if (!positiveFinite(config.recovery_start_m) ||
      !std::isfinite(config.recovery_full_m) ||
      config.recovery_full_m < 0.0 ||
      config.recovery_full_m >= config.recovery_start_m) {
    result.error =
        "clearance recovery thresholds must be finite, non-negative, and "
        "recovery_full_m must be less than recovery_start_m";
    return result;
  }
  if (!std::isfinite(configured_target_speed_mps) ||
      configured_target_speed_mps < 0.0 ||
      !std::isfinite(config.minimum_speed_mps) ||
      config.minimum_speed_mps < 0.0 ||
      config.minimum_speed_mps > configured_target_speed_mps) {
    result.error =
        "lane speeds must be finite, non-negative, and minimum speed must "
        "not exceed configured target";
    return result;
  }

  const double normalized_urgency =
      clamp((config.recovery_start_m - minimum_wheel_clearance_m) /
                (config.recovery_start_m - config.recovery_full_m),
            0.0, 1.0);
  result.urgency = normalized_urgency * normalized_urgency *
                   (3.0 - 2.0 * normalized_urgency);
  result.speed_limit_mps =
      configured_target_speed_mps -
      result.urgency *
          (configured_target_speed_mps - config.minimum_speed_mps);
  result.valid = std::isfinite(result.speed_limit_mps);
  if (!result.valid) {
    result.error = "lane clearance speed limit produced a non-finite result";
  }
  return result;
}

HeadingErrorSpeedLimit computeHeadingErrorSpeedLimit(
    double heading_error_rad, double current_target_speed_mps,
    const HeadingErrorSpeedConfig& config) {
  HeadingErrorSpeedLimit result;
  if (!std::isfinite(heading_error_rad)) {
    result.error = "heading_error_rad must be finite";
    return result;
  }
  if (!positiveFinite(config.recovery_start_rad) ||
      !positiveFinite(config.recovery_full_rad) ||
      config.recovery_full_rad <= config.recovery_start_rad) {
    result.error =
        "heading recovery thresholds must be finite, positive, and "
        "recovery_full_rad must exceed recovery_start_rad";
    return result;
  }
  if (!std::isfinite(current_target_speed_mps) ||
      current_target_speed_mps < 0.0 ||
      !std::isfinite(config.minimum_speed_mps) ||
      config.minimum_speed_mps < 0.0 ||
      config.minimum_speed_mps > current_target_speed_mps) {
    result.error =
        "heading-error speeds must be finite, non-negative, and minimum "
        "speed must not exceed the current target";
    return result;
  }

  const double normalized_urgency =
      clamp((std::abs(heading_error_rad) - config.recovery_start_rad) /
                (config.recovery_full_rad - config.recovery_start_rad),
            0.0, 1.0);
  result.urgency = normalized_urgency * normalized_urgency *
                   (3.0 - 2.0 * normalized_urgency);
  result.speed_limit_mps =
      current_target_speed_mps -
      result.urgency *
          (current_target_speed_mps - config.minimum_speed_mps);
  result.valid = std::isfinite(result.speed_limit_mps);
  if (!result.valid) {
    result.error = "heading-error speed limit produced a non-finite result";
  }
  return result;
}

}  // namespace morai_path_tracking
