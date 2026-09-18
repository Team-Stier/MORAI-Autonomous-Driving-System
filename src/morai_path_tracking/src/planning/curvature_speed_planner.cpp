#include "morai_path_tracking/planning/curvature_speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace morai_path_tracking {
namespace {

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

double distance(const Point2d& first, const Point2d& second) {
  return std::hypot(second.x - first.x, second.y - first.y);
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

struct DistancePoint {
  Point2d point;
  double distance_m{0.0};
};

struct CurvatureSample {
  double distance_m{0.0};
  double curvature_m_inv{0.0};
};

double circumcircleCurvature(const Point2d& first, const Point2d& second,
                             const Point2d& third) {
  const double first_segment = distance(first, second);
  const double second_segment = distance(second, third);
  const double chord = distance(first, third);
  const double denominator = first_segment * second_segment * chord;
  if (!std::isfinite(denominator) ||
      denominator <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double first_dx = second.x - first.x;
  const double first_dy = second.y - first.y;
  const double chord_dx = third.x - first.x;
  const double chord_dy = third.y - first.y;
  const double twice_area =
      std::abs(first_dx * chord_dy - first_dy * chord_dx);
  const double curvature = 2.0 * twice_area / denominator;
  return std::isfinite(curvature) ? curvature : 0.0;
}

std::vector<CurvatureSample> buildCurvatureProfile(
    const std::vector<Point2d>& path, double sample_spacing_m,
    double maximum_center_distance_m) {
  if (path.empty()) {
    return {};
  }

  std::size_t closest_index = 0U;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < path.size(); ++index) {
    const Point2d& point = path[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("path points must be finite");
    }
    const double point_distance = std::hypot(point.x, point.y);
    if (point_distance < closest_distance) {
      closest_distance = point_distance;
      closest_index = index;
    }
  }

  std::vector<DistancePoint> resampled;
  resampled.push_back({path[closest_index], 0.0});
  const double maximum_point_distance_m =
      maximum_center_distance_m + sample_spacing_m;
  double next_sample_distance_m = sample_spacing_m;
  double traversed_distance_m = 0.0;
  for (std::size_t index = closest_index;
       index + 1U < path.size() &&
       next_sample_distance_m <= maximum_point_distance_m + 1.0e-12;
       ++index) {
    const Point2d& start = path[index];
    const Point2d& end = path[index + 1U];
    const double segment_length_m = distance(start, end);
    if (segment_length_m <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    while (next_sample_distance_m <=
               traversed_distance_m + segment_length_m + 1.0e-12 &&
           next_sample_distance_m <= maximum_point_distance_m + 1.0e-12) {
      const double ratio =
          clamp((next_sample_distance_m - traversed_distance_m) /
                    segment_length_m,
                0.0, 1.0);
      resampled.push_back(
          {{start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y)},
           next_sample_distance_m});
      next_sample_distance_m += sample_spacing_m;
    }
    traversed_distance_m += segment_length_m;
  }

  std::vector<CurvatureSample> profile;
  if (resampled.size() < 3U) {
    return profile;
  }
  profile.reserve(resampled.size() - 2U);
  for (std::size_t index = 0U; index + 2U < resampled.size(); ++index) {
    profile.push_back(
        {resampled[index + 1U].distance_m,
         circumcircleCurvature(resampled[index].point,
                               resampled[index + 1U].point,
                               resampled[index + 2U].point)});
  }
  return profile;
}

}  // namespace

CurvatureSpeedPlanner::CurvatureSpeedPlanner(
    const CurvatureSpeedPlannerConfig& config)
    : config_(config) {
  requireFiniteNonNegative("configured_target_speed_mps",
                           config_.configured_target_speed_mps);
  requireFiniteNonNegative("minimum_curve_speed_mps",
                           config_.minimum_curve_speed_mps);
  if (config_.minimum_curve_speed_mps >
      config_.configured_target_speed_mps) {
    throw std::invalid_argument(
        "minimum_curve_speed_mps must not exceed configured_target_speed_mps");
  }
  requireFinitePositive("maximum_lateral_acceleration_mps2",
                        config_.maximum_lateral_acceleration_mps2);
  requireFiniteNonNegative("curvature_speed_reduction_gain_m",
                           config_.curvature_speed_reduction_gain_m);
  requireFinitePositive("preview_distance_m", config_.preview_distance_m);
  requireFinitePositive("lookahead_curvature_preview_distance_m",
                        config_.lookahead_curvature_preview_distance_m);
  requireFinitePositive("curvature_sample_spacing_m",
                        config_.curvature_sample_spacing_m);
  requireFinitePositive("curve_approach_deceleration_mps2",
                        config_.curve_approach_deceleration_mps2);
  requireFinitePositive("curvature_epsilon_m_inv",
                        config_.curvature_epsilon_m_inv);
  requireFiniteNonNegative("target_speed_filter_time_constant_sec",
                           config_.target_speed_filter_time_constant_sec);
  requireFinitePositive("target_speed_acceleration_limit_mps2",
                        config_.target_speed_acceleration_limit_mps2);
  requireFinitePositive(
      "curve_target_speed_acceleration_limit_mps2",
      config_.curve_target_speed_acceleration_limit_mps2);
  requireFinitePositive("target_speed_deceleration_limit_mps2",
                        config_.target_speed_deceleration_limit_mps2);
}

CurvatureSpeedPlan CurvatureSpeedPlanner::update(
    const std::vector<Point2d>& path_in_vehicle_frame, double dt_sec) {
  requireFinitePositive("dt_sec", dt_sec);

  CurvatureSpeedPlan result;
  result.raw_target_speed_mps = config_.configured_target_speed_mps;
  result.curvature_speed_limit_mps = config_.configured_target_speed_mps;
  const double maximum_profile_distance_m =
      std::max(config_.preview_distance_m,
               config_.lookahead_curvature_preview_distance_m);
  const std::vector<CurvatureSample> profile = buildCurvatureProfile(
      path_in_vehicle_frame, config_.curvature_sample_spacing_m,
      maximum_profile_distance_m);
  for (const CurvatureSample& sample : profile) {
    if (sample.distance_m <=
        config_.lookahead_curvature_preview_distance_m + 1.0e-12) {
      result.lookahead_curvature_m_inv =
          std::max(result.lookahead_curvature_m_inv,
                   sample.curvature_m_inv);
    }
    if (sample.distance_m > config_.preview_distance_m + 1.0e-12 ||
        sample.curvature_m_inv <= config_.curvature_epsilon_m_inv) {
      continue;
    }

    const double lateral_acceleration_limit_mps =
        std::sqrt(config_.maximum_lateral_acceleration_mps2 /
                  sample.curvature_m_inv);
    const double curvature_speed_limit_mps =
        lateral_acceleration_limit_mps /
        (1.0 + config_.curvature_speed_reduction_gain_m *
                   sample.curvature_m_inv);
    const double curve_target_mps =
        clamp(curvature_speed_limit_mps, config_.minimum_curve_speed_mps,
              config_.configured_target_speed_mps);
    const double allowed_now_mps =
        std::sqrt(curve_target_mps * curve_target_mps +
                  2.0 * config_.curve_approach_deceleration_mps2 *
                      sample.distance_m);
    const double candidate_target_mps =
        std::min(config_.configured_target_speed_mps, allowed_now_mps);
    if (candidate_target_mps + 1.0e-12 <
        result.raw_target_speed_mps) {
      result.preview_curvature_m_inv = sample.curvature_m_inv;
      result.speed_limiting_curve_distance_m = sample.distance_m;
      result.curvature_speed_limit_mps = curvature_speed_limit_mps;
      result.raw_target_speed_mps = candidate_target_mps;
    }
  }

  if (!has_last_target_) {
    filtered_target_speed_mps_ = result.raw_target_speed_mps;
    result.filtered_target_speed_mps = filtered_target_speed_mps_;
    result.target_speed_mps = filtered_target_speed_mps_;
    has_last_target_ = true;
  } else {
    if (config_.target_speed_filter_time_constant_sec > 0.0) {
      const double alpha =
          1.0 -
          std::exp(-dt_sec /
                   config_.target_speed_filter_time_constant_sec);
      filtered_target_speed_mps_ +=
          alpha * (result.raw_target_speed_mps -
                   filtered_target_speed_mps_);
    } else {
      filtered_target_speed_mps_ = result.raw_target_speed_mps;
    }
    result.filtered_target_speed_mps = filtered_target_speed_mps_;
    const double minimum_target =
        last_target_speed_mps_ -
        config_.target_speed_deceleration_limit_mps2 * dt_sec;
    const bool curve_constrains_target =
        result.raw_target_speed_mps + 1.0e-12 <
        config_.configured_target_speed_mps;
    const double acceleration_limit_mps2 =
        curve_constrains_target
            ? config_
                  .curve_target_speed_acceleration_limit_mps2
            : config_.target_speed_acceleration_limit_mps2;
    const double maximum_target =
        last_target_speed_mps_ +
        acceleration_limit_mps2 * dt_sec;
    result.target_speed_mps =
        std::max(minimum_target,
                 std::min(maximum_target,
                          result.filtered_target_speed_mps));
  }
  last_target_speed_mps_ = result.target_speed_mps;
  return result;
}

void CurvatureSpeedPlanner::reset() noexcept {
  has_last_target_ = false;
  filtered_target_speed_mps_ = 0.0;
  last_target_speed_mps_ = 0.0;
}

}  // namespace morai_path_tracking
