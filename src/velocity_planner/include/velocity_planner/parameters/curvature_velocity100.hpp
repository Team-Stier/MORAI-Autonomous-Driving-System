#pragma once
#include <velocity_planner/cores/curvature_velocity100.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace velocity_planner {
inline CurvatureVelocity100Parameters loadCurvatureVelocity100Parameters(const node_pipeline::ParamReader& params) {
  CurvatureVelocity100Parameters result;
  result.configured_target_speed_mps = params.number("configured_target_speed_mps");
  result.minimum_curve_speed_mps = params.number("minimum_curve_speed_mps");
  result.maximum_lateral_acceleration_mps2 = params.number("maximum_lateral_acceleration_mps2");
  result.curvature_speed_reduction_gain_m = params.number("curvature_speed_reduction_gain_m");
  result.preview_distance_m = params.number("preview_distance_m");
  result.lookahead_curvature_preview_distance_m = params.number("lookahead_curvature_preview_distance_m");
  result.curvature_sample_spacing_m = params.number("curvature_sample_spacing_m");
  result.curve_approach_deceleration_mps2 = params.number("curve_approach_deceleration_mps2");
  result.curvature_epsilon_m_inv = params.number("curvature_epsilon_m_inv");
  result.target_speed_filter_time_constant_sec = params.number("target_speed_filter_time_constant_sec");
  result.target_speed_acceleration_limit_mps2 = params.number("target_speed_acceleration_limit_mps2");
  result.curve_target_speed_acceleration_limit_mps2 = params.number("curve_target_speed_acceleration_limit_mps2");
  result.target_speed_deceleration_limit_mps2 = params.number("target_speed_deceleration_limit_mps2");
  for (double value : {result.configured_target_speed_mps, result.minimum_curve_speed_mps,
                       result.curvature_speed_reduction_gain_m, result.target_speed_filter_time_constant_sec}) {
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("curvature_velocity100 coefficients must be finite and non-negative");
  }
  for (double value : {result.maximum_lateral_acceleration_mps2, result.preview_distance_m,
                       result.lookahead_curvature_preview_distance_m, result.curvature_sample_spacing_m,
                       result.curve_approach_deceleration_mps2, result.curvature_epsilon_m_inv,
                       result.target_speed_acceleration_limit_mps2, result.curve_target_speed_acceleration_limit_mps2,
                       result.target_speed_deceleration_limit_mps2}) {
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("curvature_velocity100 coefficients must be finite and positive");
  }
  if (result.minimum_curve_speed_mps > result.configured_target_speed_mps)
    throw std::invalid_argument("minimum_curve_speed_mps must not exceed configured_target_speed_mps");
  return result;
}
}  // namespace velocity_planner
