#pragma once
#include <path_planner/cores/roundabout_gt_prediction.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline RoundaboutGtPredictionParameters loadRoundaboutGtPredictionParameters(const node_pipeline::ParamReader& params) {
  RoundaboutGtPredictionParameters p;
  p.acceleration_persistence_sec = params.number("acceleration_persistence_sec");
  p.max_longitudinal_acceleration_mps2 = params.number("max_longitudinal_acceleration_mps2");
  p.max_turn_rate_radps = params.number("max_turn_rate_radps");
  p.turning_speed_threshold_mps = params.number("turning_speed_threshold_mps");
  for (double value : {p.acceleration_persistence_sec, p.max_longitudinal_acceleration_mps2,
                       p.max_turn_rate_radps, p.turning_speed_threshold_mps})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("roundabout GT parameters must be finite and positive");
  if (p.acceleration_persistence_sec > 2.0 || p.max_longitudinal_acceleration_mps2 > 6.0 ||
      p.max_turn_rate_radps > 1.5 || p.turning_speed_threshold_mps > 2.0)
    throw std::invalid_argument("roundabout GT parameter range is invalid");
  return p;
}
}  // namespace path_planner
