#pragma once
#include <path_planner/cores/ego_compensated_prediction.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline EgoCompensatedPredictionParameters loadEgoCompensatedPredictionParameters(const node_pipeline::ParamReader& params) {
  EgoCompensatedPredictionParameters parameters;
  if (params.has("yaw_offset_rad")) parameters.yaw_offset_rad = params.number("yaw_offset_rad");
  if (!std::isfinite(parameters.yaw_offset_rad)) throw std::invalid_argument("invalid yaw_offset_rad");
  return parameters;
}
}  // namespace path_planner
