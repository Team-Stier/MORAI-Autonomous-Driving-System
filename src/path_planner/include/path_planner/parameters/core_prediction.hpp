#pragma once
#include <path_planner/cores/core_prediction.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline PredictionParameters loadCorePredictionParameters(const node_pipeline::ParamReader& params) {
  PredictionParameters parameters;
  if (params.has("step_sec")) parameters.step_sec = params.number("step_sec");
  if (parameters.step_sec != 0.5)
    throw std::invalid_argument("invalid prediction parameters");
  return parameters;
}
}  // namespace path_planner
