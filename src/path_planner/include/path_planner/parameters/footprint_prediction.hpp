#pragma once
#include <path_planner/cores/footprint_prediction.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline FootprintPredictionParameters loadFootprintPredictionParameters(const node_pipeline::ParamReader& params) {
  FootprintPredictionParameters parameters;
  parameters.history_sec = params.number("history_sec");
  if (!std::isfinite(parameters.history_sec) || parameters.history_sec <= 0.0)
    throw std::invalid_argument("invalid footprint prediction history_sec");
  return parameters;
}
}  // namespace path_planner
