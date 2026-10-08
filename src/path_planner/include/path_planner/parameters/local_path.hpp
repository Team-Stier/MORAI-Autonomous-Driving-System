#pragma once
#include <path_planner/cores/local_path.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline LocalPathParameters loadLocalPathParameters(const node_pipeline::ParamReader& params) {
  LocalPathParameters parameters;
  parameters.local_path_length_m = params.number("local_path_length_m");
  if (!std::isfinite(parameters.local_path_length_m) || parameters.local_path_length_m <= 0)
    throw std::invalid_argument("local_path_length_m must be finite and positive");
  return parameters;
}
}  // namespace path_planner
