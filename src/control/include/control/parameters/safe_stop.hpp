#pragma once
#include <control/cores/safe_stop.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace control {
inline SafeStopParameters loadSafeStopParameters(const node_pipeline::ParamReader& params) {
  SafeStopParameters parameters;
  parameters.brake = params.number("brake");
  if (!std::isfinite(parameters.brake) || parameters.brake <= 0 || parameters.brake > 1)
    throw std::invalid_argument("safe_stop brake must be in (0, 1]");
  return parameters;
}
}  // namespace control
