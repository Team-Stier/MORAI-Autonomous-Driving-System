#pragma once
#include <control/cores/safe_stop.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace control {
inline SafeStopParameters loadSafeStopParameters(const node_pipeline::ParamReader& params) {
  SafeStopParameters parameters;
  parameters.brake = params.number("brake");
  parameters.steering_limit_rad = params.number("steering_limit_deg") * 3.14159265358979323846 / 180.0;
  if (!std::isfinite(parameters.steering_limit_rad) || parameters.steering_limit_rad <= 0 ||
      parameters.steering_limit_rad > 3.14159265358979323846 / 2)
    throw std::invalid_argument("safe_stop steering limit must be in (0, 90] degrees");
  if (!std::isfinite(parameters.brake) || parameters.brake <= 0 || parameters.brake > 1)
    throw std::invalid_argument("safe_stop brake must be in (0, 1]");
  return parameters;
}
}  // namespace control
