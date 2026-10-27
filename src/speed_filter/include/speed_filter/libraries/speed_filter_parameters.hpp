#pragma once
#include <speed_filter/libraries/speed_limiter.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace speed_filter {
inline SpeedFilterParameters readSpeedFilterParameters(const node_pipeline::ParamReader& params) {
  SpeedFilterParameters config;
  config.speed_limit_kph = params.number("speed_limit_kph");
  config.stop_tolerance_sec = params.number("stop_tolerance_sec");
  config.start_duration_sec = params.number("start_duration_sec");
  config.start_accel = params.number("start_accel");
  config.stop_speed_mps = params.number("stop_speed_mps");
  if (!std::isfinite(config.speed_limit_kph) || config.speed_limit_kph < 0 ||
      !std::isfinite(config.stop_speed_mps) || config.stop_speed_mps < 0 ||
      !std::isfinite(config.stop_tolerance_sec) || config.stop_tolerance_sec <= 0 ||
      !std::isfinite(config.start_duration_sec) || config.start_duration_sec <= 0 ||
      !std::isfinite(config.start_accel) || config.start_accel <= 0 || config.start_accel > 1)
    throw std::invalid_argument("invalid speed filter parameters");
  return config;
}
}  // namespace speed_filter
