#pragma once
#include <speed_filter/types.hpp>
#include <node_pipeline/sdk.hpp>
#include <cmath>
#include <stdexcept>
namespace speed_filter { namespace parameters {
struct SpeedLimit60 {
  SpeedFilterParameters operator()(const node_pipeline::Parameters& params) const {
    SpeedFilterParameters config;
    config.speed_limit_kph = params.get("speed_limit_kph", 60.0);
    config.stop_tolerance_sec = params.get("stop_tolerance_sec", config.stop_tolerance_sec);
    config.start_duration_sec = params.get("start_duration_sec", config.start_duration_sec);
    config.start_accel = params.get("start_accel", config.start_accel);
    config.stop_speed_mps = params.get("stop_speed_mps", config.stop_speed_mps);
    if (!std::isfinite(config.speed_limit_kph) || config.speed_limit_kph < 0 ||
        !std::isfinite(config.stop_speed_mps) || config.stop_speed_mps < 0 ||
        !std::isfinite(config.stop_tolerance_sec) || config.stop_tolerance_sec <= 0 ||
        !std::isfinite(config.start_duration_sec) || config.start_duration_sec <= 0 ||
        !std::isfinite(config.start_accel) || config.start_accel <= 0 || config.start_accel > 1)
      throw std::invalid_argument("invalid speed_limit60 speed filter parameters");
    return config;
  }
};
}}
