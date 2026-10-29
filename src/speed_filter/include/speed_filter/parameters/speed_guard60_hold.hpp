#pragma once
#include <speed_filter/cores/speed_guard60_hold.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace speed_filter {
inline SpeedGuard60HoldParameters loadSpeedGuard60HoldParameters(const node_pipeline::ParamReader& params) {
  SpeedGuard60HoldParameters config;
  config.speed_limit_kph = params.number("speed_limit_kph");
  config.prediction_horizon_sec = params.number("prediction_horizon_sec");
  config.acceleration_filter_time_constant_sec = params.number("acceleration_filter_time_constant_sec");
  config.accel_taper_start_kph = params.number("accel_taper_start_kph");
  config.accel_cutoff_kph = params.number("accel_cutoff_kph");
  config.brake_start_kph = params.number("brake_start_kph");
  config.brake_full_kph = params.number("brake_full_kph");
  config.maximum_guard_brake = params.number("maximum_guard_brake");
  const auto finite = [](double value) { return std::isfinite(value); };
  if (!finite(config.speed_limit_kph) || config.speed_limit_kph <= 0 ||
      !finite(config.prediction_horizon_sec) || config.prediction_horizon_sec < 0 ||
      !finite(config.acceleration_filter_time_constant_sec) || config.acceleration_filter_time_constant_sec <= 0 ||
      !finite(config.accel_taper_start_kph) || !finite(config.accel_cutoff_kph) ||
      config.accel_taper_start_kph >= config.accel_cutoff_kph ||
      !finite(config.brake_start_kph) || !finite(config.brake_full_kph) ||
      config.brake_start_kph >= config.brake_full_kph || config.brake_full_kph > config.speed_limit_kph ||
      !finite(config.maximum_guard_brake) || config.maximum_guard_brake < 0 || config.maximum_guard_brake > 1)
    throw std::invalid_argument("invalid speed guard parameters");
  return config;
}
}  // namespace speed_filter
