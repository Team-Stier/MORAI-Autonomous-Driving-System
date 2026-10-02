#pragma once
#include <speed_filter/types.hpp>
#include <ros/node_handle.h>
#include <cmath>
#include <stdexcept>
#include <string>

namespace speed_filter {
inline SpeedFilterParameters loadSpeedFilterParameters(ros::NodeHandle& params,
                                                       const std::string& name,
                                                       double speed_limit_kph) {
  SpeedFilterParameters config;
  config.speed_limit_kph = speed_limit_kph;
  params.param(name + "/speed_limit_kph", config.speed_limit_kph, config.speed_limit_kph);
  params.param(name + "/stop_tolerance_sec", config.stop_tolerance_sec, config.stop_tolerance_sec);
  params.param(name + "/start_duration_sec", config.start_duration_sec, config.start_duration_sec);
  params.param(name + "/start_accel", config.start_accel, config.start_accel);
  params.param(name + "/stop_speed_mps", config.stop_speed_mps, config.stop_speed_mps);
  if (!std::isfinite(config.speed_limit_kph) || config.speed_limit_kph < 0 ||
      !std::isfinite(config.stop_speed_mps) || config.stop_speed_mps < 0 ||
      !std::isfinite(config.stop_tolerance_sec) || config.stop_tolerance_sec <= 0 ||
      !std::isfinite(config.start_duration_sec) || config.start_duration_sec <= 0 ||
      !std::isfinite(config.start_accel) || config.start_accel <= 0 || config.start_accel > 1)
    throw std::invalid_argument("invalid " + name + " speed filter parameters");
  return config;
}
}  // namespace speed_filter
