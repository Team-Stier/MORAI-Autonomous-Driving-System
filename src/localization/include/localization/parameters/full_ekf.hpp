#pragma once
#include <localization/cores/full_ekf.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace localization {
inline EkfParameters loadFullEkfParameters(const node_pipeline::ParamReader& params) {
  EkfParameters ekf;
  ekf.acceleration_noise = params.number("acceleration_noise");
  ekf.gps_position_noise = params.number("gps_position_noise");
  ekf.velocity_noise = params.number("velocity_noise");
  ekf.gps_height_m = params.number("gps_height_m");
  ekf.yaw_offset_rad = params.number("yaw_offset_rad");
  for (double value : {ekf.acceleration_noise, ekf.gps_position_noise, ekf.velocity_noise})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid EKF noise parameter");
  if (!std::isfinite(ekf.yaw_offset_rad) || !std::isfinite(ekf.gps_height_m)) throw std::invalid_argument("invalid heading offset");
  return ekf;
}
}  // namespace localization
