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
  if (params.has("max_prediction_dt_sec")) ekf.max_prediction_dt_sec = params.number("max_prediction_dt_sec");
  for (double value : {ekf.acceleration_noise, ekf.gps_position_noise, ekf.velocity_noise})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid EKF noise parameter");
  if (!std::isfinite(ekf.yaw_offset_rad) || !std::isfinite(ekf.gps_height_m)) throw std::invalid_argument("invalid heading offset");
  if (!std::isfinite(ekf.max_prediction_dt_sec) || ekf.max_prediction_dt_sec <= 0) throw std::invalid_argument("invalid max_prediction_dt_sec");
  return ekf;
}
}  // namespace localization
