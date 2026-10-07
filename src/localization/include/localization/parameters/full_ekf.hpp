#pragma once
#include <localization/cores/full_ekf.hpp>
#include <node_pipeline/sdk.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace localization { namespace parameters {
struct FullEkf {
  EkfParameters operator()(const node_pipeline::Parameters& parameters) const {
    EkfParameters ekf;
    ekf.acceleration_noise = parameters.get("acceleration_noise", ekf.acceleration_noise);
    ekf.gps_position_noise = parameters.get("gps_position_noise", ekf.gps_position_noise);
    ekf.velocity_noise = parameters.get("velocity_noise", ekf.velocity_noise);
    ekf.gps_height_m = parameters.get("gps_height_m", ekf.gps_height_m);
    ekf.yaw_offset_rad = parameters.get("yaw_offset_rad", ekf.yaw_offset_rad);
    for (double value : {ekf.acceleration_noise, ekf.gps_position_noise, ekf.velocity_noise})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid EKF noise parameter");
    if (!std::isfinite(ekf.yaw_offset_rad) || !std::isfinite(ekf.gps_height_m)) throw std::invalid_argument("invalid heading offset");
    return ekf;
  }
};
}}  // namespace localization::parameters
