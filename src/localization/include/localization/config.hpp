#pragma once
#include <localization/types.hpp>
#include <ros/node_handle.h>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace localization {
inline EkfParameters loadEkfParameters(ros::NodeHandle& params) {
  EkfParameters ekf;
  params.param("ekf/acceleration_noise", ekf.acceleration_noise, ekf.acceleration_noise);
  params.param("ekf/gps_position_noise", ekf.gps_position_noise, ekf.gps_position_noise);
  params.param("ekf/velocity_noise", ekf.velocity_noise, ekf.velocity_noise);
  params.param("ekf/gps_height_m", ekf.gps_height_m, ekf.gps_height_m);
  params.param("ekf/yaw_offset_rad", ekf.yaw_offset_rad, ekf.yaw_offset_rad);
  for (double value : {ekf.acceleration_noise, ekf.gps_position_noise, ekf.velocity_noise})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid EKF noise parameter");
  if (!std::isfinite(ekf.yaw_offset_rad) || !std::isfinite(ekf.gps_height_m)) throw std::invalid_argument("invalid heading offset");
  return ekf;
}
}  // namespace localization
