#pragma once
#include <localization/cores/tunnel_wall.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>

namespace localization {
inline TunnelWallParameters loadTunnelWallParameters(const node_pipeline::ParamReader& params) {
  TunnelWallParameters config;
  config.origin_x_m = params.number("origin_x_m");
  config.origin_y_m = params.number("origin_y_m");
  config.axis_yaw_rad = params.number("axis_yaw_rad");
  config.left_wall_m = params.number("left_wall_m");
  config.right_wall_m = params.number("right_wall_m");
  config.wall_normal_z_ratio = params.number("wall_normal_z_ratio");
  config.yaw_offset_rad = params.number("yaw_offset_rad");
  config.acceleration_noise_mps2 = params.number("acceleration_noise_mps2");
  config.velocity_noise_mps = params.number("velocity_noise_mps");
  config.lateral_velocity_noise_mps = params.number("lateral_velocity_noise_mps");
  config.wall_noise_m = params.number("wall_noise_m");
  config.wall_fit_band_m = params.number("wall_fit_band_m");
  const auto min_points = params.integer("wall_min_points");
  config.wall_min_side_m = params.number("wall_min_side_m");
  config.wall_max_range_m = params.number("wall_max_range_m");
  config.wall_min_station_m = params.number("wall_min_station_m");
  config.wall_max_station_m = params.number("wall_max_station_m");
  config.wall_min_height_m = params.number("wall_min_height_m");
  config.wall_max_height_m = params.number("wall_max_height_m");
  for (double value : {config.origin_x_m, config.origin_y_m, config.axis_yaw_rad, config.left_wall_m, config.right_wall_m,
                       config.wall_normal_z_ratio, config.yaw_offset_rad, config.wall_min_station_m,
                       config.wall_max_station_m, config.wall_min_height_m, config.wall_max_height_m})
    if (!std::isfinite(value)) throw std::invalid_argument("tunnel wall parameters must be finite");
  for (double value : {config.acceleration_noise_mps2, config.velocity_noise_mps, config.lateral_velocity_noise_mps,
                       config.wall_noise_m, config.wall_fit_band_m, config.wall_min_side_m, config.wall_max_range_m})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid tunnel wall positive coefficient");
  if (min_points <= 0 || min_points > INT32_MAX || config.left_wall_m <= config.right_wall_m ||
      config.wall_min_height_m >= config.wall_max_height_m || config.wall_min_station_m >= config.wall_max_station_m)
    throw std::invalid_argument("invalid tunnel wall geometry");
  config.wall_min_points = static_cast<int>(min_points);
  return config;
}
}  // namespace localization
