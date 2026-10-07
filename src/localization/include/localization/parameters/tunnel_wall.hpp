#pragma once
#include <localization/cores/tunnel_wall.hpp>
#include <node_pipeline/sdk.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace localization { namespace parameters {
struct TunnelWall {
  TunnelWallParameters operator()(const node_pipeline::Parameters& parameters) const {
    TunnelWallParameters config;
    config.origin_x_m = parameters.get("origin_x_m", config.origin_x_m);
    config.origin_y_m = parameters.get("origin_y_m", config.origin_y_m);
    config.axis_yaw_rad = parameters.get("axis_yaw_rad", config.axis_yaw_rad);
    config.left_wall_m = parameters.get("left_wall_m", config.left_wall_m);
    config.right_wall_m = parameters.get("right_wall_m", config.right_wall_m);
    config.wall_normal_z_ratio = parameters.get("wall_normal_z_ratio", config.wall_normal_z_ratio);
    config.yaw_offset_rad = parameters.get("yaw_offset_rad", config.yaw_offset_rad);
    config.acceleration_noise_mps2 = parameters.get("acceleration_noise_mps2", config.acceleration_noise_mps2);
    config.velocity_noise_mps = parameters.get("velocity_noise_mps", config.velocity_noise_mps);
    config.lateral_velocity_noise_mps = parameters.get("lateral_velocity_noise_mps", config.lateral_velocity_noise_mps);
    config.wall_noise_m = parameters.get("wall_noise_m", config.wall_noise_m);
    config.wall_fit_band_m = parameters.get("wall_fit_band_m", config.wall_fit_band_m);
    config.wall_min_points = parameters.get("wall_min_points", config.wall_min_points);
    config.wall_min_side_m = parameters.get("wall_min_side_m", config.wall_min_side_m);
    config.wall_max_range_m = parameters.get("wall_max_range_m", config.wall_max_range_m);
    config.wall_min_station_m = parameters.get("wall_min_station_m", config.wall_min_station_m);
    config.wall_max_station_m = parameters.get("wall_max_station_m", config.wall_max_station_m);
    config.wall_min_height_m = parameters.get("wall_min_height_m", config.wall_min_height_m);
    config.wall_max_height_m = parameters.get("wall_max_height_m", config.wall_max_height_m);
    if (!std::isfinite(config.origin_x_m)) throw std::invalid_argument("invalid tunnel wall origin_x_m");
    if (!std::isfinite(config.origin_y_m)) throw std::invalid_argument("invalid tunnel wall origin_y_m");
    if (!std::isfinite(config.axis_yaw_rad)) throw std::invalid_argument("invalid tunnel wall axis_yaw_rad");
    if (!std::isfinite(config.left_wall_m)) throw std::invalid_argument("invalid tunnel wall left_wall_m");
    if (!std::isfinite(config.right_wall_m)) throw std::invalid_argument("invalid tunnel wall right_wall_m");
    if (!std::isfinite(config.wall_normal_z_ratio)) throw std::invalid_argument("invalid tunnel wall wall_normal_z_ratio");
    if (!std::isfinite(config.yaw_offset_rad)) throw std::invalid_argument("invalid tunnel wall yaw_offset_rad");
    if (!std::isfinite(config.acceleration_noise_mps2)) throw std::invalid_argument("invalid tunnel wall acceleration_noise_mps2");
    if (!std::isfinite(config.velocity_noise_mps)) throw std::invalid_argument("invalid tunnel wall velocity_noise_mps");
    if (!std::isfinite(config.lateral_velocity_noise_mps)) throw std::invalid_argument("invalid tunnel wall lateral_velocity_noise_mps");
    if (!std::isfinite(config.wall_noise_m)) throw std::invalid_argument("invalid tunnel wall wall_noise_m");
    if (!std::isfinite(config.wall_fit_band_m)) throw std::invalid_argument("invalid tunnel wall wall_fit_band_m");
    if (!std::isfinite(config.wall_min_side_m)) throw std::invalid_argument("invalid tunnel wall wall_min_side_m");
    if (!std::isfinite(config.wall_max_range_m)) throw std::invalid_argument("invalid tunnel wall wall_max_range_m");
    if (!std::isfinite(config.wall_min_station_m)) throw std::invalid_argument("invalid tunnel wall wall_min_station_m");
    if (!std::isfinite(config.wall_max_station_m)) throw std::invalid_argument("invalid tunnel wall wall_max_station_m");
    if (!std::isfinite(config.wall_min_height_m)) throw std::invalid_argument("invalid tunnel wall wall_min_height_m");
    if (!std::isfinite(config.wall_max_height_m)) throw std::invalid_argument("invalid tunnel wall wall_max_height_m");
    for (double value : {config.acceleration_noise_mps2, config.velocity_noise_mps,
                         config.lateral_velocity_noise_mps, config.wall_noise_m, config.wall_fit_band_m, config.wall_min_side_m,
                         config.wall_max_range_m})
      if (value <= 0) throw std::invalid_argument("invalid tunnel wall positive coefficient");
    if (config.wall_min_points <= 0 || config.left_wall_m <= config.right_wall_m ||
        config.wall_min_height_m >= config.wall_max_height_m ||
        config.wall_min_station_m >= config.wall_max_station_m)
      throw std::invalid_argument("invalid tunnel wall geometry");
    return config;
  }
};
}}  // namespace localization::parameters
