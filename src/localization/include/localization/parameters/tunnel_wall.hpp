#pragma once
#include <localization/cores/tunnel_wall.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace localization {
inline TunnelWallParameters loadTunnelWallParameters(const node_pipeline::ParamReader& parameters) {
  TunnelWallParameters config;
  if (parameters.has("origin_x_m")) config.origin_x_m = parameters.number("origin_x_m");
  if (parameters.has("origin_y_m")) config.origin_y_m = parameters.number("origin_y_m");
  if (parameters.has("axis_yaw_rad")) config.axis_yaw_rad = parameters.number("axis_yaw_rad");
  if (parameters.has("left_wall_m")) config.left_wall_m = parameters.number("left_wall_m");
  if (parameters.has("right_wall_m")) config.right_wall_m = parameters.number("right_wall_m");
  if (parameters.has("wall_normal_z_ratio")) config.wall_normal_z_ratio = parameters.number("wall_normal_z_ratio");
  if (parameters.has("yaw_offset_rad")) config.yaw_offset_rad = parameters.number("yaw_offset_rad");
  if (parameters.has("acceleration_noise_mps2")) config.acceleration_noise_mps2 = parameters.number("acceleration_noise_mps2");
  if (parameters.has("velocity_noise_mps")) config.velocity_noise_mps = parameters.number("velocity_noise_mps");
  if (parameters.has("lateral_velocity_noise_mps")) config.lateral_velocity_noise_mps = parameters.number("lateral_velocity_noise_mps");
  if (parameters.has("wall_noise_m")) config.wall_noise_m = parameters.number("wall_noise_m");
  if (parameters.has("wall_fit_band_m")) config.wall_fit_band_m = parameters.number("wall_fit_band_m");
  if (parameters.has("wall_min_points")) config.wall_min_points = parameters.integer("wall_min_points");
  if (parameters.has("wall_min_side_m")) config.wall_min_side_m = parameters.number("wall_min_side_m");
  if (parameters.has("wall_max_range_m")) config.wall_max_range_m = parameters.number("wall_max_range_m");
  if (parameters.has("wall_min_station_m")) config.wall_min_station_m = parameters.number("wall_min_station_m");
  if (parameters.has("wall_max_station_m")) config.wall_max_station_m = parameters.number("wall_max_station_m");
  if (parameters.has("wall_min_height_m")) config.wall_min_height_m = parameters.number("wall_min_height_m");
  if (parameters.has("wall_max_height_m")) config.wall_max_height_m = parameters.number("wall_max_height_m");
  if (parameters.has("initial_state")) {
    const auto values = parameters.numbers("initial_state");
    if (values.size() != 6) throw std::invalid_argument("invalid tunnel wall initial_state size");
    for (std::size_t i = 0; i < 6; ++i) config.initial_state[i] = values[i];
  }
  if (parameters.has("initial_covariance")) {
    const auto values = parameters.numbers("initial_covariance");
    if (values.size() != 36) throw std::invalid_argument("invalid tunnel wall initial_covariance size");
    for (std::size_t row = 0; row < 6; ++row)
      for (std::size_t col = 0; col < 6; ++col)
        config.initial_covariance(row, col) = values[row * 6 + col];
  }
  if (parameters.has("initial_previous_imu_stamp"))
    config.initial_previous_imu_stamp = parameters.integer("initial_previous_imu_stamp");
  if (parameters.has("initial_previous_status_stamp"))
    config.initial_previous_status_stamp = parameters.integer("initial_previous_status_stamp");
  if (parameters.has("initial_previous_lidar_stamp"))
    config.initial_previous_lidar_stamp = parameters.integer("initial_previous_lidar_stamp");
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
}  // namespace localization
