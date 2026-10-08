#pragma once
#include <lidar_leveler/cores/roll_pitch_roi_leveler.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace lidar_leveler {
inline RollPitchRoiLevelerParameters loadRollPitchRoiLevelerParameters(const node_pipeline::ParamReader& params) {
  RollPitchRoiLevelerParameters config;
  config.z_min_m = params.number("z_min_m");
  config.z_max_m = params.number("z_max_m");
  if (!std::isfinite(config.z_min_m) || !std::isfinite(config.z_max_m) || config.z_min_m > config.z_max_m)
    throw std::invalid_argument("roll_pitch_roi_leveler z ROI must have finite bounds with z_min_m <= z_max_m");
  return config;
}
}  // namespace lidar_leveler
