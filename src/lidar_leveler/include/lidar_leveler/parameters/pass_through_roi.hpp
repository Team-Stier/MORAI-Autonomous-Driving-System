#pragma once
#include <lidar_leveler/cores/pass_through_roi.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace lidar_leveler {
inline PassThroughRoiParameters loadPassThroughRoiParameters(const node_pipeline::ParamReader& params) {
  PassThroughRoiParameters config;
  if (params.has("z_min_m")) config.z_min_m = params.number("z_min_m");
  if (params.has("z_max_m")) config.z_max_m = params.number("z_max_m");
  if (!std::isfinite(config.z_min_m) || !std::isfinite(config.z_max_m) || config.z_min_m > config.z_max_m)
    throw std::invalid_argument("pass_through_roi z ROI must have finite bounds with z_min_m <= z_max_m");
  return config;
}
}  // namespace lidar_leveler
