#pragma once
#include <lidar_leveler/cores/roll_pitch_roi_leveler.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace lidar_leveler {
inline RollPitchRoiLevelerParameters loadRollPitchRoiLevelerParameters(const node_pipeline::ParamReader& params) {
  RollPitchRoiLevelerParameters parameters;
  parameters.z_min_m = params.number("z_min_m");
  parameters.z_max_m = params.number("z_max_m");
  if (!std::isfinite(parameters.z_min_m) || !std::isfinite(parameters.z_max_m) || parameters.z_min_m > parameters.z_max_m)
    throw std::invalid_argument("z_min_m and z_max_m must be finite and ordered");
  return parameters;
}
}  // namespace lidar_leveler
