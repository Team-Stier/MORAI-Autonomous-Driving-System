#pragma once
#include <object_detector/cores/z_roi.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace object_detector {
inline ZRoiParameters loadZRoiParameters(const node_pipeline::ParamReader& params) {
  ZRoiParameters config;
  config.z_min_m = params.number("z_min_m");
  config.z_max_m = params.number("z_max_m");
  if (!std::isfinite(config.z_min_m) || !std::isfinite(config.z_max_m) ||
      config.z_min_m > config.z_max_m)
    throw std::invalid_argument("z_roi bounds must be finite with z_min_m <= z_max_m");
  return config;
}
}  // namespace object_detector
