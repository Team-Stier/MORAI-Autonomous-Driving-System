#pragma once
#include <lidar_leveler/types.hpp>
#include <node_pipeline/sdk.hpp>
#include <cmath>
#include <stdexcept>
namespace lidar_leveler { namespace parameters {
struct PassThrough {
  PassThroughParameters operator()(const node_pipeline::Parameters& params) const {
    PassThroughParameters config;
    config.z_min_m = params.get("z_min_m", config.z_min_m);
    config.z_max_m = params.get("z_max_m", config.z_max_m);
    if (!std::isfinite(config.z_min_m) || !std::isfinite(config.z_max_m) || config.z_min_m > config.z_max_m)
      throw std::invalid_argument("pass_through z ROI must have finite bounds with z_min_m <= z_max_m");
    return config;
  }
};
}}
