#pragma once
#include <object_detector/cores/tracker.hpp>
#include <node_pipeline/sdk.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace object_detector { namespace parameters {
struct Tracker {
  TrackerParameters operator()(const node_pipeline::Parameters& parameters) const {
    TrackerParameters config;
    config.yaw_offset_rad = parameters.get("yaw_offset_rad", config.yaw_offset_rad);
    config.association_gate_m = parameters.get("association_gate_m", config.association_gate_m);
    config.shape_weight = parameters.get("shape_weight", config.shape_weight);
    config.max_age_sec = parameters.get("max_age_sec", config.max_age_sec);
    config.history_size = parameters.get("history_size", config.history_size);
    config.min_acceleration_observations = parameters.get("min_acceleration_observations", config.min_acceleration_observations);
    if (!std::isfinite(config.yaw_offset_rad) || !std::isfinite(config.association_gate_m) ||
        config.association_gate_m <= 0 || !std::isfinite(config.shape_weight) || config.shape_weight < 0 ||
        !std::isfinite(config.max_age_sec) || config.max_age_sec <= 0 ||
        config.min_acceleration_observations < 5 || config.history_size < config.min_acceleration_observations)
      throw std::invalid_argument("invalid tracker settings");
    return config;
  }
};
}}  // namespace object_detector::parameters
