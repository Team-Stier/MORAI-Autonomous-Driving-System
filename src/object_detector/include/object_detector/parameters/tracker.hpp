#pragma once
#include <object_detector/cores/tracker.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace object_detector {
inline TrackerParameters loadTrackerParameters(const node_pipeline::ParamReader& reader) {
  TrackerParameters p;
  p.yaw_offset_rad = reader.number("yaw_offset_rad");
  p.association_gate_m = reader.number("association_gate_m");
  p.shape_weight = reader.number("shape_weight");
  p.max_age_sec = reader.number("max_age_sec");
  p.history_size = static_cast<int>(reader.integer("history_size"));
  p.min_acceleration_observations = static_cast<int>(reader.integer("min_acceleration_observations"));
  if (!std::isfinite(p.yaw_offset_rad) || !std::isfinite(p.association_gate_m) ||
      p.association_gate_m <= 0 || !std::isfinite(p.shape_weight) || p.shape_weight < 0 ||
      !std::isfinite(p.max_age_sec) || p.max_age_sec <= 0 || p.history_size < 3 ||
      p.min_acceleration_observations < 3 || p.min_acceleration_observations > p.history_size)
    throw std::invalid_argument("invalid tracker parameters");
  return p;
}
}  // namespace object_detector
