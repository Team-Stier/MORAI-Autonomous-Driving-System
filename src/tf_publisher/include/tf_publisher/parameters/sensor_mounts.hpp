#pragma once
#include <tf_publisher/cores/sensor_mounts.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace tf_publisher {
inline SensorMountsParameters loadSensorMountsParameters(const node_pipeline::ParamReader& params) {
  SensorMountsParameters parameters;
  for (const auto& child : params.keys("mounts")) {
    const auto translation = params.numbers("mounts/" + child + "/translation");
    const auto rotation = params.numbers("mounts/" + child + "/rotation");
    if (child == "base_link" || translation.size() != 3 || rotation.size() != 4)
      throw std::invalid_argument("invalid sensor mount: " + child);
    for (const double value : translation)
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite sensor mount translation: " + child);
    double norm = 0;
    for (const double value : rotation) {
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite sensor mount rotation: " + child);
      norm += value * value;
    }
    if (std::fabs(norm - 1) > 1e-6) throw std::invalid_argument("sensor mount rotation must be a unit quaternion: " + child);
    SensorMount mount;
    mount.child_frame = child;
    for (std::size_t i = 0; i < 3; ++i) mount.translation_m[i] = translation[i];
    for (std::size_t i = 0; i < 4; ++i) mount.rotation_xyzw[i] = rotation[i];
    parameters.mounts.push_back(mount);
  }
  if (parameters.mounts.empty()) throw std::invalid_argument("sensor mounts are empty");
  return parameters;
}
}  // namespace tf_publisher
