#pragma once
#include <visualizer/cores/object_info.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace visualizer {
inline ObjectInfoParameters loadObjectInfoParameters(const node_pipeline::ParamReader& params) {
  ObjectInfoParameters parameters;
  parameters.yaw_offset_rad = params.number("yaw_offset_rad");
  if (!std::isfinite(parameters.yaw_offset_rad)) throw std::invalid_argument("invalid Object Info yaw offset");
  const auto offset = params.numbers("offset_xyz");
  if (offset.size() != 3) throw std::invalid_argument("Object Info offset_xyz requires three values");
  for (std::size_t i = 0; i < offset.size(); ++i) {
    if (!std::isfinite(offset[i])) throw std::invalid_argument("nonfinite Object Info offset");
    parameters.offset_xyz[i] = offset[i];
  }
  if (params.has("profiles")) {
    for (const auto& name : params.keys("profiles")) {
      const auto values = params.numbers("profiles/" + name);
      if (values.size() != 9) throw std::invalid_argument("Object Info profile requires nine values: " + name);
      std::array<double, 9> profile;
      for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i]) || ((i < 3 || i >= 6) && values[i] <= 0))
          throw std::invalid_argument("invalid Object Info profile: " + name);
        profile[i] = values[i];
      }
      parameters.profiles.push_back(profile);
    }
  }
  return parameters;
}
}  // namespace visualizer
