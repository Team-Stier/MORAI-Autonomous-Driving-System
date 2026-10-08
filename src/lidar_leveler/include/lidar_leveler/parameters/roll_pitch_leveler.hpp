#pragma once
#include <lidar_leveler/cores/roll_pitch_leveler.hpp>
#include <node_pipeline/extension.hpp>

namespace lidar_leveler {
inline RollPitchLevelerParameters loadRollPitchLevelerParameters(const node_pipeline::ParamReader&) {
  return {};
}
}  // namespace lidar_leveler
