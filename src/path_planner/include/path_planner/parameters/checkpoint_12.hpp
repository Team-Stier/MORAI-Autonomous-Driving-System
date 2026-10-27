#pragma once
#include <path_planner/cores/checkpoint_12.hpp>
#include <path_planner/libraries/checkpoints.hpp>
#include <node_pipeline/extension.hpp>

namespace path_planner {
inline Checkpoint12Parameters loadCheckpoint12Parameters(const node_pipeline::ParamReader& params) {
  return {readCheckpoints(params.asset("checkpoints_file"))};
}
}  // namespace path_planner
