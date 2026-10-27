#pragma once
#include <path_planner/cores/checkpoint_02.hpp>
#include <path_planner/libraries/checkpoints.hpp>
#include <node_pipeline/extension.hpp>

namespace path_planner {
inline Checkpoint02Parameters loadCheckpoint02Parameters(const node_pipeline::ParamReader& params) {
  return {readCheckpoints(params.asset("checkpoints_file"))};
}
}  // namespace path_planner
