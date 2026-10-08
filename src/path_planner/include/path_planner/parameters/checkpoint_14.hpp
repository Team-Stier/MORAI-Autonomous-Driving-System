#pragma once
#include <path_planner/cores/checkpoint_14.hpp>
#include <path_planner/libraries/checkpoint_route.hpp>

namespace path_planner {
inline CheckpointRouteParameters loadCheckpoint14Parameters(const node_pipeline::ParamReader& params) {
  return readCheckpointRouteParameters(params);
}
}  // namespace path_planner
