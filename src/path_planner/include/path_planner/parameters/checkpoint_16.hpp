#pragma once
#include <path_planner/cores/checkpoint_16.hpp>
#include <path_planner/libraries/checkpoint_route.hpp>

namespace path_planner {
inline CheckpointRouteParameters loadCheckpoint16Parameters(const node_pipeline::ParamReader& params) {
  return readCheckpointRouteParameters(params);
}
}  // namespace path_planner
