#pragma once
#include <node_pipeline/extension.hpp>
#include <visualizer/cores/ego_vehicle.hpp>

namespace visualizer {
inline EgoVehicleParameters loadEgoVehicleParameters(const node_pipeline::ParamReader& params) {
  return {params.flag("gt"), params.flag("flatten_z")};
}
}  // namespace visualizer
