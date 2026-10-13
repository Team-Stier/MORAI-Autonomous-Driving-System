#pragma once
#include <visualizer/cores/local_path.hpp>
#include <node_pipeline/extension.hpp>

namespace visualizer {
inline LocalPathParameters loadLocalPathParameters(const node_pipeline::ParamReader& params) {
  return {params.flag("flatten_z")};
}
}  // namespace visualizer
