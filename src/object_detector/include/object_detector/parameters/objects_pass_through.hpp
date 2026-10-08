#pragma once
#include <object_detector/cores/objects_pass_through.hpp>
#include <node_pipeline/extension.hpp>

namespace object_detector {
inline ObjectsPassThroughParameters loadObjectsPassThroughParameters(const node_pipeline::ParamReader&) {
  return {};
}
}  // namespace object_detector
