#pragma once
#include <visualizer/cores/object_boxes.hpp>
#include <node_pipeline/extension.hpp>
#include <stdexcept>

namespace visualizer {
inline ObjectBoxesParameters loadObjectBoxesParameters(const node_pipeline::ParamReader& params) {
  ObjectBoxesParameters parameters;
  const auto prediction_horizon = params.integer("prediction_horizon");
  if (prediction_horizon < -1 || prediction_horizon > 9)
    throw std::invalid_argument("prediction_horizon must be -1 or 0 through 9");
  parameters.prediction_horizon = static_cast<int>(prediction_horizon);
  parameters.flatten_z = params.flag("flatten_z");
  return parameters;
}
}  // namespace visualizer
