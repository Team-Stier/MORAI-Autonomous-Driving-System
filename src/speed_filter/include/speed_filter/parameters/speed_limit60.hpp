#pragma once
#include <speed_filter/cores/speed_limit60.hpp>
#include <speed_filter/libraries/speed_filter_parameters.hpp>

namespace speed_filter {
inline SpeedFilterParameters loadSpeedLimit60Parameters(const node_pipeline::ParamReader& params) {
  return readSpeedFilterParameters(params);
}
}  // namespace speed_filter
