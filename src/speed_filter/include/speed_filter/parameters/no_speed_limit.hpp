#pragma once
#include <speed_filter/cores/no_speed_limit.hpp>
#include <speed_filter/libraries/speed_filter_parameters.hpp>

namespace speed_filter {
inline SpeedFilterParameters loadNoSpeedLimitParameters(const node_pipeline::ParamReader& params) {
  return readSpeedFilterParameters(params);
}
}  // namespace speed_filter
