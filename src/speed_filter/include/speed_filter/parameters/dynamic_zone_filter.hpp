#pragma once
#include <speed_filter/cores/dynamic_zone_filter.hpp>
#include <speed_filter/libraries/speed_filter_parameters.hpp>

namespace speed_filter {
inline SpeedFilterParameters loadDynamicZoneFilterParameters(const node_pipeline::ParamReader& params) {
  return readSpeedFilterParameters(params);
}
}  // namespace speed_filter
