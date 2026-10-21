#pragma once
#include <speed_filter/cores/speed_limit60.hpp>
#include <speed_filter/libraries/speed_filter_parameters.hpp>
#include <sstream>

namespace speed_filter {
inline SpeedLimit60Parameters loadSpeedLimit60Parameters(const node_pipeline::ParamReader& params) {
  SpeedLimit60Parameters result;
  static_cast<SpeedFilterParameters&>(result) = readSpeedFilterParameters(params);
  if (params.has("speed_exemption_links_file")) {
    std::istringstream links(params.asset("speed_exemption_links_file"));
    std::string id;
    while (links >> id) result.speed_exemption_link_ids.insert(id);
  }
  return result;
}
}  // namespace speed_filter
