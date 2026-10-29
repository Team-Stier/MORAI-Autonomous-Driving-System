#pragma once
#include <path_planner/cores/rddf_core.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace path_planner {
inline RddfCoreParameters loadRddfCoreParameters(const node_pipeline::ParamReader& params) {
  RddfCoreParameters parameters;
  if (params.has("local_path_length_m"))
    parameters.local_path_length_m = params.number("local_path_length_m");
  if (!std::isfinite(parameters.local_path_length_m) || parameters.local_path_length_m <= 0.0)
    throw std::invalid_argument("local_path_length_m must be finite and positive");

  std::istringstream asset(params.asset("global_path_file"));
  std::string line;
  bool has_segment = false;
  while (std::getline(asset, line)) {
    std::istringstream row(line);
    row >> std::ws;
    if (row.eof()) continue;
    geometry_msgs::Point point;
    if (!(row >> point.x >> point.y >> point.z) ||
        !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::invalid_argument("global_path_file rows must contain finite x y z in map meters");
    row >> std::ws;
    if (!row.eof()) throw std::invalid_argument("unexpected global_path_file columns");
    if (!parameters.global_path.empty()) {
      const auto& previous = parameters.global_path.back();
      has_segment = has_segment || point.x != previous.x || point.y != previous.y;
    }
    parameters.global_path.push_back(point);
  }
  if (!has_segment) throw std::invalid_argument("global_path_file needs a nonzero XY segment");
  return parameters;
}
}  // namespace path_planner
