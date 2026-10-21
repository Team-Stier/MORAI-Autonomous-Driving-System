#pragma once
#include <path_planner/cores/global_path.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline GlobalPathParameters loadGlobalPathParameters(const node_pipeline::ParamReader& params) {
  GlobalPathParameters result;
  for (double id : params.numbers("adjacent_checkpoint_lanelet_ids")) {
    if (!std::isfinite(id) || id <= 0 || id > 9007199254740991.0 || std::floor(id) != id)
      throw std::invalid_argument("invalid adjacent checkpoint lanelet ID");
    result.adjacent_checkpoint_lanelet_ids.push_back(static_cast<lanelet::Id>(id));
  }
  // 섹션 판정 Core의 기존 정적 설정에서 같은 반경을 읽는다. 계산은 Core에 둔다.
  result.checkpoint_radius_m = node_pipeline::ParamReader::fromString(
      params.asset("section_parameters_file")).number("radius_m");
  if (!std::isfinite(result.checkpoint_radius_m) || result.checkpoint_radius_m <= 0)
    throw std::invalid_argument("invalid checkpoint radius");
  return result;
}
}  // namespace path_planner
