#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/map_parameters.hpp>
#include <node_pipeline/extension.hpp>

namespace path_planner {
// 지도를 읽고 체크포인트 CSV의 lanelet_id가 모두 지도에 있는지 시작 시 확인한다.
inline SharedMapParameters loadSharedMapParameters(const node_pipeline::ParamReader& params) {
  return {readCheckedLaneletMap(params.asset("map_file"), params.asset("checkpoints_file"))};
}
}  // namespace path_planner
