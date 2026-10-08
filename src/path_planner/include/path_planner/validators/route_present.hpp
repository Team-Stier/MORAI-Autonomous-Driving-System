#pragma once
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <vector>

namespace path_planner {
// 경로 탐색 결과가 없으면 route를 Missing으로 기록한다.
struct RoutePresent {
  void operator()(const Route& route, const node_pipeline::HealthLimits&, std::vector<node_pipeline::Kind>& issues) const {
    if (!route) issues.push_back(node_pipeline::Kind::Missing);
  }
};
}  // namespace path_planner
