#pragma once
#include <path_planner/libraries/types.hpp>
#include <memory>
#include <utility>

namespace path_planner {
struct SharedMapParameters {
  std::shared_ptr<lanelet::LaneletMap> map;
};

// GlobalPath와 LocalPath가 같은 lanelet 인스턴스를 쓰도록 노드당 한 번 읽은 지도를 공유한다.
class SharedMap {
 public:
  explicit SharedMap(SharedMapParameters parameters) : map(std::move(parameters.map)) {}
  std::shared_ptr<lanelet::LaneletMap> map;
};
}  // namespace path_planner
