#pragma once
#include <path_planner/libraries/types.hpp>
#include <tuple>

namespace path_planner {
// 관측 Objects를 10개 예측 시점에 그대로 복사하고 route·ego를 다음 Stage로 넘긴다.
class PredictionPassThrough {
 public:
  using Result = std::tuple<PredictedObjects, Route, nav_msgs::Odometry>;

  Result process(const interfaces::Objects& objects, const Route& route, const nav_msgs::Odometry& ego) const {
    PredictedObjects predictions;
    predictions.fill(objects);
    return std::make_tuple(predictions, route, ego);
  }
};
}  // namespace path_planner
