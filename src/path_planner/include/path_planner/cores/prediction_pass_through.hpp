#pragma once
#include <path_planner/types.hpp>

namespace path_planner {
class PredictionPassThrough {
 public:
  using Result = PredictedObjects;

  Result process(const interfaces::Objects& objects,
                 const Route&) const {
    Result result;
    result.fill(objects);
    return result;
  }
};
}  // namespace path_planner
