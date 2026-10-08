#pragma once
#include <path_planner/libraries/checkpoint_route.hpp>

namespace path_planner {
// 섹션 16: 남은 체크포인트가 없으므로 빈 배열을 전달한다.
class Checkpoint16 {
 public:
  explicit Checkpoint16(const CheckpointRouteParameters& parameters) : route_(parameters, 16) {}
  CheckpointRoute::Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                                  const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf) {
    return route_.pass(ego, competition, objects, tf);
  }

 private:
  CheckpointRoute route_;
};
}  // namespace path_planner
