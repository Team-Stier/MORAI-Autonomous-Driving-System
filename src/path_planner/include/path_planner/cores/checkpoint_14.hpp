#pragma once
#include <path_planner/libraries/checkpoint_route.hpp>

namespace path_planner {
// 섹션 14: 남은 체크포인트 15~16을 전달한다.
class Checkpoint14 {
 public:
  explicit Checkpoint14(const CheckpointRouteParameters& parameters) : route_(parameters, 14) {}
  CheckpointRoute::Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                                  const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf) {
    return route_.pass(ego, competition, objects, tf);
  }

 private:
  CheckpointRoute route_;
};
}  // namespace path_planner
