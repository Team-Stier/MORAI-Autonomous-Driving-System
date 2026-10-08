#pragma once
#include <path_planner/libraries/checkpoint_route.hpp>

namespace path_planner {
// 섹션 6: 남은 체크포인트 7~16을 전달한다.
class Checkpoint06 {
 public:
  explicit Checkpoint06(const CheckpointRouteParameters& parameters) : route_(parameters, 6) {}
  CheckpointRoute::Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                                  const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf) {
    return route_.pass(ego, competition, objects, tf);
  }

 private:
  CheckpointRoute route_;
};
}  // namespace path_planner
