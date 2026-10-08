#pragma once
#include <path_planner/libraries/checkpoints.hpp>
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <tuple>

namespace path_planner {
struct CheckpointRouteParameters {
  std::array<Checkpoint, 16> checkpoints{};
};

inline CheckpointRouteParameters readCheckpointRouteParameters(const node_pipeline::ParamReader& params) {
  return {readCheckpoints(params.asset("checkpoints_file"))};
}

// 체크포인트 Core의 공통 계산이다. 구독 입력을 그대로 넘기고 completed 이후의 체크포인트를 덧붙인다.
class CheckpointRoute {
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::Objects,
                            tf2_msgs::TFMessage, Checkpoints>;

  CheckpointRoute(const CheckpointRouteParameters& parameters, std::size_t completed) {
    if (completed > parameters.checkpoints.size()) throw std::invalid_argument("completed checkpoints out of range");
    remaining_.assign(parameters.checkpoints.begin() + static_cast<std::ptrdiff_t>(completed), parameters.checkpoints.end());
  }

  Result pass(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
              const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf) const {
    return std::make_tuple(ego, competition, objects, tf, remaining_);
  }

 private:
  Checkpoints remaining_;
};
}  // namespace path_planner
