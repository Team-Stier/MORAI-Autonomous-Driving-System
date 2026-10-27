#pragma once
#include <path_planner/libraries/types.hpp>
#include <sensor_msgs/Imu.h>
#include <array>
#include <tuple>

namespace path_planner {
struct Checkpoint01Parameters {
  std::array<Checkpoint, 16> checkpoints{};
};

// 섹션 1: 남은 체크포인트 2~16을 전달한다.
class Checkpoint01 {
 public:
  explicit Checkpoint01(const Checkpoint01Parameters& parameters)
      : remaining_(parameters.checkpoints.begin() + 1, parameters.checkpoints.end()) {}
  std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::Objects,
             tf2_msgs::TFMessage, sensor_msgs::Imu, Checkpoints> process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                                  const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                                  const sensor_msgs::Imu& imu) {
    return std::make_tuple(ego, competition, objects, tf, imu, remaining_);
  }

 private:
  Checkpoints remaining_;
};
}  // namespace path_planner
