#pragma once
#include <path_planner/libraries/types.hpp>
#include <sensor_msgs/Imu.h>
#include <array>
#include <tuple>

namespace path_planner {
struct CheckpointArrivalParameters {
  std::array<Checkpoint, 16> checkpoints{};
};

// 섹션 17: 출발·도착 lanelet 하나만 목적지로 전달한다.
class CheckpointArrival {
 public:
  explicit CheckpointArrival(const CheckpointArrivalParameters& parameters)
      : remaining_{parameters.checkpoints[0]} {}
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
