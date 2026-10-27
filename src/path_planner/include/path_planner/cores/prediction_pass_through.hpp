#pragma once
#include <sensor_msgs/Imu.h>
#include <path_planner/libraries/types.hpp>
#include <tuple>

namespace path_planner {
// 관측 Objects를 10개 예측 시점에 그대로 복사하고 입력을 다음 Stage로 넘긴다.
class PredictionPassThrough {
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
                            interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu, Route, PredictedObjects>;

  Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) const {
    PredictedObjects predictions;
    predictions.fill(objects);
    return std::make_tuple(ego, competition, objects, tf, imu, route, predictions);
  }
};
}  // namespace path_planner
