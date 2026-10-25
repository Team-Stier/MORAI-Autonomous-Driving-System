#pragma once
#include <path_planner/libraries/types.hpp>
#include <sensor_msgs/Imu.h>
#include <tuple>

namespace path_planner {
class BboxPredictionAdapter {
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
                            interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu, Route, PredictedObjects>;

  Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) const {
    PredictedObjects predictions;
    for (std::size_t i = 0; i < predictions.size(); ++i) {
      auto& snapshot = predictions[i];
      snapshot.header = objects.header;
      snapshot.objects.reserve(objects.objects.size());
      for (const auto& object : objects.objects) {
        interfaces::Object future;
        future.id = object.id;
        future.label = object.label;
        future.bbox_size = object.bbox_size;
        future.bbox_pose = object.predicted_bbox_poses[i];
        future.position = future.bbox_pose.position;
        snapshot.objects.push_back(future);
      }
    }
    return std::make_tuple(ego, competition, objects, tf, imu, route, predictions);
  }
};
}  // namespace path_planner
