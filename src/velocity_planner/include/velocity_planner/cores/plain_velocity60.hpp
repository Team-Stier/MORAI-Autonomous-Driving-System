#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Path.h>
#include <interfaces/TrafficLight.h>
#include <tf2_msgs/TFMessage.h>
#include <cmath>
#include <tuple>
#include <utility>

namespace velocity_planner {
class PlainVelocity60 {
 public:
  std::tuple<interfaces::Path, interfaces::TrafficLight,
             interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const interfaces::Path& path, const interfaces::TrafficLight& light,
      const interfaces::CompetitionVehicleStatus& competition, const tf2_msgs::TFMessage& tf) {
    interfaces::Path message = path;
    for (auto& point : message.points) {
      if (std::isnan(point.velocity)) {
        point.velocity = 60.0 / 3.6;
        point.acceleration = 0.0;
      }
    }
    return {std::move(message), light, competition, tf};
  }
};
}  // namespace velocity_planner
