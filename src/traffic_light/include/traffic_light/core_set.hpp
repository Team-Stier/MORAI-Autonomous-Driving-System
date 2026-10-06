#pragma once
#include <traffic_light/cores/yolo_traffic_light_core.hpp>
#include <traffic_light/types.hpp>
#include <utility>
namespace traffic_light {
enum class CoreId { YoloTrafficLight };
struct CoreSet {
  explicit CoreSet(TrafficLightParameters parameters)
      : yolo_traffic_light(std::move(parameters)) {}
  YoloTrafficLightCore yolo_traffic_light;
};
}  // namespace traffic_light
