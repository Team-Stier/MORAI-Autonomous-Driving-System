#pragma once
#include <interfaces/SectionAnnotation.h>
#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <array>
#include <cstddef>
#include <string>
namespace traffic_light {
using Message = interfaces::TrafficLight;

enum class DetectionState : std::uint8_t { Unknown, Go, LeftTurn, PrepareStop, Stop };

struct DetectionParameters {
  std::array<float, 4> confidence{{0.4F, 0.4F, 0.35F, 0.4F}};
  float nms_iou = 0.45F;
  float ambiguity_margin = 0.05F;
  std::size_t history_size = 5;
  std::size_t minimum_votes = 3;
};
}  // namespace traffic_light
