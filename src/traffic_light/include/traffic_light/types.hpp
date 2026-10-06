#pragma once
#include <node_pipeline/components.hpp>
#include <interfaces/SectionAnnotation.h>
#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <array>
#include <cstddef>
#include <string>
namespace traffic_light {
using Data = node_pipeline::Data<sensor_msgs::CompressedImage>;
using Message = interfaces::TrafficLight;

enum class DetectionState : std::uint8_t { Unknown, Go, LeftTurn, PrepareStop, Stop };

struct DetectionParameters {
  std::array<float, 4> confidence{{0.4F, 0.4F, 0.35F, 0.4F}};
  float nms_iou = 0.45F;
  float ambiguity_margin = 0.05F;
  std::size_t history_size = 5;
  std::size_t minimum_votes = 3;
};

struct TrafficLightParameters {
  std::string model_path;
  std::string manifest_path;
  int input_width = 1280;
  int input_height = 1280;
  bool allow_cpu_fallback = false;
  DetectionParameters detection;
};
}  // namespace traffic_light
