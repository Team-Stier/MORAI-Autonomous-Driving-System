#pragma once

#include <traffic_light/types.hpp>
#include <ros/node_handle.h>
#include <cmath>
#include <stdexcept>

namespace traffic_light {

inline TrafficLightParameters defaultTrafficLightParameters() {
  return {};
}

inline void validateTrafficLightParameters(const TrafficLightParameters& p) {
  if (p.model_path.empty()) throw std::invalid_argument("model_path is required");
  if (p.manifest_path.empty()) throw std::invalid_argument("manifest_path is required");
  if (p.input_width <= 0) throw std::invalid_argument("input_width must be positive");
  if (p.input_height <= 0) throw std::invalid_argument("input_height must be positive");
  for (const float threshold : p.detection.confidence) {
    if (!std::isfinite(threshold) || threshold < 0 || threshold > 1)
      throw std::invalid_argument("confidence must be finite and in [0,1]");
  }
  if (!std::isfinite(p.detection.nms_iou) || p.detection.nms_iou < 0 || p.detection.nms_iou > 1)
    throw std::invalid_argument("nms_iou must be finite and in [0,1]");
  if (!std::isfinite(p.detection.ambiguity_margin) || p.detection.ambiguity_margin < 0 ||
      p.detection.ambiguity_margin > 1)
    throw std::invalid_argument("ambiguity_margin must be finite and in [0,1]");
  if (p.detection.history_size == 0) throw std::invalid_argument("history_size must be positive");
  if (p.detection.minimum_votes == 0 || p.detection.minimum_votes > p.detection.history_size)
    throw std::invalid_argument("minimum_votes must be within history_size");
}

inline TrafficLightParameters loadTrafficLightParameters(ros::NodeHandle& node) {
  TrafficLightParameters p = defaultTrafficLightParameters();
  node.param("model_path", p.model_path, p.model_path);
  node.param("manifest_path", p.manifest_path, p.manifest_path);
  node.param("input_width", p.input_width, p.input_width);
  node.param("input_height", p.input_height, p.input_height);
  node.param("allow_cpu_fallback", p.allow_cpu_fallback, p.allow_cpu_fallback);
  double value = 0;
  const std::array<std::string, 4> names{{"go", "left_turn", "prepare_stop", "stop"}};
  for (std::size_t i = 0; i < names.size(); ++i) {
    value = p.detection.confidence[i];
    node.param("confidence/" + names[i], value, value);
    p.detection.confidence[i] = static_cast<float>(value);
  }
  value = p.detection.nms_iou;
  node.param("nms_iou", value, value);
  p.detection.nms_iou = static_cast<float>(value);
  value = p.detection.ambiguity_margin;
  node.param("ambiguity_margin", value, value);
  p.detection.ambiguity_margin = static_cast<float>(value);
  int history_size = static_cast<int>(p.detection.history_size);
  int minimum_votes = static_cast<int>(p.detection.minimum_votes);
  node.param("history_size", history_size, history_size);
  node.param("minimum_votes", minimum_votes, minimum_votes);
  p.detection.history_size = history_size > 0 ? static_cast<std::size_t>(history_size) : 0;
  p.detection.minimum_votes = minimum_votes > 0 ? static_cast<std::size_t>(minimum_votes) : 0;
  validateTrafficLightParameters(p);
  return p;
}

}  // namespace traffic_light
