#pragma once

#include <traffic_light/cores/yolo_traffic_light.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace traffic_light {

inline YoloTrafficLightParameters loadYoloTrafficLightParameters(
    const node_pipeline::ParamReader& params) {
  YoloTrafficLightParameters p;
  p.model = params.asset("model");
  if (params.has("input_width")) p.input_width = static_cast<int>(params.integer("input_width"));
  if (params.has("input_height")) p.input_height = static_cast<int>(params.integer("input_height"));
  const std::array<std::string, 4> names{{"go", "left_turn", "prepare_stop", "stop"}};
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (params.has("confidence/" + names[i]))
      p.detection.confidence[i] = static_cast<float>(params.number("confidence/" + names[i]));
  }
  if (params.has("nms_iou")) p.detection.nms_iou = static_cast<float>(params.number("nms_iou"));
  if (params.has("ambiguity_margin"))
    p.detection.ambiguity_margin = static_cast<float>(params.number("ambiguity_margin"));
  int history_size = static_cast<int>(p.detection.history_size);
  int minimum_votes = static_cast<int>(p.detection.minimum_votes);
  if (params.has("history_size")) history_size = static_cast<int>(params.integer("history_size"));
  if (params.has("minimum_votes")) minimum_votes = static_cast<int>(params.integer("minimum_votes"));
  p.detection.history_size = history_size > 0 ? static_cast<std::size_t>(history_size) : 0;
  p.detection.minimum_votes = minimum_votes > 0 ? static_cast<std::size_t>(minimum_votes) : 0;

  if (p.model.empty()) throw std::invalid_argument("model is required");
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
  return p;
}

}  // namespace traffic_light
