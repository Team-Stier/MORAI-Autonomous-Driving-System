#pragma once
#include <traffic_light/cores/yolo_traffic_light.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace traffic_light {
inline YoloTrafficLightParameters loadYoloTrafficLightParameters(const node_pipeline::ParamReader& params) {
  YoloTrafficLightParameters p;
  p.model = params.asset("model");
  if (p.model.empty()) throw std::invalid_argument("model is required");
  p.device = params.has("device") ? params.text("device") : "cuda";
  if (p.device != "cuda" && p.device != "cpu") throw std::invalid_argument("device must be cuda or cpu");
  p.input_width = static_cast<int>(params.integer("input_width"));
  p.input_height = static_cast<int>(params.integer("input_height"));
  if (p.input_width <= 0) throw std::invalid_argument("input_width must be positive");
  if (p.input_height <= 0) throw std::invalid_argument("input_height must be positive");

  // confidence의 키 순서가 모델 출력 클래스 인덱스 순서다.
  const std::vector<std::string> names{"go", "left_turn", "prepare_stop", "stop"};
  if (params.keys("confidence") != names)
    throw std::invalid_argument("class order must be go,left_turn,prepare_stop,stop");
  for (std::size_t i = 0; i < names.size(); ++i) {
    p.detection.confidence[i] = static_cast<float>(params.number("confidence/" + names[i]));
  }
  p.detection.nms_iou = static_cast<float>(params.number("nms_iou"));
  const std::int64_t history_size = params.integer("history_size");
  const std::int64_t minimum_votes = params.integer("minimum_votes");
  p.detection.history_size = history_size > 0 ? static_cast<std::size_t>(history_size) : 0;
  p.detection.minimum_votes = minimum_votes > 0 ? static_cast<std::size_t>(minimum_votes) : 0;

  for (const float threshold : p.detection.confidence) {
    if (!std::isfinite(threshold) || threshold < 0 || threshold > 1)
      throw std::invalid_argument("confidence must be finite and in [0,1]");
  }
  if (!std::isfinite(p.detection.nms_iou) || p.detection.nms_iou < 0 || p.detection.nms_iou > 1)
    throw std::invalid_argument("nms_iou must be finite and in [0,1]");
  if (p.detection.history_size == 0) throw std::invalid_argument("history_size must be positive");
  if (p.detection.minimum_votes == 0 || p.detection.minimum_votes > p.detection.history_size)
    throw std::invalid_argument("minimum_votes must be within history_size");
  return p;
}
}  // namespace traffic_light
