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
  if (params.has("unknown_votes")) p.detection.unknown_votes = params.flag("unknown_votes");
  if (params.has("maximum_center_y_px"))
    p.detection.maximum_center_y_px = params.number("maximum_center_y_px");
  p.detection.history_size = history_size > 0 ? static_cast<std::size_t>(history_size) : 0;
  p.detection.minimum_votes = minimum_votes > 0 ? static_cast<std::size_t>(minimum_votes) : 0;

  auto& t = p.track;
  if (params.has("track/minimum_confidence"))
    t.minimum_confidence = static_cast<float>(params.number("track/minimum_confidence"));
  if (params.has("track/minimum_iou"))
    t.minimum_iou = static_cast<float>(params.number("track/minimum_iou"));
  if (params.has("track/timeout_sec")) t.timeout_sec = params.number("track/timeout_sec");
  if (params.has("track/minimum_lit_pixels"))
    t.minimum_lit_pixels = static_cast<int>(params.integer("track/minimum_lit_pixels"));
  if (params.has("track/lit_saturation"))
    t.lit_saturation = static_cast<int>(params.integer("track/lit_saturation"));
  if (params.has("track/lit_value")) t.lit_value = static_cast<int>(params.integer("track/lit_value"));
  if (params.has("track/red_hue_below"))
    t.red_hue_below = static_cast<int>(params.integer("track/red_hue_below"));
  if (params.has("track/red_hue_above"))
    t.red_hue_above = static_cast<int>(params.integer("track/red_hue_above"));
  const std::array<std::pair<std::string, std::array<int, 2>*>, 2> ranges{{
      {"track/yellow_hue", &t.yellow_hue}, {"track/green_hue", &t.green_hue}}};
  for (const auto& range : ranges) {
    if (!params.has(range.first)) continue;
    const auto values = params.numbers(range.first);
    if (values.size() != 2) throw std::invalid_argument(range.first + " must have two values");
    (*range.second)[0] = static_cast<int>(values[0]);
    (*range.second)[1] = static_cast<int>(values[1]);
  }

  auto& d = p.dehaze;
  if (params.has("dehaze/transmission")) d.transmission = params.number("dehaze/transmission");
  if (params.has("dehaze/window")) d.window = static_cast<int>(params.integer("dehaze/window"));
  if (params.has("dehaze/airlight_fraction"))
    d.airlight_fraction = params.number("dehaze/airlight_fraction");

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
  if (!std::isfinite(p.detection.maximum_center_y_px))
    throw std::invalid_argument("maximum_center_y_px must be finite");
  if (p.detection.history_size == 0) throw std::invalid_argument("history_size must be positive");
  if (p.detection.minimum_votes == 0 || p.detection.minimum_votes > p.detection.history_size)
    throw std::invalid_argument("minimum_votes must be within history_size");
  if (!std::isfinite(t.minimum_confidence) || t.minimum_confidence <= 0 || t.minimum_confidence > 1)
    throw std::invalid_argument("track/minimum_confidence must be finite and in (0,1]");
  if (!std::isfinite(t.minimum_iou) || t.minimum_iou <= 0 || t.minimum_iou > 1)
    throw std::invalid_argument("track/minimum_iou must be finite and in (0,1]");
  if (!std::isfinite(t.timeout_sec) || t.timeout_sec < 0)
    throw std::invalid_argument("track/timeout_sec must be finite and non-negative");
  if (t.minimum_lit_pixels <= 0) throw std::invalid_argument("track/minimum_lit_pixels must be positive");
  if (t.lit_saturation < 0 || t.lit_saturation > 255 || t.lit_value < 0 || t.lit_value > 255)
    throw std::invalid_argument("track/lit_saturation and track/lit_value must be in [0,255]");
  const auto hue = [](int value) { return value >= 0 && value <= 180; };
  if (!hue(t.red_hue_below) || !hue(t.red_hue_above) || t.red_hue_below > t.red_hue_above ||
      !hue(t.yellow_hue[0]) || !hue(t.yellow_hue[1]) || t.yellow_hue[0] > t.yellow_hue[1] ||
      !hue(t.green_hue[0]) || !hue(t.green_hue[1]) || t.green_hue[0] > t.green_hue[1])
    throw std::invalid_argument("track hue bounds must be ordered and in [0,180]");
  if (!std::isfinite(d.transmission) || d.transmission <= 0 || d.transmission > 1)
    throw std::invalid_argument("dehaze/transmission must be finite and in (0,1]");
  if (d.window <= 0) throw std::invalid_argument("dehaze/window must be positive");
  if (!std::isfinite(d.airlight_fraction) || d.airlight_fraction <= 0 || d.airlight_fraction > 1)
    throw std::invalid_argument("dehaze/airlight_fraction must be finite and in (0,1]");
  return p;
}

}  // namespace traffic_light
