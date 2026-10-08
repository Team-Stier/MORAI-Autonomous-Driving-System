#pragma once
#include <traffic_light/cores/yolo_traffic_light.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

namespace traffic_light {
inline YoloTrafficLightParameters loadYoloTrafficLightParameters(const node_pipeline::ParamReader& params) {
  YoloTrafficLightParameters parameters;
  parameters.model = params.asset("model");
  const std::string device = params.has("device") ? params.text("device") : "cuda";
  if (device == "cuda") {
    parameters.device = InferenceDevice::Cuda;
  } else if (device == "cpu") {
    parameters.device = InferenceDevice::Cpu;
  } else {
    throw std::invalid_argument("yolo_traffic_light device must be cuda or cpu");
  }
  const auto positive = [&](const std::string& key, std::int64_t maximum) {
    const auto value = params.integer(key);
    if (value <= 0 || value > maximum) throw std::invalid_argument("invalid yolo_traffic_light " + key);
    return value;
  };
  const auto probability = [&](const std::string& key) {
    const double value = params.number(key);
    if (!std::isfinite(value) || value < 0 || value > 1)
      throw std::invalid_argument("yolo_traffic_light " + key + " must be in [0, 1]");
    return static_cast<float>(value);
  };
  parameters.input_width = static_cast<int>(positive("input_width", 8192));
  parameters.input_height = static_cast<int>(positive("input_height", 8192));

  const auto keys = params.keys("classes");
  if (keys.empty()) throw std::invalid_argument("yolo_traffic_light classes must not be empty");
  parameters.classes.resize(keys.size());
  std::vector<bool> seen(keys.size(), false);
  for (const auto& key : keys) {
    std::size_t parsed = 0;
    unsigned long index = 0;
    try {
      index = std::stoul(key, &parsed);
    } catch (const std::exception&) {
      parsed = 0;
    }
    if (key.empty() || parsed != key.size() || index >= keys.size() || seen[index])
      throw std::invalid_argument("yolo_traffic_light classes keys must be the class indices 0..N-1");
    seen[index] = true;
    LampClass& entry = parameters.classes[index];
    const std::string prefix = "classes/" + key + "/";
    entry.name = params.text(prefix + "name");
    const std::string lamp = params.text(prefix + "lamp");
    if (lamp == "red") {
      entry.lamp = Lamp::Red;
    } else if (lamp == "yellow") {
      entry.lamp = Lamp::Yellow;
    } else if (lamp == "green") {
      entry.lamp = Lamp::Green;
    } else if (lamp == "left") {
      entry.lamp = Lamp::Left;
    } else {
      throw std::invalid_argument("yolo_traffic_light " + prefix + "lamp must be red, yellow, green or left");
    }
    entry.confidence = probability(prefix + "confidence");
  }
  parameters.nms_iou = probability("nms_iou");
  parameters.history_size = static_cast<std::size_t>(positive("history_size", 1000));
  parameters.minimum_votes = static_cast<std::size_t>(positive("minimum_votes", 1000));
  if (parameters.minimum_votes > parameters.history_size)
    throw std::invalid_argument("yolo_traffic_light minimum_votes must not exceed history_size");
  return parameters;
}
}  // namespace traffic_light
