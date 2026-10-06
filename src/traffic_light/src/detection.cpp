#include <traffic_light/detection.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace traffic_light {
namespace {

DetectionState stateForClass(int class_id) {
  static const std::array<DetectionState, 4> states{{
      DetectionState::Go, DetectionState::LeftTurn,
      DetectionState::PrepareStop, DetectionState::Stop}};
  return states.at(static_cast<std::size_t>(class_id));
}

}  // namespace

std::vector<Detection> decodeYoloOutput(const float* data,
                                        const std::vector<std::int64_t>& shape,
                                        const DetectionParameters& parameters) {
  if (!data || shape.size() != 3 || shape[0] != 1 ||
      !((shape[1] == 8 && shape[2] > 0) || (shape[2] == 8 && shape[1] > 0))) {
    throw std::invalid_argument("expected YOLO output [1,8,N] or [1,N,8]");
  }
  const bool channels_first = shape[1] == 8;
  const std::size_t count = static_cast<std::size_t>(channels_first ? shape[2] : shape[1]);
  const auto value = [&](std::size_t row, std::size_t channel) {
    return channels_first ? data[channel * count + row] : data[row * 8 + channel];
  };
  for (std::size_t i = 0; i < count * 8; ++i) {
    if (!std::isfinite(data[i])) throw std::invalid_argument("non-finite YOLO output");
  }

  std::array<std::vector<cv::Rect2d>, 4> boxes;
  std::array<std::vector<float>, 4> scores;
  for (std::size_t row = 0; row < count; ++row) {
    int best_class = 0;
    float best_score = value(row, 4);
    for (int class_id = 1; class_id < 4; ++class_id) {
      const float score = value(row, 4 + class_id);
      if (score > best_score) {
        best_class = class_id;
        best_score = score;
      }
    }
    if (best_score < parameters.confidence[best_class]) continue;
    const float width = value(row, 2);
    const float height = value(row, 3);
    if (width <= 0 || height <= 0) continue;
    boxes[best_class].emplace_back(value(row, 0) - width / 2,
                                   value(row, 1) - height / 2, width, height);
    scores[best_class].push_back(best_score);
  }

  std::vector<Detection> detections;
  for (int class_id = 0; class_id < 4; ++class_id) {
    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes[class_id], scores[class_id],
                      parameters.confidence[class_id], parameters.nms_iou, kept);
    for (const int index : kept) {
      const cv::Rect2d& box = boxes[class_id][index];
      detections.push_back({cv::Rect2f(box), scores[class_id][index], stateForClass(class_id)});
    }
  }
  std::stable_sort(detections.begin(), detections.end(),
                   [](const Detection& a, const Detection& b) {
                     return a.confidence > b.confidence;
                   });
  return detections;
}

DetectionState chooseFrameState(const std::vector<Detection>& detections,
                                float ambiguity_margin) {
  if (detections.empty()) return DetectionState::Unknown;
  const Detection& best = detections.front();
  for (const auto& detection : detections) {
    if (detection.state != best.state &&
        best.confidence - detection.confidence <= ambiguity_margin) {
      return DetectionState::Unknown;
    }
  }
  return best.state;
}

std::uint8_t toRosState(DetectionState state) {
  switch (state) {
    case DetectionState::Go: return interfaces::TrafficLight::GREEN;
    case DetectionState::LeftTurn: return interfaces::TrafficLight::LEFT;
    case DetectionState::PrepareStop: return interfaces::TrafficLight::YELLOW;
    case DetectionState::Stop: return interfaces::TrafficLight::RED;
    case DetectionState::Unknown: return interfaces::TrafficLight::UNKNOWN;
  }
  return interfaces::TrafficLight::UNKNOWN;
}

TemporalVoter::TemporalVoter(std::size_t history_size, std::size_t minimum_votes)
    : history_size_(history_size), minimum_votes_(minimum_votes) {
  if (history_size == 0 || minimum_votes == 0 || minimum_votes > history_size) {
    throw std::invalid_argument("invalid temporal vote configuration");
  }
}

DetectionState TemporalVoter::push(DetectionState state) {
  history_.push_back(state);
  if (history_.size() > history_size_) history_.pop_front();
  std::array<std::size_t, 5> counts{{0, 0, 0, 0, 0}};
  for (const auto item : history_) ++counts[static_cast<std::size_t>(item)];
  const std::size_t maximum = *std::max_element(counts.begin(), counts.end());
  if (maximum < minimum_votes_) return DetectionState::Unknown;
  for (auto iterator = history_.rbegin(); iterator != history_.rend(); ++iterator) {
    if (counts[static_cast<std::size_t>(*iterator)] == maximum) return *iterator;
  }
  return DetectionState::Unknown;
}

}  // namespace traffic_light
