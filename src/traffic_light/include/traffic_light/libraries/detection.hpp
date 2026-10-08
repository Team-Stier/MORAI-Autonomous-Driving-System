#pragma once

#include <traffic_light/libraries/types.hpp>
#include <opencv2/core.hpp>
#include <cstdint>
#include <deque>
#include <vector>

namespace traffic_light {

struct Detection {
  cv::Rect2f box;
  float confidence;
  DetectionState state;
};

std::vector<Detection> decodeYoloOutput(const float* data,
                                        const std::vector<std::int64_t>& shape,
                                        const DetectionParameters& parameters);
DetectionState chooseFrameState(const std::vector<Detection>& detections,
                                float ambiguity_margin);
std::uint8_t toRosState(DetectionState state);

class TemporalVoter {
 public:
  TemporalVoter(std::size_t history_size, std::size_t minimum_votes);
  DetectionState push(DetectionState state);

 private:
  std::size_t history_size_;
  std::size_t minimum_votes_;
  std::deque<DetectionState> history_;
};

}  // namespace traffic_light
