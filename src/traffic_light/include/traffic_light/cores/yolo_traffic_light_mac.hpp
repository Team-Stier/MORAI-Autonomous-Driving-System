#pragma once

#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <curl/curl.h>
#include <ros/console.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace traffic_light {

struct YoloTrafficLightMacParameters {
  struct DetectionParameters {
    std::array<float, 4> confidence{{0.4F, 0.4F, 0.35F, 0.4F}};
    float nms_iou = 0.45F;
    float ambiguity_margin = 0.05F;
    std::size_t history_size = 5;
    std::size_t minimum_votes = 3;
  };

  std::string inference_url = "http://127.0.0.1:18765/infer";
  long timeout_ms = 1000;
  int input_width = 1280;
  int input_height = 1280;
  DetectionParameters detection;
};

class YoloTrafficLightMac {
 public:
  explicit YoloTrafficLightMac(YoloTrafficLightMacParameters value)
      : parameters(std::move(value)) {}

  interfaces::TrafficLight process(const sensor_msgs::CompressedImage& message) {
    cv::Mat image = cv::imdecode(message.data, cv::IMREAD_COLOR);
    const float scale = std::min(parameters.input_width / static_cast<float>(image.cols),
                                 parameters.input_height / static_cast<float>(image.rows));
    const int resized_width = static_cast<int>(std::round(image.cols * scale));
    const int resized_height = static_cast<int>(std::round(image.rows * scale));
    cv::Mat resized;
    cv::resize(image, resized, {resized_width, resized_height});
    const int left = (parameters.input_width - resized_width) / 2;
    const int right = parameters.input_width - resized_width - left;
    const int top = (parameters.input_height - resized_height) / 2;
    const int bottom = parameters.input_height - resized_height - top;
    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    cv::Mat blob = cv::dnn::blobFromImage(padded, 1.0 / 255.0,
                                          {parameters.input_width, parameters.input_height},
                                          cv::Scalar(), true, false, CV_32F);
    std::vector<unsigned char> response;
    const auto result = infer(blob, response);
    std::uint32_t channels = 0, count = 0;
    if (response.size() >= 16) {
      std::memcpy(&channels, response.data() + 8, 4);
      std::memcpy(&count, response.data() + 12, 4);
    }
    if (result != CURLE_OK || response.size() < 16 ||
        std::memcmp(response.data(), "MORAIGPU", 8) != 0 || channels != 8 ||
        count == 0 || count > 33600 || response.size() != 16 + 8u * count * sizeof(float)) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Mac GPU inference unavailable or invalid: " << curl_easy_strerror(result));
      interfaces::TrafficLight output;
      output.state = toRosState(pushState(DetectionState::Unknown));
      return output;
    }
    std::vector<float> tensor(8u * count);
    std::memcpy(tensor.data(), response.data() + 16, tensor.size() * sizeof(float));
    const auto detections = decodeYoloOutput(tensor.data(), {1, 8, count}, parameters.detection);
    interfaces::TrafficLight output;
    output.state = toRosState(pushState(
        chooseFrameState(detections, parameters.detection.ambiguity_margin)));
    return output;
  }

 private:
  static std::size_t receive(void* data, std::size_t size, std::size_t count, void* target) {
    auto& response = *static_cast<std::vector<unsigned char>*>(target);
    const std::size_t bytes = size * count;
    if (bytes > 1075216u - response.size()) return 0;
    try {
      const auto* begin = static_cast<const unsigned char*>(data);
      response.insert(response.end(), begin, begin + bytes);
      return bytes;
    } catch (...) {
      return 0;
    }
  }

  CURLcode infer(const cv::Mat& blob, std::vector<unsigned char>& response) {
    if (!request) return CURLE_FAILED_INIT;
    curl_easy_reset(request.get());
    curl_slist* entries = curl_slist_append(nullptr, "Content-Type: application/octet-stream");
    entries = curl_slist_append(entries, "Expect:");
    const std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)> headers(entries, curl_slist_free_all);
    curl_easy_setopt(request.get(), CURLOPT_URL, parameters.inference_url.c_str());
    curl_easy_setopt(request.get(), CURLOPT_HTTPHEADER, headers.get());
    curl_easy_setopt(request.get(), CURLOPT_POSTFIELDS, blob.ptr<float>());
    curl_easy_setopt(request.get(), CURLOPT_POSTFIELDSIZE_LARGE,
                     static_cast<curl_off_t>(blob.total() * sizeof(float)));
    curl_easy_setopt(request.get(), CURLOPT_TIMEOUT_MS, parameters.timeout_ms);
    curl_easy_setopt(request.get(), CURLOPT_CONNECTTIMEOUT_MS, parameters.timeout_ms);
    curl_easy_setopt(request.get(), CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(request.get(), CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(request.get(), CURLOPT_WRITEFUNCTION, &YoloTrafficLightMac::receive);
    curl_easy_setopt(request.get(), CURLOPT_WRITEDATA, &response);
    return curl_easy_perform(request.get());
  }

  enum class DetectionState : std::uint8_t { Unknown, Go, LeftTurn, PrepareStop, Stop };

  struct Detection {
    cv::Rect2f box;
    float confidence;
    DetectionState state;
  };

  static DetectionState stateForClass(int class_id) {
    static const std::array<DetectionState, 4> states{{
        DetectionState::Go, DetectionState::LeftTurn,
        DetectionState::PrepareStop, DetectionState::Stop}};
    return states[static_cast<std::size_t>(class_id)];
  }

  static std::vector<Detection> decodeYoloOutput(
      const float* data, const std::vector<std::int64_t>& shape,
      const YoloTrafficLightMacParameters::DetectionParameters& parameters) {
    const bool channels_first = shape[1] == 8;
    const std::size_t count = static_cast<std::size_t>(channels_first ? shape[2] : shape[1]);
    const auto value = [&](std::size_t row, std::size_t channel) {
      return channels_first ? data[channel * count + row] : data[row * 8 + channel];
    };

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

  static DetectionState chooseFrameState(const std::vector<Detection>& detections,
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

  static std::uint8_t toRosState(DetectionState state) {
    switch (state) {
      case DetectionState::Go: return interfaces::TrafficLight::GREEN;
      case DetectionState::LeftTurn: return interfaces::TrafficLight::LEFT;
      case DetectionState::PrepareStop: return interfaces::TrafficLight::YELLOW;
      case DetectionState::Stop: return interfaces::TrafficLight::RED;
      case DetectionState::Unknown: return interfaces::TrafficLight::UNKNOWN;
    }
    return interfaces::TrafficLight::UNKNOWN;
  }

  DetectionState pushState(DetectionState state) {
    history.push_back(state);
    if (history.size() > parameters.detection.history_size) history.pop_front();
    if (state == DetectionState::Unknown) return DetectionState::Unknown;
    std::array<std::size_t, 5> counts{{0, 0, 0, 0, 0}};
    for (const auto item : history) ++counts[static_cast<std::size_t>(item)];
    const std::size_t maximum = *std::max_element(counts.begin(), counts.end());
    if (maximum < parameters.detection.minimum_votes) return DetectionState::Unknown;
    for (auto iterator = history.rbegin(); iterator != history.rend(); ++iterator) {
      if (counts[static_cast<std::size_t>(*iterator)] == maximum) return *iterator;
    }
    return DetectionState::Unknown;
  }

  YoloTrafficLightMacParameters parameters;
  std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> request{curl_easy_init(), curl_easy_cleanup};
  std::deque<DetectionState> history;
};

}  // namespace traffic_light
