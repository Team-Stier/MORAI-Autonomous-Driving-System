#pragma once

#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace traffic_light {

struct YoloTrafficLightParameters {
  struct DetectionParameters {
    std::array<float, 4> confidence{{0.4F, 0.4F, 0.35F, 0.4F}};
    float nms_iou = 0.45F;
    float ambiguity_margin = 0.05F;
    std::size_t history_size = 5;
    std::size_t minimum_votes = 3;
  };

  std::string model;
  int input_width = 1280;
  int input_height = 1280;
  DetectionParameters detection;
};

class YoloTrafficLight {
 public:
  explicit YoloTrafficLight(YoloTrafficLightParameters value)
      : parameters(std::move(value)), env(ORT_LOGGING_LEVEL_WARNING, "traffic_light") {
    OrtCUDAProviderOptions cuda_options{};
    options.AppendExecutionProvider_CUDA(cuda_options);
    session.reset(new Ort::Session(env, parameters.model.data(), parameters.model.size(), options));
    input_name = session->GetInputNameAllocated(0, allocator).get();
    output_name = session->GetOutputNameAllocated(0, allocator).get();
  }

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
    const std::array<std::int64_t, 4> input_shape{{
        1, 3, parameters.input_height, parameters.input_width}};
    auto tensor = Ort::Value::CreateTensor<float>(
        memory, blob.ptr<float>(), blob.total(), input_shape.data(), input_shape.size());
    const char* input_names[] = {input_name.c_str()};
    const char* output_names[] = {output_name.c_str()};
    auto outputs = session->Run(Ort::RunOptions{nullptr}, input_names, &tensor, 1,
                                output_names, 1);
    const auto shape = outputs.front().GetTensorTypeAndShapeInfo().GetShape();
    const auto detections = decodeYoloOutput(outputs.front().GetTensorData<float>(), shape,
                                             parameters.detection);
    interfaces::TrafficLight output;
    output.state = toRosState(pushState(
        chooseFrameState(detections, parameters.detection.ambiguity_margin)));
    return output;
  }

 private:
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
      const YoloTrafficLightParameters::DetectionParameters& parameters) {
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
    std::array<std::size_t, 5> counts{{0, 0, 0, 0, 0}};
    for (const auto item : history) ++counts[static_cast<std::size_t>(item)];
    const std::size_t maximum = *std::max_element(counts.begin(), counts.end());
    if (maximum < parameters.detection.minimum_votes) return DetectionState::Unknown;
    for (auto iterator = history.rbegin(); iterator != history.rend(); ++iterator) {
      if (counts[static_cast<std::size_t>(*iterator)] == maximum) return *iterator;
    }
    return DetectionState::Unknown;
  }

  YoloTrafficLightParameters parameters;
  Ort::Env env;
  Ort::SessionOptions options;
  std::unique_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::string input_name;
  std::string output_name;
  std::deque<DetectionState> history;
};

}  // namespace traffic_light
