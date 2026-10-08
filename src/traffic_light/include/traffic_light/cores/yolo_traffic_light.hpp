#pragma once

#include <traffic_light/libraries/detection.hpp>
#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/console.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace traffic_light {

struct YoloTrafficLightParameters {
  std::string model;            // ONNX model bytes
  std::string device = "cuda";  // cuda | cpu
  int input_width = 1280;
  int input_height = 1280;
  DetectionParameters detection;
};

class YoloTrafficLight {
 public:
  explicit YoloTrafficLight(YoloTrafficLightParameters parameters);
  ~YoloTrafficLight();
  YoloTrafficLight(YoloTrafficLight&&) noexcept;
  YoloTrafficLight& operator=(YoloTrafficLight&&) noexcept;
  YoloTrafficLight(const YoloTrafficLight&) = delete;
  YoloTrafficLight& operator=(const YoloTrafficLight&) = delete;

  interfaces::TrafficLight process(const sensor_msgs::CompressedImage& message);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct YoloTrafficLight::Impl {
  explicit Impl(YoloTrafficLightParameters value)
      : parameters(std::move(value)),
        env(ORT_LOGGING_LEVEL_WARNING, "traffic_light") {
    try {
      if (parameters.device == "cuda") {
        OrtCUDAProviderOptions cuda_options{};
        options.AppendExecutionProvider_CUDA(cuda_options);
      }
      session.reset(new Ort::Session(env, parameters.model.data(), parameters.model.size(), options));
      const std::string error = modelError();
      if (!error.empty()) {
        ROS_ERROR("yolo_traffic_light session creation failed: %s", error.c_str());
        session.reset();
        return;
      }
      input_name = session->GetInputNameAllocated(0, allocator).get();
      output_name = session->GetOutputNameAllocated(0, allocator).get();
    } catch (const std::exception& error) {
      ROS_ERROR("yolo_traffic_light session creation failed (%s): %s", parameters.device.c_str(), error.what());
      session.reset();
    }
  }

  std::string modelError() const {
    if (session->GetInputCount() != 1 || session->GetOutputCount() != 1) {
      return "model must have one input and one output";
    }
    const auto input_type = session->GetInputTypeInfo(0);
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
      return "model input must be float32";
    const auto input_shape = input_info.GetShape();
    if (input_shape != std::vector<std::int64_t>{1, 3, parameters.input_height, parameters.input_width}) {
      return "unexpected model input shape";
    }
    const auto output_type = session->GetOutputTypeInfo(0);
    const auto output_info = output_type.GetTensorTypeAndShapeInfo();
    if (output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
      return "model output must be float32";
    const auto output_shape = output_info.GetShape();
    if (output_shape.size() != 3 || output_shape[0] != 1 ||
        !((output_shape[1] == 8 && output_shape[2] > 0) ||
          (output_shape[2] == 8 && output_shape[1] > 0)))
      return "model output must be [1,8,N] or [1,N,8]";
    return {};
  }

  // 검출된 등의 집합을 TrafficLight 조합 enum으로 바꾼다. 비트: red=1, yellow=2, green=4, left=8.
  static std::uint8_t combination(const std::vector<Detection>& detections) {
    const std::array<std::uint8_t, 16> states{{
        interfaces::TrafficLight::UNKNOWN, interfaces::TrafficLight::RED,
        interfaces::TrafficLight::YELLOW, interfaces::TrafficLight::RED_YELLOW,
        interfaces::TrafficLight::GREEN, interfaces::TrafficLight::RED_GREEN,
        interfaces::TrafficLight::YELLOW_GREEN, interfaces::TrafficLight::RED_YELLOW_GREEN,
        interfaces::TrafficLight::LEFT, interfaces::TrafficLight::RED_LEFT,
        interfaces::TrafficLight::YELLOW_LEFT, interfaces::TrafficLight::RED_YELLOW_LEFT,
        interfaces::TrafficLight::GREEN_LEFT, interfaces::TrafficLight::RED_GREEN_LEFT,
        interfaces::TrafficLight::YELLOW_GREEN_LEFT, interfaces::TrafficLight::RED_YELLOW_GREEN_LEFT}};
    std::size_t lamps = 0;
    for (const auto& detection : detections) {
      switch (detection.state) {
        case DetectionState::Stop: lamps |= 1U; break;
        case DetectionState::PrepareStop: lamps |= 2U; break;
        case DetectionState::Go: lamps |= 4U; break;
        case DetectionState::LeftTurn: lamps |= 8U; break;
        case DetectionState::Unknown: break;
      }
    }
    return states[lamps];
  }

  // TemporalVoter::push와 같은 다수결을 조합 enum 값에 적용한다.
  std::uint8_t vote(std::uint8_t state) {
    history.push_back(state);
    if (history.size() > parameters.detection.history_size) history.pop_front();
    std::array<std::size_t, interfaces::TrafficLight::RED_YELLOW_GREEN_LEFT + 1> counts{};
    for (const auto item : history) ++counts[item];
    const std::size_t maximum = *std::max_element(counts.begin(), counts.end());
    if (maximum < parameters.detection.minimum_votes) return interfaces::TrafficLight::UNKNOWN;
    for (auto iterator = history.rbegin(); iterator != history.rend(); ++iterator) {
      if (counts[*iterator] == maximum) return *iterator;
    }
    return interfaces::TrafficLight::UNKNOWN;
  }

  interfaces::TrafficLight run(const sensor_msgs::CompressedImage& message) {
    interfaces::TrafficLight output;
    output.state = interfaces::TrafficLight::UNKNOWN;
    if (!session) return output;
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
    output.state = vote(combination(detections));
    return output;
  }

  YoloTrafficLightParameters parameters;
  Ort::Env env;
  Ort::SessionOptions options;
  std::unique_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::string input_name;
  std::string output_name;
  std::deque<std::uint8_t> history;
};

inline YoloTrafficLight::YoloTrafficLight(YoloTrafficLightParameters parameters)
    : impl_(new Impl(std::move(parameters))) {}
inline YoloTrafficLight::~YoloTrafficLight() = default;
inline YoloTrafficLight::YoloTrafficLight(YoloTrafficLight&&) noexcept = default;
inline YoloTrafficLight& YoloTrafficLight::operator=(YoloTrafficLight&&) noexcept = default;
inline interfaces::TrafficLight YoloTrafficLight::process(
    const sensor_msgs::CompressedImage& message) {
  return impl_->run(message);
}

}  // namespace traffic_light
