#pragma once

#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace traffic_light {

struct Camera7FrameCpuParameters {
  // image: RGB float32 [1,3,480,640], 0..1, full camera7 frame.
  // state_probabilities: float32 [1,17], softmax in TrafficLight enum order.
  std::string model;
  int intra_op_threads = 2;
  float minimum_confidence = 0.0F;
};

class Camera7FrameCpu {
 public:
  explicit Camera7FrameCpu(Camera7FrameCpuParameters value)
      : parameters_(std::move(value)), env_(ORT_LOGGING_LEVEL_WARNING, "camera7_frame_cpu") {
    options_.SetIntraOpNumThreads(parameters_.intra_op_threads);
    options_.SetInterOpNumThreads(1);
    options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    model_.reset(new Ort::Session(env_, parameters_.model.data(), parameters_.model.size(), options_));
  }

  interfaces::TrafficLight process(const sensor_msgs::CompressedImage& message) {
    const cv::Mat image = cv::imdecode(message.data, cv::IMREAD_COLOR);
    cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0, cv::Size(640, 480),
                                         cv::Scalar(), true, false, CV_32F);
    const std::array<std::int64_t, 4> shape{{1, 3, 480, 640}};
    auto input = Ort::Value::CreateTensor<float>(memory_, blob.ptr<float>(), blob.total(),
                                                shape.data(), shape.size());
    const char* input_names[] = {"image"};
    const char* output_names[] = {"state_probabilities"};
    auto outputs = model_->Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
    const float* probabilities = outputs.front().GetTensorData<float>();
    const float* chosen = std::max_element(probabilities, probabilities + 17);
    interfaces::TrafficLight result;
    result.state = *chosen < parameters_.minimum_confidence
        ? static_cast<std::uint8_t>(interfaces::TrafficLight::UNKNOWN)
        : static_cast<std::uint8_t>(chosen - probabilities);
    return result;
  }

 private:
  Camera7FrameCpuParameters parameters_;
  Ort::Env env_;
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> model_;
  Ort::MemoryInfo memory_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
};

}  // namespace traffic_light
