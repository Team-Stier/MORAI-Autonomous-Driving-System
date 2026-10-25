#pragma once

#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

namespace traffic_light {

struct Camera7FrameGpuParameters {
  // image: RGB float32 [1,3,480,640], 0..1, full camera7 frame.
  // state_probabilities: float32 [1,17], softmax in TrafficLight enum order.
  std::string model;
  int intra_op_threads = 2;
  float minimum_confidence = 0.0F;
};

class Camera7FrameGpu {
 public:
  explicit Camera7FrameGpu(Camera7FrameGpuParameters value)
      : parameters_(std::move(value)), env_(ORT_LOGGING_LEVEL_WARNING, "camera7_frame_gpu") {
    options_.SetIntraOpNumThreads(parameters_.intra_op_threads);
    options_.SetInterOpNumThreads(1);
    options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    env_.RegisterExecutionProviderLibrary("camera7_webgpu", "libonnxruntime_providers_webgpu.so");
    std::vector<Ort::ConstEpDevice> devices;
    for (auto device : env_.GetEpDevices()) {
      if (std::strcmp(device.EpName(), "WebGpuExecutionProvider") == 0 &&
          device.Device().Type() == OrtHardwareDeviceType_GPU && devices.empty())
        devices.push_back(device);
    }
    options_.AppendExecutionProvider_V2(env_, devices,
        std::unordered_map<std::string, std::string>{{"adapterIndex", "0"},
          {"preferredLayout", "NCHW"}, {"powerPreference", "high-performance"}});
    options_.AddConfigEntry("session.disable_cpu_ep_fallback", "1");
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
  Camera7FrameGpuParameters parameters_;
  Ort::Env env_;
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> model_;
  Ort::MemoryInfo memory_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
};

}  // namespace traffic_light
