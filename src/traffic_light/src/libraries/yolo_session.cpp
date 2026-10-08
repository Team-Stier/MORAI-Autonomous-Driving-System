#include <traffic_light/libraries/yolo_session.hpp>
#include <ros/console.h>
#include <onnxruntime_cxx_api.h>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace traffic_light {
struct YoloSession::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "traffic_light"};
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::string input_name;
  std::string output_name;
  std::array<std::int64_t, 4> input_shape{};
  std::size_t channels = 0;
  bool channels_first = true;
};

YoloSession::YoloSession(const std::string& model, InferenceDevice device, int input_width, int input_height,
                         std::size_t channels)
    : impl_(new Impl) {
  impl_->input_shape = {{1, 3, input_height, input_width}};
  impl_->channels = channels;
  const char* device_name = device == InferenceDevice::Cuda ? "cuda" : "cpu";
  try {
    Ort::SessionOptions options;
    if (device == InferenceDevice::Cuda) {
      OrtCUDAProviderOptions cuda_options{};
      options.AppendExecutionProvider_CUDA(cuda_options);
    }
    impl_->session.reset(new Ort::Session(impl_->env, model.data(), model.size(), options));
  } catch (const Ort::Exception& error) {
    impl_->session.reset();
    ROS_ERROR("traffic_light: cannot create the %s inference session: %s; publishing UNKNOWN", device_name,
              error.what());
    return;
  }

  Ort::Session& session = *impl_->session;
  if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
    throw std::invalid_argument("traffic_light model must have one input and one output");
  const auto input_type = session.GetInputTypeInfo(0);
  const auto input = input_type.GetTensorTypeAndShapeInfo();
  if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      input.GetShape() != std::vector<std::int64_t>(impl_->input_shape.begin(), impl_->input_shape.end()))
    throw std::invalid_argument("traffic_light model input must be float32 [1,3," + std::to_string(input_height) +
                                "," + std::to_string(input_width) + "]");
  const auto output_type = session.GetOutputTypeInfo(0);
  const auto output = output_type.GetTensorTypeAndShapeInfo();
  const auto shape = output.GetShape();
  const auto expected = static_cast<std::int64_t>(channels);
  const bool first = shape.size() == 3 && shape[0] == 1 && shape[1] == expected && shape[2] > 0;
  const bool last = shape.size() == 3 && shape[0] == 1 && shape[2] == expected && shape[1] > 0;
  if (output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || !(first || last))
    throw std::invalid_argument("traffic_light model output must be float32 [1," + std::to_string(channels) +
                                ",N] or [1,N," + std::to_string(channels) + "] for the configured classes");
  impl_->channels_first = first;
  Ort::AllocatorWithDefaultOptions allocator;
  impl_->input_name = session.GetInputNameAllocated(0, allocator).get();
  impl_->output_name = session.GetOutputNameAllocated(0, allocator).get();
  ROS_INFO("traffic_light: %s inference session ready", device_name);
}

YoloSession::~YoloSession() = default;
YoloSession::YoloSession(YoloSession&&) noexcept = default;
YoloSession& YoloSession::operator=(YoloSession&&) noexcept = default;

bool YoloSession::ready() const { return impl_ && impl_->session; }

YoloOutput YoloSession::run(std::vector<float>& input) {
  YoloOutput result;
  if (!ready()) return result;
  result.channels = impl_->channels;
  result.channels_first = impl_->channels_first;
  try {
    const Ort::Value tensor = Ort::Value::CreateTensor<float>(
        impl_->memory, input.data(), input.size(), impl_->input_shape.data(), impl_->input_shape.size());
    const char* input_names[] = {impl_->input_name.c_str()};
    const char* output_names[] = {impl_->output_name.c_str()};
    auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, input_names, &tensor, 1, output_names, 1);
    const auto info = outputs.front().GetTensorTypeAndShapeInfo();
    const float* data = outputs.front().GetTensorData<float>();
    result.values.assign(data, data + info.GetElementCount());
  } catch (const Ort::Exception& error) {
    result.values.clear();
    ROS_ERROR_THROTTLE(1.0, "traffic_light: inference failed: %s; frame state UNKNOWN", error.what());
  }
  result.count = result.channels ? result.values.size() / result.channels : 0;
  return result;
}
}  // namespace traffic_light
