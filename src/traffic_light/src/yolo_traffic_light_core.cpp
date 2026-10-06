#include <traffic_light/cores/yolo_traffic_light_core.hpp>
#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <fstream>
#include <stdexcept>

namespace traffic_light {
namespace {

void validateManifest(const std::string& path) {
  YAML::Node manifest;
  try {
    manifest = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("cannot parse manifest " + path + ": " + error.what());
  }
  const std::array<std::string, 4> expected{{"go", "left_turn", "prepare_stop", "stop"}};
  const YAML::Node classes = manifest["classes"];
  if (!classes.IsSequence() || classes.size() != expected.size()) {
    throw std::invalid_argument("manifest class order must be go,left_turn,prepare_stop,stop");
  }
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (!classes[i].IsScalar() || classes[i].as<std::string>() != expected[i])
      throw std::invalid_argument("manifest class order must be go,left_turn,prepare_stop,stop");
  }
}

}  // namespace

struct YoloTrafficLightCore::Impl {
  explicit Impl(TrafficLightParameters value)
      : parameters(std::move(value)),
        env(ORT_LOGGING_LEVEL_WARNING, "traffic_light"),
        voter(parameters.detection.history_size, parameters.detection.minimum_votes) {
    validateManifest(parameters.manifest_path);
    std::ifstream model(parameters.model_path, std::ios::binary);
    if (!model) throw std::invalid_argument("cannot read model: " + parameters.model_path);

    OrtCUDAProviderOptions cuda_options{};
    try {
      options.AppendExecutionProvider_CUDA(cuda_options);
      session.reset(new Ort::Session(env, parameters.model_path.c_str(), options));
    } catch (const Ort::Exception&) {
      if (!parameters.allow_cpu_fallback) throw;
      options = Ort::SessionOptions{};
      session.reset(new Ort::Session(env, parameters.model_path.c_str(), options));
    }
    if (session->GetInputCount() != 1 || session->GetOutputCount() != 1) {
      throw std::invalid_argument("model must have one input and one output");
    }
    const auto input_type = session->GetInputTypeInfo(0);
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
      throw std::invalid_argument("model input must be float32");
    const auto input_shape = input_info.GetShape();
    if (input_shape != std::vector<std::int64_t>{1, 3, parameters.input_height, parameters.input_width}) {
      throw std::invalid_argument("unexpected model input shape");
    }
    const auto output_type = session->GetOutputTypeInfo(0);
    const auto output_info = output_type.GetTensorTypeAndShapeInfo();
    if (output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
      throw std::invalid_argument("model output must be float32");
    const auto output_shape = output_info.GetShape();
    if (output_shape.size() != 3 || output_shape[0] != 1 ||
        !((output_shape[1] == 8 && output_shape[2] > 0) ||
          (output_shape[2] == 8 && output_shape[1] > 0)))
      throw std::invalid_argument("model output must be [1,8,N] or [1,N,8]");
    input_name = session->GetInputNameAllocated(0, allocator).get();
    output_name = session->GetOutputNameAllocated(0, allocator).get();
  }

  interfaces::TrafficLight unknown() {
    interfaces::TrafficLight output;
    output.state = toRosState(voter.push(DetectionState::Unknown));
    return output;
  }

  interfaces::TrafficLight run(const sensor_msgs::CompressedImage& message) {
    cv::Mat image = cv::imdecode(message.data, cv::IMREAD_COLOR);
    if (image.empty()) return unknown();
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
    output.state = toRosState(voter.push(
        chooseFrameState(detections, parameters.detection.ambiguity_margin)));
    return output;
  }

  TrafficLightParameters parameters;
  Ort::Env env;
  Ort::SessionOptions options;
  std::unique_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::string input_name;
  std::string output_name;
  TemporalVoter voter;
};

YoloTrafficLightCore::YoloTrafficLightCore(TrafficLightParameters parameters)
    : impl_(new Impl(std::move(parameters))) {}
YoloTrafficLightCore::~YoloTrafficLightCore() = default;
YoloTrafficLightCore::YoloTrafficLightCore(YoloTrafficLightCore&&) noexcept = default;
YoloTrafficLightCore& YoloTrafficLightCore::operator=(YoloTrafficLightCore&&) noexcept = default;
interfaces::TrafficLight YoloTrafficLightCore::process(
    const sensor_msgs::CompressedImage& message) {
  return impl_->run(message);
}

}  // namespace traffic_light
