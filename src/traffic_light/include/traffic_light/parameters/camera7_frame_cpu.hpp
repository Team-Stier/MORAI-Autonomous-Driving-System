#pragma once
#include <traffic_light/cores/camera7_frame_cpu.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace traffic_light {
inline Camera7FrameCpuParameters loadCamera7FrameCpuParameters(const node_pipeline::ParamReader& params) {
  Camera7FrameCpuParameters value;
  if (params.has("intra_op_threads")) {
    const auto threads = params.integer("intra_op_threads");
    if (threads < 1 || threads > 12)
      throw std::invalid_argument("intra_op_threads must be in [1,12]");
    value.intra_op_threads = static_cast<int>(threads);
  }
  const double confidence = params.has("minimum_confidence")
      ? params.number("minimum_confidence") : value.minimum_confidence;
  if (!std::isfinite(confidence) || confidence < 0.0 || confidence > 1.0)
    throw std::invalid_argument("minimum_confidence must be finite and in [0,1]");
  value.minimum_confidence = static_cast<float>(confidence);
  value.model = params.asset("model");

  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "camera7_frame_assets");
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  Ort::Session session(env, value.model.data(), value.model.size(), options);
  if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
    throw std::invalid_argument("camera7 frame model must have one input and one output");
  const auto input_type = session.GetInputTypeInfo(0);
  const auto output_type = session.GetOutputTypeInfo(0);
  if (input_type.GetONNXType() != ONNX_TYPE_TENSOR || output_type.GetONNXType() != ONNX_TYPE_TENSOR)
    throw std::invalid_argument("camera7 frame model input/output must be tensors");
  const auto input = input_type.GetTensorTypeAndShapeInfo();
  const auto output = output_type.GetTensorTypeAndShapeInfo();
  if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      input.GetShape() != std::vector<std::int64_t>({1, 3, 480, 640}) ||
      output.GetShape() != std::vector<std::int64_t>({1, 17}))
    throw std::invalid_argument("camera7 frame model requires RGB float32 [1,3,480,640] to [1,17]");
  Ort::AllocatorWithDefaultOptions allocator;
  const std::string input_name = session.GetInputNameAllocated(0, allocator).get();
  const std::string output_name = session.GetOutputNameAllocated(0, allocator).get();
  if (input_name != "image" || output_name != "state_probabilities")
    throw std::invalid_argument("camera7 frame model tensor names must be image and state_probabilities");
  return value;
}
}  // namespace traffic_light
