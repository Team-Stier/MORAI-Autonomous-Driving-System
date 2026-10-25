#include <traffic_light/parameters/camera7_frame_gpu.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace {
std::string varint(std::uint64_t value) {
  std::string out;
  while (value >= 128) {
    out.push_back(static_cast<char>((value & 127) | 128));
    value >>= 7;
  }
  out.push_back(static_cast<char>(value));
  return out;
}
std::string integer(int field, std::uint64_t value) {
  return varint(field << 3) + varint(value);
}
std::string bytes(int field, const std::string& value) {
  return varint((field << 3) | 2) + varint(value.size()) + value;
}
std::string tensor(const std::string& name, const std::vector<int>& shape,
                   const std::vector<float>& values) {
  std::string data;
  for (const int dimension : shape) data += integer(1, dimension);
  data += integer(2, 1) + bytes(8, name);
  data += bytes(9, std::string(reinterpret_cast<const char*>(values.data()),
                              values.size() * sizeof(float)));
  return data;
}
std::string valueInfo(const std::string& name, const std::vector<int>& shape) {
  std::string dimensions;
  for (const int dimension : shape) dimensions += bytes(1, integer(1, dimension));
  return bytes(1, name) + bytes(2, bytes(1, integer(1, 1) + bytes(2, dimensions)));
}
std::string node(const std::string& operation, const std::vector<std::string>& inputs,
                 const std::string& output, const std::string& attributes = {}) {
  std::string result;
  for (const auto& input : inputs) result += bytes(1, input);
  return result + bytes(2, output) + bytes(4, operation) + attributes;
}
std::string model(const std::string& graph) {
  return integer(1, 8) + bytes(2, "camera7-frame-native-fixture") +
         bytes(7, graph) + bytes(8, integer(2, 17));
}
std::string colorModel(bool cpu_only = false) {
  const auto axes = bytes(1, "axes") + integer(8, 2) + integer(8, 3) + integer(20, 7);
  const auto keep = bytes(1, "keepdims") + integer(3, 0) + integer(20, 2);
  std::string graph;
  if (cpu_only) {
    const auto to_string = bytes(1, "to") + integer(3, 8) + integer(20, 2);
    const auto to_float = bytes(1, "to") + integer(3, 1) + integer(20, 2);
    graph += bytes(1, node("Cast", {"image"}, "text", bytes(5, to_string)));
    graph += bytes(1, node("Cast", {"text"}, "pixels", bytes(5, to_float)));
  }
  graph += bytes(1, node("ReduceMean", {cpu_only ? "pixels" : "image"}, "rgb", bytes(5, axes) + bytes(5, keep)));
  graph += bytes(1, node("MatMul", {"rgb", "weights"}, "logits"));
  graph += bytes(1, node("Add", {"logits", "bias"}, "shifted"));
  const auto axis = bytes(1, "axis") + integer(3, 1) + integer(20, 2);
  graph += bytes(1, node("Softmax", {"shifted"}, "state_probabilities", bytes(5, axis)));
  std::vector<float> weights(3 * 17, 0.0F);
  weights[interfaces::TrafficLight::RED] = 12.0F;
  weights[17 + interfaces::TrafficLight::GREEN] = 12.0F;
  weights[34 + interfaces::TrafficLight::LEFT] = 12.0F;
  std::vector<float> bias(17, 0.0F);
  bias[interfaces::TrafficLight::UNKNOWN] = 2.0F;
  graph += bytes(5, tensor("weights", {3, 17}, weights));
  graph += bytes(5, tensor("bias", {17}, bias));
  graph += bytes(2, "whole-frame-color-fixture") + bytes(11, valueInfo("image", {1, 3, 480, 640})) +
           bytes(12, valueInfo("state_probabilities", {1, 17}));
  return model(graph);
}
sensor_msgs::CompressedImage frame(const cv::Scalar& color, std::uint32_t seconds = 10) {
  sensor_msgs::CompressedImage message;
  message.header.stamp = ros::Time(seconds, 0);
  message.header.frame_id = "camera_7_optical_frame";
  message.format = "jpeg";
  cv::imencode(".jpg", cv::Mat(480, 640, CV_8UC3, color), message.data);
  return message;
}
traffic_light::Camera7FrameGpuParameters parameters() {
  traffic_light::Camera7FrameGpuParameters result;
  result.model = colorModel();
  return result;
}
class ModelAsset {
 public:
  explicit ModelAsset(const std::string& model_data = colorModel()) {
    char pattern[] = "/tmp/camera7_frame_fixture_XXXXXX";
    directory_ = mkdtemp(pattern);
    std::ofstream file(directory_ + "/model.onnx", std::ios::binary);
    file << model_data;
  }
  ~ModelAsset() {
    unlink((directory_ + "/model.onnx").c_str());
    rmdir(directory_.c_str());
  }
  node_pipeline::ParamReader config(const std::string& extra = {}) const {
    return node_pipeline::ParamReader::fromString("model: model.onnx\n" + extra, directory_);
  }
 private:
  std::string directory_;
};
}  // namespace

TEST(Camera7FrameGpu, FirstFrameUsesWholeRgbAndLeavesHeaderForPipeline) {
  traffic_light::Camera7FrameGpu core(parameters());
  const auto result = core.process(frame({0, 0, 255}));
  EXPECT_EQ(interfaces::TrafficLight::RED, result.state);
  EXPECT_TRUE(result.header.stamp.isZero());
  EXPECT_TRUE(result.header.frame_id.empty());
  EXPECT_EQ(interfaces::TrafficLight::LEFT, core.process(frame({255, 0, 0})).state);
}

TEST(Camera7FrameGpu, UsesPixelsAcrossTheNativeFullFrame) {
  traffic_light::Camera7FrameGpu core(parameters());
  auto image = frame({0, 0, 0});
  cv::Mat pixels(480, 640, CV_8UC3, cv::Scalar::all(0));
  pixels(cv::Rect(0, 0, 160, 480)).setTo(cv::Scalar(0, 0, 255));
  cv::imencode(".jpg", pixels, image.data);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(image).state);
}

TEST(Camera7FrameGpu, FirstRestrictiveTransitionUsesTheCurrentFrame) {
  traffic_light::Camera7FrameGpu core(parameters());
  EXPECT_EQ(interfaces::TrafficLight::GREEN, core.process(frame({0, 255, 0})).state);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(frame({0, 0, 255}, 11)).state);
}

TEST(Camera7FrameGpu, RejectsCpuOnlyGraphInsteadOfFallingBack) {
  auto value = parameters();
  value.model = colorModel(true);
  EXPECT_THROW(traffic_light::Camera7FrameGpu core(value), Ort::Exception);
}

TEST(Camera7FrameGpu, LoaderAcceptsTheRgbEnumAssetContract) {
  const ModelAsset asset;
  const auto loaded = traffic_light::loadCamera7FrameGpuParameters(
      asset.config("intra_op_threads: 3\nminimum_confidence: 0.8\n"));
  EXPECT_EQ(3, loaded.intra_op_threads);
  EXPECT_FLOAT_EQ(0.8F, loaded.minimum_confidence);
  EXPECT_EQ(colorModel(), loaded.model);
}

TEST(Camera7FrameGpu, LoaderRejectsInvalidStartupParameters) {
  const ModelAsset asset;
  EXPECT_THROW(traffic_light::loadCamera7FrameGpuParameters(
      asset.config("intra_op_threads: 0\n")), std::invalid_argument);
  EXPECT_THROW(traffic_light::loadCamera7FrameGpuParameters(
      asset.config("minimum_confidence: 1.1\n")), std::invalid_argument);
}
