#include <traffic_light/parameters/camera7_frame_cpu.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <gtest/gtest.h>
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
std::string constantModel(const std::vector<float>& probabilities,
                          const std::vector<int>& input_shape = {1, 3, 480, 640},
                          const std::string& input_name = "image") {
  const std::vector<int> output_shape{1, static_cast<int>(probabilities.size())};
  const auto attribute = bytes(1, "value") +
      bytes(5, tensor("constant", output_shape, probabilities)) + integer(20, 4);
  return model(bytes(1, node("Constant", {}, "state_probabilities", bytes(5, attribute))) +
               bytes(2, "constant-state-fixture") + bytes(11, valueInfo(input_name, input_shape)) +
               bytes(12, valueInfo("state_probabilities", output_shape)));
}
std::string colorModel() {
  const auto axes = bytes(1, "axes") + integer(8, 2) + integer(8, 3) + integer(20, 7);
  const auto keep = bytes(1, "keepdims") + integer(3, 0) + integer(20, 2);
  std::string graph = bytes(1, node("ReduceMean", {"image"}, "rgb", bytes(5, axes) + bytes(5, keep)));
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
traffic_light::Camera7FrameCpuParameters parameters() {
  traffic_light::Camera7FrameCpuParameters result;
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

TEST(Camera7FrameCpu, FirstFrameUsesWholeImageRgbAndLeavesHeaderForPipeline) {
  traffic_light::Camera7FrameCpu core(parameters());
  const auto result = core.process(frame({0, 0, 255}));
  EXPECT_EQ(interfaces::TrafficLight::RED, result.state);
  EXPECT_TRUE(result.header.stamp.isZero());
  EXPECT_TRUE(result.header.frame_id.empty());
  EXPECT_EQ(interfaces::TrafficLight::LEFT, core.process(frame({255, 0, 0}, 11)).state);
}

TEST(Camera7FrameCpu, UsesPixelsAcrossTheNativeFullFrame) {
  traffic_light::Camera7FrameCpu core(parameters());
  sensor_msgs::CompressedImage image;
  image.header.stamp = ros::Time(10);
  image.format = "jpeg";
  cv::Mat pixels(480, 640, CV_8UC3, cv::Scalar::all(0));
  pixels(cv::Rect(0, 0, 160, 480)).setTo(cv::Scalar(0, 0, 255));
  cv::imencode(".jpg", pixels, image.data);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(image).state);
}

TEST(Camera7FrameCpu, DecodableOtherImageSizesProduceTheFixedWholeFrameTensor) {
  traffic_light::Camera7FrameCpu core(parameters());
  for (const auto& size : {cv::Size(320, 240), cv::Size(1280, 960)}) {
    auto image = frame({0, 0, 255});
    cv::imencode(".jpg", cv::Mat(size, CV_8UC3, cv::Scalar(0, 0, 255)), image.data);
    EXPECT_EQ(interfaces::TrafficLight::RED, core.process(image).state);
  }
}

TEST(Camera7FrameCpu, FirstRestrictiveTransitionDoesNotRetainPreviousVotes) {
  traffic_light::Camera7FrameCpu core(parameters());
  EXPECT_EQ(interfaces::TrafficLight::GREEN, core.process(frame({0, 255, 0})).state);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(frame({0, 0, 255}, 11)).state);
}

TEST(Camera7FrameCpu, ArgmaxIndicesMatchEveryTrafficLightEnumIncludingUnknownAndOff) {
  const std::array<std::uint8_t, 17> expected{{
      interfaces::TrafficLight::UNKNOWN, interfaces::TrafficLight::OFF,
      interfaces::TrafficLight::RED, interfaces::TrafficLight::YELLOW,
      interfaces::TrafficLight::GREEN, interfaces::TrafficLight::LEFT,
      interfaces::TrafficLight::RED_YELLOW, interfaces::TrafficLight::RED_GREEN,
      interfaces::TrafficLight::RED_LEFT, interfaces::TrafficLight::YELLOW_GREEN,
      interfaces::TrafficLight::YELLOW_LEFT, interfaces::TrafficLight::GREEN_LEFT,
      interfaces::TrafficLight::RED_YELLOW_GREEN, interfaces::TrafficLight::RED_YELLOW_LEFT,
      interfaces::TrafficLight::RED_GREEN_LEFT, interfaces::TrafficLight::YELLOW_GREEN_LEFT,
      interfaces::TrafficLight::RED_YELLOW_GREEN_LEFT}};
  const auto image = frame({0, 0, 0});
  for (std::size_t state = 0; state < expected.size(); ++state) {
    auto value = parameters();
    std::vector<float> probabilities(17, 0.00625F);
    probabilities[state] = 0.9F;
    value.model = constantModel(probabilities);
    traffic_light::Camera7FrameCpu core(value);
    EXPECT_EQ(expected[state], core.process(image).state) << "state " << state;
  }
}

TEST(Camera7FrameCpu, OptionalConfidenceThresholdUsesUnknownForWeakClass) {
  auto value = parameters();
  std::vector<float> probabilities(17, 0.0375F);
  probabilities[interfaces::TrafficLight::RED] = 0.4F;
  value.model = constantModel(probabilities);
  traffic_light::Camera7FrameCpu immediate(value);
  EXPECT_EQ(interfaces::TrafficLight::RED, immediate.process(frame({0, 0, 0})).state);
  value.minimum_confidence = 0.5F;
  traffic_light::Camera7FrameCpu cautious(value);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, cautious.process(frame({0, 0, 0})).state);
}

TEST(Camera7FrameCpu, LoaderUsesDeclaredCpuThreadsAndConfidence) {
  const ModelAsset asset;
  const auto loaded = traffic_light::loadCamera7FrameCpuParameters(
      asset.config("intra_op_threads: 3\nminimum_confidence: 0.8\n"));
  EXPECT_EQ(3, loaded.intra_op_threads);
  EXPECT_FLOAT_EQ(0.8F, loaded.minimum_confidence);
  traffic_light::Camera7FrameCpu core(loaded);
  EXPECT_EQ(interfaces::TrafficLight::GREEN, core.process(frame({0, 255, 0})).state);
}

TEST(Camera7FrameCpu, LoaderRejectsInvalidStartupParameters) {
  const ModelAsset asset;
  for (const std::string& extra : {"intra_op_threads: 0\n", "minimum_confidence: 1.1\n",
                                 "minimum_confidence: .nan\n"}) {
    EXPECT_THROW(traffic_light::loadCamera7FrameCpuParameters(asset.config(extra)), std::invalid_argument);
  }
}

TEST(Camera7FrameCpu, LoaderRejectsIncompatibleStaticTensorShapeAndTrainingNames) {
  const std::vector<float> probabilities(17, 1.0F / 17.0F);
  const ModelAsset wrong_input(constantModel(probabilities, {1, 3, 640, 640}));
  EXPECT_THROW(traffic_light::loadCamera7FrameCpuParameters(wrong_input.config()), std::invalid_argument);
  const ModelAsset wrong_output(constantModel(std::vector<float>(16, 1.0F / 16.0F)));
  EXPECT_THROW(traffic_light::loadCamera7FrameCpuParameters(wrong_output.config()), std::invalid_argument);
  const ModelAsset wrong_name(constantModel(probabilities, {1, 3, 480, 640}, "images"));
  EXPECT_THROW(traffic_light::loadCamera7FrameCpuParameters(wrong_name.config()), std::invalid_argument);
}
