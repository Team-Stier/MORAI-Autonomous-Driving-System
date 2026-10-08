#include <traffic_light/cores/yolo_traffic_light.hpp>
#include <traffic_light/parameters/yolo_traffic_light.hpp>
#include <traffic_light/validators/decodable_image.hpp>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace pkg = traffic_light;
using Light = interfaces::TrafficLight;

namespace {
const unsigned kRed = pkg::lampBit(pkg::Lamp::Red);
const unsigned kYellow = pkg::lampBit(pkg::Lamp::Yellow);
const unsigned kGreen = pkg::lampBit(pkg::Lamp::Green);
const unsigned kLeft = pkg::lampBit(pkg::Lamp::Left);

std::vector<pkg::LampClass> fourClasses() {
  return {{"go", pkg::Lamp::Green, 0.40F}, {"left_turn", pkg::Lamp::Left, 0.40F},
          {"prepare_stop", pkg::Lamp::Yellow, 0.35F}, {"stop", pkg::Lamp::Red, 0.40F}};
}

pkg::YoloOutput channelsFirst(std::size_t count) {
  pkg::YoloOutput output;
  output.values.assign(8 * count, 0.0F);
  output.count = count;
  output.channels = 8;
  output.channels_first = true;
  return output;
}

void candidate(pkg::YoloOutput& output, std::size_t row, float cx, float cy, float w, float h,
               std::size_t class_index, float score) {
  const float values[] = {cx, cy, w, h};
  for (std::size_t channel = 0; channel < 4; ++channel) output.values[channel * output.count + row] = values[channel];
  output.values[(4 + class_index) * output.count + row] = score;
}

std::string config(const std::string& model, const std::string& device, int history_size, int minimum_votes) {
  return "model: " + model + "\n" + (device.empty() ? "" : "device: " + device + "\n") +
         "input_width: 1280\ninput_height: 1280\n"
         "classes:\n"
         "  \"0\": {name: go, lamp: green, confidence: 0.40}\n"
         "  \"1\": {name: left_turn, lamp: left, confidence: 0.40}\n"
         "  \"2\": {name: prepare_stop, lamp: yellow, confidence: 0.35}\n"
         "  \"3\": {name: stop, lamp: red, confidence: 0.40}\n"
         "nms_iou: 0.45\n"
         "history_size: " + std::to_string(history_size) + "\nminimum_votes: " + std::to_string(minimum_votes) + "\n";
}

pkg::YoloTrafficLightParameters load(const std::string& yaml) {
  return pkg::loadYoloTrafficLightParameters(node_pipeline::ParamReader::fromString(yaml, NP_PACKAGE_DIR));
}

pkg::YoloTrafficLightParameters fixture(const std::string& device, int history_size = 1, int minimum_votes = 1) {
  return load(config("test/data/yolo_green_left.onnx", device, history_size, minimum_votes));
}

sensor_msgs::CompressedImage compressed(const cv::Mat& image, const std::string& extension) {
  sensor_msgs::CompressedImage message;
  message.header.stamp.fromNSec(123456789);
  message.header.frame_id = "camera_7_optical_frame";
  message.format = extension.substr(1);
  cv::imencode(extension, image, message.data);
  return message;
}

sensor_msgs::CompressedImage blankJpeg() { return compressed(cv::Mat(480, 640, CV_8UC3, cv::Scalar(40, 40, 40)), ".jpg"); }

std::vector<node_pipeline::Kind> validate(const sensor_msgs::CompressedImage& image) {
  std::vector<node_pipeline::Kind> issues;
  pkg::DecodableImage()(image, node_pipeline::HealthLimits{}, issues);
  return issues;
}
}  // namespace

TEST(LampCombination, MapsEveryLampSetToTheCombinationEnum) {
  const std::vector<std::pair<unsigned, std::uint8_t>> expected{
      {0, Light::UNKNOWN}, {kRed, Light::RED}, {kYellow, Light::YELLOW}, {kGreen, Light::GREEN},
      {kLeft, Light::LEFT}, {kRed | kYellow, Light::RED_YELLOW}, {kRed | kGreen, Light::RED_GREEN},
      {kRed | kLeft, Light::RED_LEFT}, {kYellow | kGreen, Light::YELLOW_GREEN},
      {kYellow | kLeft, Light::YELLOW_LEFT}, {kGreen | kLeft, Light::GREEN_LEFT},
      {kRed | kYellow | kGreen, Light::RED_YELLOW_GREEN}, {kRed | kYellow | kLeft, Light::RED_YELLOW_LEFT},
      {kRed | kGreen | kLeft, Light::RED_GREEN_LEFT}, {kYellow | kGreen | kLeft, Light::YELLOW_GREEN_LEFT},
      {kRed | kYellow | kGreen | kLeft, Light::RED_YELLOW_GREEN_LEFT}};
  ASSERT_EQ(16u, expected.size());
  for (const auto& entry : expected) {
    EXPECT_EQ(entry.second, pkg::combinationState(entry.first)) << "lamps " << entry.first;
    EXPECT_NE(Light::OFF, pkg::combinationState(entry.first));
  }
}

TEST(LampCombination, UsesTheUnionOfDetectedClasses) {
  const auto classes = fourClasses();
  const std::vector<pkg::Detection> green_left{{{0, 0, 1, 1}, 0.9F, 0}, {{5, 0, 1, 1}, 0.8F, 1}, {{9, 0, 1, 1}, 0.7F, 0}};
  EXPECT_EQ(Light::GREEN_LEFT, pkg::combinationState(pkg::lampMask(green_left, classes)));
  const std::vector<pkg::Detection> red{{{0, 0, 1, 1}, 0.9F, 3}};
  EXPECT_EQ(Light::RED, pkg::combinationState(pkg::lampMask(red, classes)));
  EXPECT_EQ(Light::UNKNOWN, pkg::combinationState(pkg::lampMask({}, classes)));
}

TEST(YoloDecode, DecodesChannelsFirstTensor) {
  auto output = channelsFirst(2);
  candidate(output, 0, 100, 200, 20, 40, 0, 0.9F);
  candidate(output, 1, 300, 400, 60, 80, 3, 0.8F);
  const auto detections = pkg::decodeYoloOutput(output, fourClasses(), 0.45F);
  ASSERT_EQ(2u, detections.size());
  EXPECT_EQ(0u, detections[0].class_index);
  EXPECT_FLOAT_EQ(0.9F, detections[0].confidence);
  EXPECT_FLOAT_EQ(90, detections[0].box.x);
  EXPECT_FLOAT_EQ(180, detections[0].box.y);
  EXPECT_FLOAT_EQ(20, detections[0].box.width);
  EXPECT_FLOAT_EQ(40, detections[0].box.height);
  EXPECT_EQ(3u, detections[1].class_index);
}

TEST(YoloDecode, DecodesRowsFirstTensor) {
  pkg::YoloOutput output;
  output.values = {100, 200, 20, 40, 0, 0.85F, 0, 0};
  output.count = 1;
  output.channels = 8;
  output.channels_first = false;
  const auto detections = pkg::decodeYoloOutput(output, fourClasses(), 0.45F);
  ASSERT_EQ(1u, detections.size());
  EXPECT_EQ(1u, detections[0].class_index);
}

TEST(YoloDecode, AppliesPerClassConfidence) {
  auto output = channelsFirst(2);
  candidate(output, 0, 100, 100, 20, 20, 0, 0.39F);
  candidate(output, 1, 200, 200, 20, 20, 2, 0.36F);
  const auto detections = pkg::decodeYoloOutput(output, fourClasses(), 0.45F);
  ASSERT_EQ(1u, detections.size());
  EXPECT_EQ(2u, detections[0].class_index);
}

TEST(YoloDecode, SuppressesOverlapWithinAClassOnly) {
  auto output = channelsFirst(3);
  candidate(output, 0, 100, 100, 40, 40, 0, 0.9F);
  candidate(output, 1, 101, 101, 40, 40, 0, 0.8F);
  candidate(output, 2, 101, 101, 40, 40, 3, 0.7F);
  const auto detections = pkg::decodeYoloOutput(output, fourClasses(), 0.45F);
  ASSERT_EQ(2u, detections.size());
  EXPECT_FLOAT_EQ(0.9F, detections[0].confidence);
  EXPECT_EQ(3u, detections[1].class_index);
}

TEST(YoloDecode, SkipsNonFiniteAndEmptyBoxes) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto output = channelsFirst(3);
  candidate(output, 0, 100, 100, 20, 20, 0, nan);
  candidate(output, 1, 200, 200, 0, 20, 1, 0.9F);
  candidate(output, 2, nan, 300, 20, 20, 3, 0.9F);
  EXPECT_TRUE(pkg::decodeYoloOutput(output, fourClasses(), 0.45F).empty());
}

TEST(YoloDecode, EmptyOutputHasNoDetections) {
  EXPECT_TRUE(pkg::decodeYoloOutput(pkg::YoloOutput{}, fourClasses(), 0.45F).empty());
}

TEST(Letterbox, PadsCenteredAndWritesRgbPlanes) {
  const cv::Mat image(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  const auto tensor = pkg::letterboxTensor(image, 1280, 1280);
  const std::size_t plane = 1280 * 1280;
  ASSERT_EQ(3 * plane, tensor.size());
  for (std::size_t channel = 0; channel < 3; ++channel) {
    EXPECT_FLOAT_EQ(114.0F / 255.0F, tensor[channel * plane]);
    EXPECT_FLOAT_EQ(114.0F / 255.0F, tensor[channel * plane + 159 * 1280 + 640]);
    EXPECT_FLOAT_EQ(114.0F / 255.0F, tensor[channel * plane + 1120 * 1280 + 640]);
  }
  const std::size_t inside = 160 * 1280 + 640;
  EXPECT_FLOAT_EQ(30.0F / 255.0F, tensor[inside]);
  EXPECT_FLOAT_EQ(20.0F / 255.0F, tensor[plane + inside]);
  EXPECT_FLOAT_EQ(10.0F / 255.0F, tensor[2 * plane + inside]);
}

TEST(StateVoter, RequiresMinimumVotes) {
  pkg::StateVoter voter(5, 3);
  EXPECT_EQ(Light::UNKNOWN, voter.push(Light::GREEN));
  EXPECT_EQ(Light::UNKNOWN, voter.push(Light::GREEN));
  EXPECT_EQ(Light::GREEN, voter.push(Light::GREEN));
}

TEST(StateVoter, BreaksTiesByMostRecentState) {
  pkg::StateVoter voter(4, 2);
  voter.push(Light::GREEN);
  voter.push(Light::RED);
  voter.push(Light::GREEN);
  EXPECT_EQ(Light::RED, voter.push(Light::RED));
}

TEST(StateVoter, UnknownFramesEvictAStaleState) {
  pkg::StateVoter voter(5, 3);
  voter.push(Light::GREEN);
  voter.push(Light::GREEN);
  EXPECT_EQ(Light::GREEN, voter.push(Light::GREEN));
  voter.push(Light::UNKNOWN);
  voter.push(Light::UNKNOWN);
  EXPECT_EQ(Light::UNKNOWN, voter.push(Light::UNKNOWN));
}

TEST(StateVoter, VotesOnWholeCombinations) {
  pkg::StateVoter voter(5, 3);
  voter.push(Light::GREEN_LEFT);
  voter.push(Light::GREEN);
  voter.push(Light::GREEN_LEFT);
  EXPECT_EQ(Light::UNKNOWN, voter.push(Light::GREEN));
  EXPECT_EQ(Light::GREEN_LEFT, voter.push(Light::GREEN_LEFT));
}

TEST(StateVoter, OneOfOneDisablesSmoothing) {
  pkg::StateVoter voter(1, 1);
  EXPECT_EQ(Light::LEFT, voter.push(Light::LEFT));
  EXPECT_EQ(Light::RED_LEFT, voter.push(Light::RED_LEFT));
}

TEST(YoloTrafficLightLoader, ReadsInstalledParameters) {
  const auto parameters = pkg::loadYoloTrafficLightParameters(node_pipeline::ParamReader::fromFile(
      NP_PACKAGE_DIR "/config/traffic_light/parameters/yolo_traffic_light.yaml", NP_PACKAGE_DIR));
  EXPECT_GT(parameters.model.size(), 1000000u);
  EXPECT_TRUE(parameters.device == pkg::InferenceDevice::Cuda);
  EXPECT_EQ(1280, parameters.input_width);
  EXPECT_EQ(1280, parameters.input_height);
  ASSERT_EQ(4u, parameters.classes.size());
  const auto expected = fourClasses();
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_EQ(expected[index].name, parameters.classes[index].name);
    EXPECT_TRUE(expected[index].lamp == parameters.classes[index].lamp);
    EXPECT_FLOAT_EQ(expected[index].confidence, parameters.classes[index].confidence);
  }
  EXPECT_FLOAT_EQ(0.45F, parameters.nms_iou);
  EXPECT_EQ(5u, parameters.history_size);
  EXPECT_EQ(3u, parameters.minimum_votes);
}

TEST(YoloTrafficLightLoader, DefaultsToCudaAndAcceptsCpu) {
  EXPECT_TRUE(fixture("").device == pkg::InferenceDevice::Cuda);
  EXPECT_TRUE(fixture("cpu").device == pkg::InferenceDevice::Cpu);
}

TEST(YoloTrafficLightLoader, RejectsInvalidSettings) {
  const std::string valid = config("test/data/yolo_green_left.onnx", "cpu", 5, 3);
  EXPECT_NO_THROW(load(valid));
  const auto replaced = [&](const std::string& from, const std::string& to) {
    std::string yaml = valid;
    yaml.replace(yaml.find(from), from.size(), to);
    return yaml;
  };
  EXPECT_THROW(load(replaced("device: cpu", "device: gpu")), std::invalid_argument);
  EXPECT_THROW(load(replaced("\"3\": {name: stop", "\"4\": {name: stop")), std::invalid_argument);
  EXPECT_THROW(load(replaced("lamp: red", "lamp: blue")), std::invalid_argument);
  EXPECT_THROW(load(replaced("confidence: 0.35", "confidence: 1.5")), std::invalid_argument);
  EXPECT_THROW(load(replaced("nms_iou: 0.45", "nms_iou: -0.1")), std::invalid_argument);
  EXPECT_THROW(load(replaced("minimum_votes: 3", "minimum_votes: 6")), std::invalid_argument);
  EXPECT_THROW(load(replaced("input_width: 1280", "input_width: 0")), std::invalid_argument);
  EXPECT_ANY_THROW(load(replaced("yolo_green_left.onnx", "missing.onnx")));
}

TEST(DecodableImage, AcceptsDecodableJpegAndPng) {
  const cv::Mat image(48, 64, CV_8UC3, cv::Scalar(0, 128, 255));
  EXPECT_TRUE(validate(compressed(image, ".jpg")).empty());
  EXPECT_TRUE(validate(compressed(image, ".png")).empty());
}

TEST(DecodableImage, RecordsMissingForCorruptOrEmptyData) {
  sensor_msgs::CompressedImage corrupt;
  corrupt.format = "jpeg";
  corrupt.data = {1, 2, 3, 4, 5};
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}, validate(corrupt));
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}, validate(sensor_msgs::CompressedImage{}));
}

TEST(YoloTrafficLight, CpuSessionReturnsTheDetectedCombinationWithoutHeader) {
  pkg::YoloTrafficLight core(fixture("cpu"));
  const auto light = core.process(blankJpeg());
  EXPECT_EQ(Light::GREEN_LEFT, light.state);
  EXPECT_TRUE(light.header.stamp.isZero());
  EXPECT_TRUE(light.header.frame_id.empty());
  EXPECT_EQ(0u, light.header.seq);
}

TEST(YoloTrafficLight, VotesAcrossFrames) {
  pkg::YoloTrafficLight core(fixture("cpu", 3, 2));
  EXPECT_EQ(Light::UNKNOWN, core.process(blankJpeg()).state);
  EXPECT_EQ(Light::GREEN_LEFT, core.process(blankJpeg()).state);
}

TEST(YoloTrafficLight, SessionCreationFailureReturnsUnknownOnEveryDevice) {
  for (const std::string device : {"cuda", "cpu"}) {
    auto parameters = fixture(device);
    parameters.model = "not an onnx model";
    std::unique_ptr<pkg::YoloTrafficLight> core;
    ASSERT_NO_THROW(core.reset(new pkg::YoloTrafficLight(parameters))) << device;
    EXPECT_EQ(Light::UNKNOWN, core->process(blankJpeg()).state) << device;
    EXPECT_EQ(Light::UNKNOWN, core->process(blankJpeg()).state) << device;
  }
}

TEST(YoloTrafficLight, RejectsModelContractMismatchAtConstruction) {
  auto wrong_channels = load(config("test/data/yolo_wrong_channels.onnx", "cpu", 1, 1));
  EXPECT_THROW(pkg::YoloTrafficLight core(wrong_channels), std::invalid_argument);
  auto wrong_input = fixture("cpu");
  wrong_input.input_width = 640;
  EXPECT_THROW(pkg::YoloTrafficLight core(wrong_input), std::invalid_argument);
}

// 배포 모델을 CPU 세션으로 실제 추론한다. 합성 영상이라 상태 값은 고정하지 않고 계약 범위만 확인한다.
TEST(YoloTrafficLight, DeployedModelRunsOnCpu) {
  auto parameters = pkg::loadYoloTrafficLightParameters(node_pipeline::ParamReader::fromFile(
      NP_PACKAGE_DIR "/config/traffic_light/parameters/yolo_traffic_light.yaml", NP_PACKAGE_DIR));
  parameters.device = pkg::InferenceDevice::Cpu;
  parameters.history_size = 1;
  parameters.minimum_votes = 1;
  pkg::YoloTrafficLight core(parameters);
  sensor_msgs::CompressedImage image;
  image.format = "jpeg";
  const cv::Mat bgr = cv::imread(NP_PACKAGE_DIR "/test/data/traffic_light_red.jpg", cv::IMREAD_COLOR);
  ASSERT_FALSE(bgr.empty());
  cv::imencode(".jpg", bgr, image.data);
  ASSERT_TRUE(validate(image).empty());
  const auto light = core.process(image);
  RecordProperty("state", static_cast<int>(light.state));
  std::cout << "deployed model state for traffic_light_red.jpg: " << static_cast<unsigned>(light.state) << std::endl;
  EXPECT_LE(light.state, Light::RED_YELLOW_GREEN_LEFT);
  EXPECT_NE(Light::OFF, light.state);
  EXPECT_TRUE(light.header.frame_id.empty());
}
