#include <traffic_light/cores/yolo_traffic_light_core.hpp>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#ifndef TRAFFIC_LIGHT_FIXTURE_DIR
#error TRAFFIC_LIGHT_FIXTURE_DIR is required
#endif

namespace tl = traffic_light;
namespace {
tl::TrafficLightParameters parameters() {
  tl::TrafficLightParameters p;
  p.model_path = TRAFFIC_LIGHT_FIXTURE_DIR "/yolo_fixture.onnx";
  p.manifest_path = TRAFFIC_LIGHT_FIXTURE_DIR "/yolo_fixture.json";
  p.detection.history_size = 1;
  p.detection.minimum_votes = 1;
  return p;
}

sensor_msgs::CompressedImage jpeg() {
  sensor_msgs::CompressedImage message;
  cv::imencode(".jpg", cv::Mat::zeros(480, 640, CV_8UC3), message.data);
  message.header.stamp.fromNSec(123);
  message.header.frame_id = "input";
  return message;
}
}  // namespace

TEST(YoloTrafficLightCore, DecodesJpegAndReturnsMappedState) {
  tl::YoloTrafficLightCore core(parameters());
  EXPECT_EQ(interfaces::TrafficLight::GREEN, core.process(jpeg()).state);
}

TEST(YoloTrafficLightCore, CorruptJpegReturnsUnknown) {
  tl::YoloTrafficLightCore core(parameters());
  sensor_msgs::CompressedImage message;
  message.data = {1, 2, 3};
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(message).state);
}

TEST(YoloTrafficLightCore, DoesNotWriteHeader) {
  tl::YoloTrafficLightCore core(parameters());
  const auto output = core.process(jpeg());
  EXPECT_TRUE(output.header.stamp.isZero());
  EXPECT_TRUE(output.header.frame_id.empty());
}

TEST(YoloTrafficLightCore, RejectsManifestClassOrderMismatch) {
  auto p = parameters();
  p.manifest_path = TRAFFIC_LIGHT_FIXTURE_DIR "/wrong_classes.json";
  EXPECT_THROW(tl::YoloTrafficLightCore core(p), std::invalid_argument);
}

TEST(YoloTrafficLightCore, RejectsExpectedClassesNestedUnderWrongTopLevelClasses) {
  auto p = parameters();
  p.manifest_path = TRAFFIC_LIGHT_FIXTURE_DIR "/nested_wrong_classes.json";
  EXPECT_THROW(tl::YoloTrafficLightCore core(p), std::invalid_argument);
}

TEST(YoloTrafficLightCore, RejectsUnexpectedOutputShapeAtStartup) {
  auto p = parameters();
  p.model_path = TRAFFIC_LIGHT_FIXTURE_DIR "/wrong_output_shape.onnx";
  EXPECT_THROW(tl::YoloTrafficLightCore core(p), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
