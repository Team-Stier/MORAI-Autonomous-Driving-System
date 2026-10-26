#include <traffic_light/parameters/camera7_lamp_cpu.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>

namespace {
cv::Mat scene() {
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::rectangle(image, cv::Point(0, 0), cv::Point(639, 179), cv::Scalar(210, 170, 120), cv::FILLED);
  return image;
}

void addLamp(cv::Mat& image, int x, int y, const cv::Scalar& bgr, int size = 8) {
  cv::rectangle(image, cv::Point(x, y), cv::Point(x + size - 1, y + size - 1), bgr, cv::FILLED);
}

sensor_msgs::CompressedImage encode(const cv::Mat& image) {
  sensor_msgs::CompressedImage message;
  message.header.frame_id = "camera_7_optical_frame";
  message.format = "jpeg";
  cv::imencode(".jpg", image, message.data, {cv::IMWRITE_JPEG_QUALITY, 90});
  return message;
}

cv::Mat fog(const cv::Mat& image, double alpha) {
  cv::Mat gray(image.size(), image.type(), cv::Scalar(170, 170, 170));
  cv::Mat out;
  cv::addWeighted(image, 1.0 - alpha, gray, alpha, 0, out);
  return out;
}

cv::Mat bright(const cv::Mat& image, double scale) {
  cv::Mat out;
  image.convertTo(out, CV_8UC3, scale);
  return out;
}

traffic_light::Camera7LampCpuParameters onceParameters() {
  traffic_light::Camera7LampCpuParameters parameters;
  parameters.minimum_repeat = 1;
  return parameters;
}

std::uint8_t readOnce(const cv::Mat& image) {
  traffic_light::Camera7LampCpu core(onceParameters());
  return core.process(encode(image)).state;
}

const cv::Scalar kWeakGreen(107, 162, 106);
const cv::Scalar kRed(40, 40, 200);
const cv::Scalar kYellow(40, 200, 220);
}  // namespace

TEST(Camera7LampCpu, CenterLampsUseLampColor) {
  cv::Mat green = scene();
  addLamp(green, 316, 70, kWeakGreen);
  EXPECT_EQ(interfaces::TrafficLight::GREEN, readOnce(green));

  cv::Mat red = scene();
  addLamp(red, 316, 70, kRed);
  EXPECT_EQ(interfaces::TrafficLight::RED, readOnce(red));

  cv::Mat yellow = scene();
  addLamp(yellow, 316, 70, kYellow);
  EXPECT_EQ(interfaces::TrafficLight::YELLOW, readOnce(yellow));
}

TEST(Camera7LampCpu, RedBesideGreenIsLeftArrow) {
  cv::Mat image = scene();
  addLamp(image, 300, 70, kRed);
  addLamp(image, 318, 70, kWeakGreen);
  EXPECT_EQ(interfaces::TrafficLight::RED_LEFT, readOnce(image));
}

TEST(Camera7LampCpu, TwoSeparatedGreensAreGreenAndArrow) {
  cv::Mat image = scene();
  addLamp(image, 300, 70, kWeakGreen);
  addLamp(image, 318, 70, kWeakGreen);
  EXPECT_EQ(interfaces::TrafficLight::GREEN_LEFT, readOnce(image));
}

TEST(Camera7LampCpu, GreenLampInRedSceneIsNotRed) {
  cv::Mat image = scene();
  cv::rectangle(image, cv::Point(0, 0), cv::Point(639, 278), cv::Scalar(30, 30, 160), cv::FILLED);
  cv::rectangle(image, cv::Point(300, 55), cv::Point(348, 100), cv::Scalar(40, 40, 40), cv::FILLED);
  addLamp(image, 316, 70, kWeakGreen);
  EXPECT_EQ(interfaces::TrafficLight::GREEN, readOnce(image));
}

TEST(Camera7LampCpu, WholeRedBandAndBrakeLightAndLaneDashAreUnknown) {
  cv::Mat red_band = scene();
  cv::rectangle(red_band, cv::Point(0, 0), cv::Point(639, 278), cv::Scalar(30, 30, 200), cv::FILLED);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, readOnce(red_band));

  cv::Mat brake = scene();
  addLamp(brake, 316, 400, kRed);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, readOnce(brake));

  cv::Mat dash = scene();
  cv::rectangle(dash, cv::Point(200, 160), cv::Point(236, 210), kYellow, cv::FILLED);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, readOnce(dash));
}

TEST(Camera7LampCpu, CenterGreenBeatsSideRedAndDisagreementIsUnknown) {
  cv::Mat sided = scene();
  addLamp(sided, 20, 60, kRed);
  addLamp(sided, 316, 70, kWeakGreen);
  EXPECT_EQ(interfaces::TrafficLight::GREEN, readOnce(sided));

  cv::Mat split = scene();
  addLamp(split, 250, 70, kRed);
  addLamp(split, 390, 70, kWeakGreen);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, readOnce(split));
}

TEST(Camera7LampCpu, HoldsLampColorUnderFogAndBrightness) {
  for (const double alpha : {0.0, 0.35, 0.45}) {
    for (const double scale : {0.75, 1.0, 1.25}) {
      SCOPED_TRACE("fog " + std::to_string(alpha) + " scale " + std::to_string(scale));
      cv::Mat green = scene();
      addLamp(green, 316, 70, kWeakGreen);
      cv::Mat red = scene();
      addLamp(red, 316, 70, kRed);
      EXPECT_EQ(interfaces::TrafficLight::GREEN, readOnce(bright(fog(green, alpha), scale)));
      EXPECT_EQ(interfaces::TrafficLight::RED, readOnce(bright(fog(red, alpha), scale)));
    }
  }
}

TEST(Camera7LampCpu, RepeatWaitsForASecondAgreeingFrame) {
  traffic_light::Camera7LampCpu core(traffic_light::Camera7LampCpuParameters{});
  cv::Mat image = scene();
  addLamp(image, 316, 70, kWeakGreen);
  const auto encoded = encode(image);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(encoded).state);
  EXPECT_EQ(interfaces::TrafficLight::GREEN, core.process(encoded).state);
  EXPECT_TRUE(core.process(encoded).header.stamp.isZero());
}

TEST(Camera7LampCpu, RepeatCountFollowsMinimumRepeat) {
  traffic_light::Camera7LampCpuParameters parameters;
  parameters.minimum_repeat = 3;
  traffic_light::Camera7LampCpu core(parameters);
  cv::Mat green = scene();
  addLamp(green, 316, 70, kWeakGreen);
  const auto green_encoded = encode(green);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(green_encoded).state);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(green_encoded).state);
  EXPECT_EQ(interfaces::TrafficLight::GREEN, core.process(green_encoded).state);

  cv::Mat red = scene();
  addLamp(red, 316, 70, kRed);
  const auto red_encoded = encode(red);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(red_encoded).state);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, core.process(red_encoded).state);
  EXPECT_EQ(interfaces::TrafficLight::RED, core.process(red_encoded).state);
}

TEST(Camera7LampCpu, LoaderRejectsAnInvertedAreaRange) {
  EXPECT_THROW(traffic_light::loadCamera7LampCpuParameters(
                   node_pipeline::ParamReader::fromString("min_area: 40\nmax_area: 4\n")),
               std::invalid_argument);
}
