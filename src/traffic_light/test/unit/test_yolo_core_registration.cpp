#include <traffic_light/core_dispatch.hpp>
#include <gtest/gtest.h>

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
}  // namespace

TEST(YoloCoreRegistration, MissingImageSkipsCoreAndPublication) {
  tl::CoreSet cores(parameters());
  node_pipeline::StageData data;
  node_pipeline::CoreResult result;
  int calls = 0;
  EXPECT_FALSE(tl::processCore(cores, tl::CoreId::YoloTrafficLight, data, result,
                               [&] { ++calls; }));
  EXPECT_EQ(0, calls);
}

TEST(YoloCoreRegistration, ValidImageCallsCoreAndProducesPayload) {
  tl::CoreSet cores(parameters());
  tl::Data input;
  sensor_msgs::CompressedImage message;
  message.data = {1, 2, 3};
  input.get<0>().sample.message =
      std::make_shared<const sensor_msgs::CompressedImage>(message);
  node_pipeline::StageData data;
  data.update(input);
  node_pipeline::CoreResult result;
  int calls = 0;
  ASSERT_TRUE(tl::processCore(cores, tl::CoreId::YoloTrafficLight, data, result,
                              [&] { ++calls; }));
  EXPECT_EQ(1, calls);
  const auto output = result(data);
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN,
            output.message<interfaces::TrafficLight>(0).state);
  EXPECT_TRUE(output.message<interfaces::TrafficLight>(0).header.stamp.isZero());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
