#include <traffic_light/config.hpp>
#include <gtest/gtest.h>
#include <limits>

namespace tl = traffic_light;

TEST(Config, ProvidesDocumentedDefaults) {
  const auto p = tl::defaultTrafficLightParameters();
  EXPECT_EQ(1280, p.input_width);
  EXPECT_EQ(1280, p.input_height);
  EXPECT_EQ((std::array<float, 4>{{0.4F, 0.4F, 0.35F, 0.4F}}), p.detection.confidence);
  EXPECT_FLOAT_EQ(0.45F, p.detection.nms_iou);
  EXPECT_FLOAT_EQ(0.05F, p.detection.ambiguity_margin);
  EXPECT_EQ(5U, p.detection.history_size);
  EXPECT_EQ(3U, p.detection.minimum_votes);
  EXPECT_FALSE(p.allow_cpu_fallback);
}

TEST(Config, RejectsMissingPaths) {
  auto p = tl::defaultTrafficLightParameters();
  EXPECT_THROW(tl::validateTrafficLightParameters(p), std::invalid_argument);
}

TEST(Config, RejectsNonPositiveInputSize) {
  auto p = tl::defaultTrafficLightParameters();
  p.model_path = p.manifest_path = "/tmp/value";
  p.input_width = 0;
  EXPECT_THROW(tl::validateTrafficLightParameters(p), std::invalid_argument);
}

TEST(Config, RejectsThresholdOutsideUnitInterval) {
  auto p = tl::defaultTrafficLightParameters();
  p.model_path = p.manifest_path = "/tmp/value";
  p.detection.confidence[0] = 1.1F;
  EXPECT_THROW(tl::validateTrafficLightParameters(p), std::invalid_argument);
}

TEST(Config, RejectsNonFiniteThreshold) {
  auto p = tl::defaultTrafficLightParameters();
  p.model_path = p.manifest_path = "/tmp/value";
  p.detection.nms_iou = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(tl::validateTrafficLightParameters(p), std::invalid_argument);
}

TEST(Config, RejectsZeroHistory) {
  auto p = tl::defaultTrafficLightParameters();
  p.model_path = p.manifest_path = "/tmp/value";
  p.detection.history_size = 0;
  EXPECT_THROW(tl::validateTrafficLightParameters(p), std::invalid_argument);
}

TEST(Config, RejectsMinimumVotesGreaterThanHistory) {
  auto p = tl::defaultTrafficLightParameters();
  p.model_path = p.manifest_path = "/tmp/value";
  p.detection.minimum_votes = 6;
  EXPECT_THROW(tl::validateTrafficLightParameters(p), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
