#include <traffic_light/detection.hpp>
#include <gtest/gtest.h>
#include <limits>

namespace tl = traffic_light;

namespace {
tl::DetectionParameters parameters() {
  tl::DetectionParameters p;
  p.confidence = {{0.4F, 0.4F, 0.35F, 0.4F}};
  p.nms_iou = 0.45F;
  return p;
}

std::vector<float> channelsFirst(std::size_t count) {
  return std::vector<float>(8 * count, 0.0F);
}

void candidate(std::vector<float>& data, std::size_t count, std::size_t index,
               float x, float y, float w, float h, int class_id, float score) {
  data[0 * count + index] = x;
  data[1 * count + index] = y;
  data[2 * count + index] = w;
  data[3 * count + index] = h;
  data[(4 + class_id) * count + index] = score;
}
}  // namespace

TEST(Detection, DecodesChannelsFirstTensor) {
  auto data = channelsFirst(2);
  candidate(data, 2, 0, 100, 200, 20, 40, 0, 0.9F);
  candidate(data, 2, 1, 300, 400, 60, 80, 3, 0.8F);
  const auto out = tl::decodeYoloOutput(data.data(), {1, 8, 2}, parameters());
  ASSERT_EQ(2U, out.size());
  EXPECT_EQ(tl::DetectionState::Go, out[0].state);
  EXPECT_FLOAT_EQ(90, out[0].box.x);
  EXPECT_FLOAT_EQ(180, out[0].box.y);
  EXPECT_EQ(tl::DetectionState::Stop, out[1].state);
}

TEST(Detection, DecodesRowsFirstTensor) {
  const std::vector<float> data{100, 200, 20, 40, 0, 0.85F, 0, 0};
  const auto out = tl::decodeYoloOutput(data.data(), {1, 1, 8}, parameters());
  ASSERT_EQ(1U, out.size());
  EXPECT_EQ(tl::DetectionState::LeftTurn, out[0].state);
}

TEST(Detection, AppliesPerClassConfidence) {
  auto data = channelsFirst(2);
  candidate(data, 2, 0, 100, 100, 20, 20, 0, 0.39F);
  candidate(data, 2, 1, 200, 200, 20, 20, 2, 0.36F);
  const auto out = tl::decodeYoloOutput(data.data(), {1, 8, 2}, parameters());
  ASSERT_EQ(1U, out.size());
  EXPECT_EQ(tl::DetectionState::PrepareStop, out[0].state);
}

TEST(Detection, SuppressesSameClassOverlap) {
  auto data = channelsFirst(3);
  candidate(data, 3, 0, 100, 100, 40, 40, 0, 0.9F);
  candidate(data, 3, 1, 101, 101, 40, 40, 0, 0.8F);
  candidate(data, 3, 2, 101, 101, 40, 40, 3, 0.7F);
  const auto out = tl::decodeYoloOutput(data.data(), {1, 8, 3}, parameters());
  EXPECT_EQ(2U, out.size());
}

TEST(Detection, RejectsUnexpectedTensorShape) {
  const float value = 0;
  EXPECT_THROW(tl::decodeYoloOutput(&value, {1, 7, 1}, parameters()), std::invalid_argument);
  EXPECT_THROW(tl::decodeYoloOutput(&value, {2, 8, 1}, parameters()), std::invalid_argument);
  EXPECT_THROW(tl::decodeYoloOutput(nullptr, {1, 8, 1}, parameters()), std::invalid_argument);
}

TEST(Detection, MapsExactFourClasses) {
  EXPECT_EQ(interfaces::TrafficLight::GREEN, tl::toRosState(tl::DetectionState::Go));
  EXPECT_EQ(interfaces::TrafficLight::LEFT, tl::toRosState(tl::DetectionState::LeftTurn));
  EXPECT_EQ(interfaces::TrafficLight::YELLOW, tl::toRosState(tl::DetectionState::PrepareStop));
  EXPECT_EQ(interfaces::TrafficLight::RED, tl::toRosState(tl::DetectionState::Stop));
  EXPECT_EQ(interfaces::TrafficLight::UNKNOWN, tl::toRosState(tl::DetectionState::Unknown));
}

TEST(Detection, ReturnsUnknownWithoutCandidates) {
  EXPECT_EQ(tl::DetectionState::Unknown, tl::chooseFrameState({}, 0.05F));
}

TEST(Detection, ReturnsUnknownForAmbiguousClassesAtMargin) {
  const std::vector<tl::Detection> detections{
      {{0, 0, 1, 1}, 0.90F, tl::DetectionState::Go},
      {{2, 2, 1, 1}, 0.85F, tl::DetectionState::Stop}};
  EXPECT_EQ(tl::DetectionState::Unknown, tl::chooseFrameState(detections, 0.05F));
}

TEST(Detection, ChoosesWinnerOutsideAmbiguityMargin) {
  const std::vector<tl::Detection> detections{
      {{0, 0, 1, 1}, 0.91F, tl::DetectionState::Go},
      {{2, 2, 1, 1}, 0.85F, tl::DetectionState::Stop}};
  EXPECT_EQ(tl::DetectionState::Go, tl::chooseFrameState(detections, 0.05F));
}

TEST(Detection, RequiresThreeOfFiveVotes) {
  tl::TemporalVoter voter(5, 3);
  EXPECT_EQ(tl::DetectionState::Unknown, voter.push(tl::DetectionState::Go));
  EXPECT_EQ(tl::DetectionState::Unknown, voter.push(tl::DetectionState::Go));
  EXPECT_EQ(tl::DetectionState::Go, voter.push(tl::DetectionState::Go));
}

TEST(Detection, BreaksVoteTieByMostRecentOccurrence) {
  tl::TemporalVoter voter(4, 2);
  voter.push(tl::DetectionState::Go);
  voter.push(tl::DetectionState::Stop);
  voter.push(tl::DetectionState::Go);
  EXPECT_EQ(tl::DetectionState::Stop, voter.push(tl::DetectionState::Stop));
}

TEST(Detection, UnknownEvictsStaleGo) {
  tl::TemporalVoter voter(5, 3);
  voter.push(tl::DetectionState::Go);
  voter.push(tl::DetectionState::Go);
  EXPECT_EQ(tl::DetectionState::Go, voter.push(tl::DetectionState::Go));
  voter.push(tl::DetectionState::Unknown);
  voter.push(tl::DetectionState::Unknown);
  EXPECT_EQ(tl::DetectionState::Unknown, voter.push(tl::DetectionState::Unknown));
}

TEST(Detection, OneOfOneDisablesSmoothing) {
  tl::TemporalVoter voter(1, 1);
  EXPECT_EQ(tl::DetectionState::LeftTurn, voter.push(tl::DetectionState::LeftTurn));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
