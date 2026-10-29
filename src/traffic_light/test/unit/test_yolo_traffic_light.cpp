#include <gtest/gtest.h>
#include <traffic_light/cores/yolo_traffic_light.hpp>
#include <limits>

namespace {
using Core = traffic_light::YoloTrafficLight;
using State = Core::DetectionState;

const cv::Rect2f kHead(20.0F, 10.0F, 24.0F, 8.0F);

// Dark horizontal head on a grey sky with one lit lamp (BGR) at lamp index 0..2 (red, yellow, green).
cv::Mat head(const cv::Scalar& lamp, int index) {
  cv::Mat image(40, 80, CV_8UC3, cv::Scalar(170, 165, 160));
  cv::rectangle(image, cv::Rect(20, 10, 24, 8), cv::Scalar(45, 40, 40), cv::FILLED);
  cv::circle(image, cv::Point(24 + 8 * index, 14), 3, lamp, cv::FILLED);
  return image;
}
const cv::Scalar kGreen(90, 220, 40), kRed(50, 40, 225), kYellow(30, 200, 240);

Core::Detection detection(State state, float confidence, const cv::Rect2f& box = kHead) {
  return {box, confidence, state};
}
}  // namespace

TEST(YoloTrafficLight, DecodeKeepsCandidatesDownToTrackMinimum) {
  traffic_light::YoloTrafficLightParameters parameters;
  // Channels-first [1, 8, 3]: cx, cy, w, h, go, left_turn, prepare_stop, stop.
  const std::vector<float> data{
      100, 300, 500,  100, 300, 500,  20, 20, 20,  8, 8, 8,
      0.20F, 0.0F, 0.0F,  0.0F, 0.0F, 0.0F,  0.0F, 0.0F, 0.0F,  0.0F, 0.04F, 0.70F};
  const auto detections = Core::decodeYoloOutput(data.data(), {1, 8, 3}, parameters);
  ASSERT_EQ(detections.size(), 2U);
  EXPECT_EQ(detections[0].state, State::Stop);
  EXPECT_FLOAT_EQ(detections[0].confidence, 0.70F);
  EXPECT_EQ(detections[1].state, State::Go);
  EXPECT_FLOAT_EQ(detections[1].box.x, 90.0F);
  EXPECT_FLOAT_EQ(detections[1].box.y, 96.0F);
}

TEST(YoloTrafficLight, CountsLitLampColours) {
  traffic_light::YoloTrafficLightParameters parameters;
  const auto green = Core::litPixels(head(kGreen, 2), kHead, parameters.track);
  EXPECT_GE(green.green, 3);
  EXPECT_EQ(green.red, 0);
  EXPECT_EQ(green.yellow, 0);
  const auto red = Core::litPixels(head(kRed, 0), kHead, parameters.track);
  EXPECT_GE(red.red, 3);
  EXPECT_EQ(red.green, 0);
  const auto yellow = Core::litPixels(head(kYellow, 1), kHead, parameters.track);
  EXPECT_GE(yellow.yellow, 3);
  EXPECT_EQ(yellow.red, 0);
  EXPECT_EQ(yellow.green, 0);
  const auto outside = Core::litPixels(head(kGreen, 2), cv::Rect2f(70, 30, 20, 20), parameters.track);
  EXPECT_EQ(outside.green, 0);
}

TEST(YoloTrafficLight, AcceptsWeakGoOnTrackedHeadWithGreenLamp) {
  traffic_light::YoloTrafficLightParameters parameters;
  std::vector<Core::Track> tracks;
  auto accepted = Core::acceptDetections({detection(State::Stop, 0.70F)}, head(kRed, 0), 10.0,
                                         parameters, tracks);
  EXPECT_EQ(Core::chooseFrameState(accepted, 0.05F), State::Stop);
  ASSERT_EQ(tracks.size(), 1U);

  accepted = Core::acceptDetections({detection(State::Go, 0.15F)}, head(kGreen, 2), 10.5,
                                    parameters, tracks);
  EXPECT_EQ(Core::chooseFrameState(accepted, 0.05F), State::Go);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_DOUBLE_EQ(tracks.front().stamp_sec, 10.5);
}

TEST(YoloTrafficLight, RejectsWeakGoWithoutTrackedHead) {
  traffic_light::YoloTrafficLightParameters parameters;
  std::vector<Core::Track> tracks;
  const auto accepted = Core::acceptDetections({detection(State::Go, 0.15F)}, head(kGreen, 2), 10.0,
                                               parameters, tracks);
  EXPECT_EQ(Core::chooseFrameState(accepted, 0.05F), State::Unknown);
  EXPECT_TRUE(tracks.empty());
}

TEST(YoloTrafficLight, RejectsWeakGoWhenLampColourDisagrees) {
  traffic_light::YoloTrafficLightParameters parameters;
  std::vector<Core::Track> tracks{{kHead, 10.0}};
  auto accepted = Core::acceptDetections({detection(State::Go, 0.15F)}, head(kRed, 0), 10.2,
                                         parameters, tracks);
  EXPECT_TRUE(accepted.empty());
  cv::Mat both = head(kRed, 0);
  cv::circle(both, cv::Point(40, 14), 3, kGreen, cv::FILLED);
  accepted = Core::acceptDetections({detection(State::Go, 0.15F)}, both, 10.3, parameters, tracks);
  EXPECT_TRUE(accepted.empty());
}

TEST(YoloTrafficLight, NeverBoostsLeftTurnOrCandidatesBelowMinimum) {
  traffic_light::YoloTrafficLightParameters parameters;
  std::vector<Core::Track> tracks{{kHead, 10.0}};
  auto accepted = Core::acceptDetections({detection(State::LeftTurn, 0.30F)}, head(kGreen, 2), 10.1,
                                         parameters, tracks);
  EXPECT_TRUE(accepted.empty());
  accepted = Core::acceptDetections({detection(State::Go, 0.04F)}, head(kGreen, 2), 10.2,
                                    parameters, tracks);
  EXPECT_TRUE(accepted.empty());
}

TEST(YoloTrafficLight, TrackExpiresAfterTimeoutOrStampRewind) {
  traffic_light::YoloTrafficLightParameters parameters;
  std::vector<Core::Track> tracks{{kHead, 10.0}};
  auto accepted = Core::acceptDetections({detection(State::Go, 0.15F)}, head(kGreen, 2), 13.5,
                                         parameters, tracks);
  EXPECT_TRUE(accepted.empty());
  EXPECT_TRUE(tracks.empty());
  tracks = {{kHead, 10.0}};
  accepted = Core::acceptDetections({detection(State::Go, 0.15F)}, head(kGreen, 2), 9.0,
                                    parameters, tracks);
  EXPECT_TRUE(accepted.empty());
}

TEST(YoloTrafficLight, WeakCandidateNeedsOverlapWithTrack) {
  traffic_light::YoloTrafficLightParameters parameters;
  std::vector<Core::Track> tracks{{cv::Rect2f(50.0F, 25.0F, 24.0F, 8.0F), 10.0}};
  const auto accepted = Core::acceptDetections({detection(State::Go, 0.15F)}, head(kGreen, 2), 10.1,
                                               parameters, tracks);
  EXPECT_TRUE(accepted.empty());
}

TEST(YoloTrafficLight, ConfidentDetectionsPassWithoutTrackOrColour) {
  traffic_light::YoloTrafficLightParameters parameters;
  std::vector<Core::Track> tracks;
  const auto accepted = Core::acceptDetections(
      {detection(State::Go, 0.90F), detection(State::Stop, 0.20F, cv::Rect2f(60, 25, 10, 5))},
      head(kGreen, 2), 10.0, parameters, tracks);
  ASSERT_EQ(accepted.size(), 1U);
  EXPECT_EQ(Core::chooseFrameState(accepted, 0.05F), State::Go);
  EXPECT_EQ(Core::toRosState(State::Go), interfaces::TrafficLight::GREEN);
}

TEST(YoloTrafficLight, AmbiguousStatesWithinMarginAreUnknown) {
  const std::vector<Core::Detection> detections{
      detection(State::Stop, 0.42F), detection(State::Go, 0.39F, cv::Rect2f(60, 25, 10, 5))};
  EXPECT_EQ(Core::chooseFrameState(detections, 0.05F), State::Unknown);
  EXPECT_EQ(Core::chooseFrameState(detections, 0.02F), State::Stop);
}

TEST(YoloTrafficLight, UnknownFramesOutvoteSparseDetectionsByDefault) {
  traffic_light::YoloTrafficLightParameters::DetectionParameters parameters;
  parameters.minimum_votes = 1;
  const std::deque<State> history{State::Unknown, State::Unknown, State::Unknown, State::Unknown,
                                  State::Stop};
  EXPECT_EQ(Core::vote(history, parameters), State::Unknown);
  parameters.unknown_votes = false;
  EXPECT_EQ(Core::vote(history, parameters), State::Stop);
}

TEST(YoloTrafficLight, VoteWithoutUnknownKeepsMajorityAndNewestFrameUnknown) {
  traffic_light::YoloTrafficLightParameters::DetectionParameters parameters;
  parameters.minimum_votes = 1;
  parameters.unknown_votes = false;
  EXPECT_EQ(Core::vote({State::Stop, State::Stop, State::Unknown, State::Unknown, State::Go}, parameters),
            State::Stop);
  EXPECT_EQ(Core::vote({State::Stop, State::Unknown, State::Unknown, State::Unknown, State::Go}, parameters),
            State::Go);
  EXPECT_EQ(Core::vote({State::Stop, State::Stop, State::Stop, State::Stop, State::Unknown}, parameters),
            State::Unknown);
  parameters.minimum_votes = 2;
  EXPECT_EQ(Core::vote({State::Unknown, State::Unknown, State::Unknown, State::Unknown, State::Stop}, parameters),
            State::Unknown);
}

TEST(YoloTrafficLight, NonFiniteBoxHasNoLitPixels) {
  traffic_light::YoloTrafficLightParameters parameters;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto lit = Core::litPixels(head(kGreen, 2), cv::Rect2f(nan, 10, 24, 8), parameters.track);
  EXPECT_EQ(lit.red + lit.yellow + lit.green, 0);
}

TEST(YoloTrafficLight, DehazeWithUnitTransmissionKeepsImage) {
  traffic_light::YoloTrafficLightParameters parameters;
  const cv::Mat image = head(kRed, 0);
  const cv::Mat output = Core::dehaze(image, parameters.dehaze);
  EXPECT_EQ(cv::norm(image, output, cv::NORM_INF), 0.0);
}

TEST(YoloTrafficLight, DehazeRestoresContrastAgainstAirlight) {
  traffic_light::YoloTrafficLightParameters parameters;
  parameters.dehaze.transmission = 0.5;
  parameters.dehaze.window = 3;
  parameters.dehaze.airlight_fraction = 0.1;
  // Haze: every pixel is 0.5 * scene + 0.5 * airlight (200); the sky (top half) is the airlight itself.
  cv::Mat image(40, 80, CV_8UC3, cv::Scalar(200, 200, 200));
  cv::rectangle(image, cv::Rect(20, 24, 24, 8), cv::Scalar(120, 120, 120), cv::FILLED);  // scene 40 seen through haze
  const cv::Mat output = Core::dehaze(image, parameters.dehaze);
  EXPECT_NEAR(output.at<cv::Vec3b>(28, 30)[0], 40, 1);
  EXPECT_NEAR(output.at<cv::Vec3b>(5, 5)[1], 200, 1);
}

TEST(YoloTrafficLight, DropsCandidatesBelowMaximumCenterRow) {
  const std::vector<Core::Detection> candidates{
      detection(State::Stop, 0.8F, cv::Rect2f(300, 200, 20, 8)),
      detection(State::Go, 0.8F, cv::Rect2f(300, 330, 90, 14))};
  const auto kept = Core::aboveRow(candidates, 285.0);
  ASSERT_EQ(kept.size(), 1U);
  EXPECT_EQ(kept.front().state, State::Stop);
  EXPECT_EQ(Core::aboveRow(candidates, 0.0).size(), 2U);
}

TEST(YoloTrafficLight, LeftCombinationFollowsLitLamps) {
  traffic_light::YoloTrafficLightParameters parameters;
  cv::Mat red_left = head(kRed, 0);
  cv::circle(red_left, cv::Point(40, 14), 3, kGreen, cv::FILLED);
  EXPECT_EQ(Core::leftCombination(Core::litPixels(red_left, kHead, parameters.track), 3),
            interfaces::TrafficLight::RED_LEFT);
  cv::Mat green_left = head(kGreen, 2);
  cv::circle(green_left, cv::Point(32, 14), 3, kGreen, cv::FILLED);
  EXPECT_EQ(Core::leftCombination(Core::litPixels(green_left, kHead, parameters.track), 3),
            interfaces::TrafficLight::GREEN_LEFT);
  cv::Mat unlit(40, 80, CV_8UC3, cv::Scalar(170, 165, 160));
  EXPECT_EQ(Core::leftCombination(Core::litPixels(unlit, kHead, parameters.track), 3),
            interfaces::TrafficLight::UNKNOWN);
}
