#include <gtest/gtest.h>
#include <velocity_planner/policies/missing_light_unknown.hpp>
#include <velocity_planner/policies/remove_invalid.hpp>
#include <velocity_planner/validators/curvature_inputs.hpp>
#include <velocity_planner/validators/stop_line_inputs.hpp>
#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <typeindex>
#include <vector>

namespace {
using node_pipeline::Kind;

// 등록 YAML의 light allowed_issues다.
const std::vector<Kind> kAllowed{Kind::Stale, Kind::ReceiveTimeout, Kind::Skew};

interfaces::TrafficLight light(std::uint8_t state) {
  interfaces::TrafficLight message;
  message.header.frame_id = "camera_7_optical_frame";
  message.header.stamp = ros::Time(100.0);
  message.state = state;
  return message;
}

// Pipeline README 6~9단계의 light 단일 입력 검사 순서: 공통 issue 뒤 Validator, 전부 허용이면 수용, 아니면 Policy 후 재검사.
template<class Validator, class Policy>
std::shared_ptr<const void> settle(interfaces::TrafficLight value, const std::vector<Kind>& common) {
  std::shared_ptr<const void> message = std::make_shared<interfaces::TrafficLight>(value);
  for (int check = 0; check < 10; ++check) {
    std::vector<Kind> issues;
    if (!message) {
      issues.push_back(Kind::Missing);
    } else {
      issues = common;
      Validator{}(*static_cast<const interfaces::TrafficLight*>(message.get()), node_pipeline::HealthLimits{}, issues);
    }
    const bool accepted = std::all_of(issues.begin(), issues.end(), [](Kind kind) {
      return std::find(kAllowed.begin(), kAllowed.end(), kind) != kAllowed.end();
    });
    if (accepted) return message;
    int best = -1;
    Kind target = Kind::Missing;
    for (const auto kind : issues) {
      if (Policy{}.priority(kind) > best) {
        best = Policy{}.priority(kind);
        target = kind;
      }
    }
    if (best < 0) continue;
    node_pipeline::EditableInput input(message, std::type_index(typeid(interfaces::TrafficLight)));
    Policy{}.correct(input, target);
  }
  return nullptr;
}

const interfaces::TrafficLight& state(const std::shared_ptr<const void>& message) {
  return *static_cast<const interfaces::TrafficLight*>(message.get());
}
}  // namespace

TEST(LightPolicy, InjectReplacesStaleGreenWithUnknown) {
  for (const auto late : {Kind::Stale, Kind::ReceiveTimeout}) {
    const auto message = settle<velocity_planner::CurvatureInputs, velocity_planner::MissingLightUnknown>(
      light(interfaces::TrafficLight::GREEN), {late});
    ASSERT_TRUE(message);
    EXPECT_EQ(state(message).state, interfaces::TrafficLight::UNKNOWN);
    EXPECT_EQ(state(message).header.frame_id, "camera_7_optical_frame");
  }
}

TEST(LightPolicy, CutoffRejectsStaleGreen) {
  for (const auto late : {Kind::Stale, Kind::ReceiveTimeout}) {
    EXPECT_FALSE((settle<velocity_planner::StopLineInputs, velocity_planner::RemoveInvalid>(
      light(interfaces::TrafficLight::GREEN), {late})));
  }
}

TEST(LightPolicy, StaleUnknownAndFreshLightPassUnchanged) {
  const auto stale_unknown = settle<velocity_planner::StopLineInputs, velocity_planner::RemoveInvalid>(
    light(interfaces::TrafficLight::UNKNOWN), {Kind::Stale, Kind::ReceiveTimeout});
  ASSERT_TRUE(stale_unknown);
  EXPECT_EQ(state(stale_unknown).state, interfaces::TrafficLight::UNKNOWN);
  const auto fresh = settle<velocity_planner::CurvatureInputs, velocity_planner::MissingLightUnknown>(
    light(interfaces::TrafficLight::RED), {Kind::Skew});
  ASSERT_TRUE(fresh);
  EXPECT_EQ(state(fresh).state, interfaces::TrafficLight::RED);
  EXPECT_EQ(state(fresh).header.stamp, ros::Time(100.0));
}

TEST(LightPolicy, InjectStillFillsMissingLight) {
  std::shared_ptr<const void> message;
  node_pipeline::EditableInput input(message, std::type_index(typeid(interfaces::TrafficLight)));
  velocity_planner::MissingLightUnknown{}.correct(input, Kind::Missing);
  ASSERT_TRUE(message);
  EXPECT_EQ(state(message).state, interfaces::TrafficLight::UNKNOWN);
}

TEST(LightPolicy, OutRangeCorrectionLeavesOtherTypesRemoved) {
  std::shared_ptr<const void> message = std::make_shared<interfaces::Path>();
  node_pipeline::EditableInput input(message, std::type_index(typeid(interfaces::Path)));
  velocity_planner::MissingLightUnknown{}.correct(input, Kind::OutRangePositive);
  EXPECT_FALSE(message);
}
