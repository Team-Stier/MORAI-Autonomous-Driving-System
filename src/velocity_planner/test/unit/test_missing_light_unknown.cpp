#include <velocity_planner/policies/missing_light_unknown.hpp>
#include <interfaces/Path.h>
#include <interfaces/TrafficLight.h>
#include <node_pipeline/detail/components.hpp>
#include <gtest/gtest.h>

using Policy = velocity_planner::MissingLightUnknown;
namespace np = node_pipeline;
namespace npd = node_pipeline::detail;

TEST(MissingLightUnknown, SuppliesUnknownForMissingTrafficLight) {
  npd::StageData data;
  data.inputs.push_back({{}, typeid(interfaces::TrafficLight)});
  npd::Health<1> health;
  health.inputs[0].issues = {np::Kind::Missing, np::Kind::ReceiveTimeout};
  EXPECT_TRUE(npd::InputPolicy::of<Policy>({0}).apply(data.typed<npd::Data<interfaces::TrafficLight>>(), health));
  ASSERT_TRUE(data.inputs[0].present());
  const auto& light = data.message<interfaces::TrafficLight>(0);
  EXPECT_EQ(0, light.state);
  EXPECT_EQ("camera_7_optical_frame", light.header.frame_id);
  EXPECT_TRUE(light.header.stamp.isZero());
  EXPECT_FALSE(data.inputs[0].has_stamp);
  EXPECT_FALSE(data.inputs[0].has_receipt);
}

TEST(MissingLightUnknown, KeepsOtherRequiredInputsMissing) {
  std::shared_ptr<const void> message;
  np::EditableInput input(message, typeid(interfaces::Path));
  Policy{}.correct(input, np::Kind::Missing);
  EXPECT_FALSE(message);
}

TEST(MissingLightUnknown, DoesNotReplaceOrRemoveReceivedTrafficLight) {
  auto light = std::make_shared<interfaces::TrafficLight>();
  light->state = interfaces::TrafficLight::GREEN;
  light->header.frame_id = "wrong_frame";
  npd::StageData data;
  data.inputs.push_back({light, typeid(interfaces::TrafficLight)});
  const auto policy = npd::InputPolicy::of<Policy>({0});
  npd::Health<1> health;
  for (const auto issue : {np::Kind::InvalidStamp, np::Kind::FutureStamp, np::Kind::FrameMismatch,
                          np::Kind::NaN, np::Kind::Inf, np::Kind::OutRangePositive,
                          np::Kind::OutRangeNegative, np::Kind::Stale, np::Kind::ReceiveTimeout, np::Kind::Skew}) {
    health.inputs[0].issues = {issue};
    EXPECT_FALSE(policy.apply(data.typed<npd::Data<interfaces::TrafficLight>>(), health));
    EXPECT_EQ(light, data.inputs[0].message);
  }
  np::EditableInput input(data.inputs[0].message, typeid(interfaces::TrafficLight));
  Policy{}.correct(input, np::Kind::Missing);
  EXPECT_EQ(light, data.inputs[0].message);
}
