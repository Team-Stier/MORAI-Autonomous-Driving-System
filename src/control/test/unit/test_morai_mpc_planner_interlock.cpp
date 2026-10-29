#include <control/validators/morai_mpc_inputs.hpp>
#include <gtest/gtest.h>
#include <algorithm>

namespace {
using node_pipeline::Kind;
std::vector<Kind> issues(const interfaces::Path& path,const std::string& frame) {
  node_pipeline::HealthLimits limits;limits.frame=frame;limits.lower=0;limits.upper=111.11111111111111;
  std::vector<Kind> result;control::MoraiMpcInputs{}(path,limits,result);return result;
}
TEST(MoraiMpcPlannerInterlock, EmptyOriginalMapPathIsMissing) {
  const auto result=issues(interfaces::Path{},"map");
  EXPECT_NE(std::find(result.begin(),result.end(),Kind::Missing),result.end());
}
TEST(MoraiMpcPlannerInterlock, OriginalMapPathDoesNotRequireInjectedFrameOrTimeFields) {
  interfaces::Path path;path.points.resize(2);
  EXPECT_TRUE(issues(path,"map").empty());
}
TEST(MoraiMpcPlannerInterlock, InjectedPathStillRequiresTheExactTransformAndTiming) {
  interfaces::Path path;path.points.resize(2);path.points[1].pose.pose.position.x=1;
  for(auto& point:path.points) {point.pose.header.frame_id="base_link";point.pose.pose.orientation.w=1;point.velocity=1;}
  const auto result=issues(path,"base_link");
  EXPECT_NE(std::find(result.begin(),result.end(),Kind::FrameMismatch),result.end());
  EXPECT_NE(std::find(result.begin(),result.end(),Kind::InvalidStamp),result.end());
}
TEST(MoraiMpcPlannerInterlock, ValidInjectedPathKeepsTheExistingContract) {
  interfaces::Path path;path.points.resize(2);path.points[1].pose.pose.position.x=1;path.points[1].time_from_start=ros::Duration(1);
  for(auto& point:path.points) {point.pose.header.frame_id="base_link";point.pose.pose.orientation.w=1;point.velocity=1;}
  auto& tf=path.map_from_path_frame;tf.header.frame_id="map";tf.child_frame_id="base_link";tf.header.stamp=ros::Time(1);tf.transform.rotation.w=1;
  EXPECT_TRUE(issues(path,"base_link").empty());
}
}  // namespace
