#include <node_pipeline/detail/ros.hpp>
#include <nav_msgs/Odometry.h>
#include <interfaces/SectionAnnotation.h>
#include <gtest/gtest.h>
namespace np = node_pipeline::detail;
TEST(OutputContract, LegacySingleOutputRetainsPayloadAndInjectsHeader) {
  nav_msgs::Odometry ego;ego.pose.pose.position.x=12.5;
  const auto result=np::wrapResult(ego,{-1});
  auto message=np::OutputReader<nav_msgs::Odometry>::read(result(np::StageData{}));
  const auto injector=np::RosHeaderInjector<nav_msgs::Odometry>::create({{"/ego","map"}});
  np::Context context;context.stamp_ns=1234567890;
  injector.injectHeader(message,context);
  EXPECT_EQ(message.header.stamp.toNSec(),1234567890u);
  EXPECT_EQ(message.header.frame_id,"map");
  EXPECT_DOUBLE_EQ(message.pose.pose.position.x,12.5);
}
TEST(OutputContract, MultipleOutputsUseTheirOwnFramesAndTheSameStamp) {
  nav_msgs::Odometry ego;ego.child_frame_id="base_link";
  interfaces::SectionAnnotation status;status.state=15;
  using Output=std::tuple<nav_msgs::Odometry,interfaces::SectionAnnotation>;
  const auto result=np::wrapResult(Output{ego,status},{-1,-1});
  auto messages=np::OutputReader<Output>::read(result(np::StageData{}));
  const auto injector=np::RosHeaderInjector<Output>::create({{"/ego","map"},{"/state","map"}});
  np::Context context;context.stamp_ns=9876543210;
  injector.injectHeader(messages,context);
  EXPECT_EQ(std::get<0>(messages).header.stamp,std::get<1>(messages).header.stamp);
  EXPECT_EQ(std::get<1>(messages).header.stamp.toNSec(),9876543210u);
  EXPECT_EQ(std::get<0>(messages).child_frame_id,"base_link");
  EXPECT_EQ(std::get<1>(messages).state,15);
}
TEST(OutputContract, MissingFinalPayloadIsRejectedBeforeHeaderInjection) {
  using Output=std::tuple<nav_msgs::Odometry,interfaces::SectionAnnotation>;
  const auto result=np::wrapResult(nav_msgs::Odometry{}, {-1});
  EXPECT_THROW(np::OutputReader<Output>::read(result(np::StageData{})),std::invalid_argument);
  EXPECT_THROW(np::RosHeaderInjector<Output>::create({{"/ego","map"}}),std::invalid_argument);
}
