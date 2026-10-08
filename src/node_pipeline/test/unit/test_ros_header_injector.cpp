// RosHeaderInjector가 context stamp와 output.frame_id만 주입하는지 검증한다.
#include <node_pipeline/detail/ros.hpp>
#include <gtest/gtest.h>

namespace npd = node_pipeline::detail;

TEST(RosHeaderInjector, InjectsContextStampAndOutputFrame) {
  interfaces::SectionAnnotation message;
  message.state = 3;
  npd::Context context;
  context.stamp_ns = 4200;
  npd::RosHeaderInjector<interfaces::SectionAnnotation>{"map"}.injectHeader(message, context);
  EXPECT_EQ(4200u, message.header.stamp.toNSec());
  EXPECT_EQ("map", message.header.frame_id);
  EXPECT_EQ(3, message.state);
}

TEST(RosHeaderInjector, LatchedContextWithoutReferenceSampleKeepsStampZero) {
  interfaces::SectionAnnotation message;
  npd::RosHeaderInjector<interfaces::SectionAnnotation>{"base_link"}.injectHeader(message, npd::Context{});
  EXPECT_EQ(0u, message.header.stamp.toNSec());
  EXPECT_EQ("base_link", message.header.frame_id);
}
