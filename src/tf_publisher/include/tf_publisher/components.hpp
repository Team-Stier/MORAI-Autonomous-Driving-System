#pragma once
#include <node_pipeline/pipeline.hpp>
#include <tf_publisher/core_dispatch.hpp>
namespace tf_publisher {
namespace np = node_pipeline;
enum class Input { Ego };
template<Input I> using InputTag = std::integral_constant<Input, I>;
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
  void operator()(const tf2_msgs::TFMessage&, const np::HealthLimits&, std::vector<np::Kind>&) const {}
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, nav_msgs::Odometry>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 1, Validator<Access>>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = np::CoreSelector<Input, CoreId>;
  using CoreSet = tf_publisher::CoreSet;
  using Stage = np::Stage<CoreSet, typename Buffer::ContextType>;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, CoreSet, HeaderInjector, IO>;
};
}  // namespace tf_publisher
