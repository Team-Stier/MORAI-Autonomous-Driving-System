#pragma once
#include <node_pipeline/pipeline.hpp>
#include <traffic_light/core_dispatch.hpp>
namespace traffic_light {
namespace np = node_pipeline;
enum class Input { Image };
template<Input I> using InputTag = std::integral_constant<Input, I>;
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, sensor_msgs::CompressedImage>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 1, Validator<Access>>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = np::CoreSelector<Input, CoreId>;
  using CoreSet = traffic_light::CoreSet;
  using Stage = np::Stage<CoreSet, typename Buffer::ContextType>;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, CoreSet, HeaderInjector, IO>;
};
}  // namespace traffic_light
