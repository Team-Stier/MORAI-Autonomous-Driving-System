#pragma once
#include <node_pipeline/components.hpp>
#include <speed_filter/core_set.hpp>
namespace speed_filter {
namespace np = node_pipeline;
enum class Input { Command, Competition };
template<Input I> using InputTag = std::integral_constant<Input, I>;
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
  void operator()(const morai_msgs::CtrlCmd&, const np::HealthLimits&, std::vector<np::Kind>&) const {}
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, morai_msgs::CtrlCmd, interfaces::CompetitionVehicleStatus>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 2, Validator<Access>>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = np::CoreSelector<Input, CoreId>;
  using CoreSet = speed_filter::CoreSet;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, Checker, Policy, Selector, CoreSet, HeaderInjector, IO>;
};
}  // namespace speed_filter
