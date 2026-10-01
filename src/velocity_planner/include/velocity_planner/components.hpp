#pragma once
#include <node_pipeline/components.hpp>
#include <velocity_planner/core_set.hpp>
namespace velocity_planner {
namespace np = node_pipeline;
enum class Input { Path, Ego, Competition, Light, Tf };
template<Input I> using InputTag = std::integral_constant<Input, I>;
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
  void operator()(const tf2_msgs::TFMessage&, const np::HealthLimits&, std::vector<np::Kind>&) const {}
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, interfaces::Path, geometry_msgs::PoseStamped, interfaces::CompetitionVehicleStatus, interfaces::TrafficLight, tf2_msgs::TFMessage>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 5, Validator<Access>>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = np::CoreSelector<Input, CoreId>;
  using CoreSet = velocity_planner::CoreSet;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, Checker, Policy, Selector, CoreSet, HeaderInjector, IO>;
};
}  // namespace velocity_planner
