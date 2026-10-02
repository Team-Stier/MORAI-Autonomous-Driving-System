#pragma once
#include <node_pipeline/components.hpp>
#include <path_planner/core_set.hpp>
namespace path_planner {
namespace np = node_pipeline;
enum class Input { Ego, Competition, Objects, Tf, Section };
template<Input I> using InputTag = std::integral_constant<Input, I>;
struct Selector {
  np::CoreSelector<Input, FirstCoreId> core1;
  np::CoreSelector<Input, SecondCoreId> core2;
  template<class Health> np::Selection<CoreIds> select(std::uint8_t section, const Health& health) const {
    const auto first = core1.select(section, health);
    const auto second = core2.select(section, health);
    return {first.selected && second.selected, {first.id, second.id}};
  }
};
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
  void operator()(const tf2_msgs::TFMessage&, const np::HealthLimits&, std::vector<np::Kind>&) const {}
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::Objects, tf2_msgs::TFMessage, interfaces::SectionAnnotation>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, Buffer::count, Validator<Access>>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = path_planner::Selector;
  using CoreSet = path_planner::CoreSet;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, Checker, Policy, Selector, CoreSet, HeaderInjector, IO>;
};
}  // namespace path_planner
