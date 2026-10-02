#pragma once
#include <node_pipeline/components.hpp>
#include <object_detector/core_set.hpp>
namespace object_detector {
namespace np = node_pipeline;
enum class Input { Cloud };
template<Input I> using InputTag = std::integral_constant<Input, I>;
struct Selector {
  np::CoreSelector<Input, FirstCoreId> core1;
  np::CoreSelector<Input, SecondCoreId> core2;
  template<class Health> np::Selection<CoreId> select(std::uint8_t section, const Health& health) const {
    const auto first = core1.select(section, health);
    const auto second = core2.select(section, health);
    return {first.selected && second.selected, {first.id, second.id}};
  }
};
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, sensor_msgs::PointCloud2>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 1, Validator<Access>>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = object_detector::Selector;
  using CoreSet = object_detector::CoreSet;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, Checker, Policy, Selector, CoreSet, HeaderInjector, IO>;
};
}  // namespace object_detector
