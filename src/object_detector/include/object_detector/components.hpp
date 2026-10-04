#pragma once
#include <node_pipeline/pipeline.hpp>
#include <object_detector/core_dispatch.hpp>
namespace object_detector {
namespace np = node_pipeline;
enum class Input { Cloud };
template<Input I> using InputTag = std::integral_constant<Input, I>;
enum class Stage2Input { Objects };
struct Selector {
  np::CoreSelector<Input, FirstCoreId> stage1;
  np::CoreSelector<Stage2Input, SecondCoreId> stage2;
};
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
  void operator()(const interfaces::Objects& m, const np::HealthLimits&, std::vector<np::Kind>& issues) const {
    for (const auto& object : m.objects) {
      for (double value : {object.position.x, object.position.y, object.position.z}) {
        if (std::isnan(value)) issues.push_back(np::Kind::NaN);
        else if (std::isinf(value)) issues.push_back(np::Kind::Inf);
      }
    }
  }
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, sensor_msgs::PointCloud2>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 1, Validator<Access>>;
  using Stage2Checker = np::HealthChecker<Stage2Input, 1, Validator<Access>>;
  using Stage2Policy = np::InputPolicy<Stage2Input, np::RemoveInput, np::RemoveInput>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = object_detector::Selector;
  using CoreSet = object_detector::CoreSet;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, CoreSet, HeaderInjector, IO>;
};
}  // namespace object_detector
