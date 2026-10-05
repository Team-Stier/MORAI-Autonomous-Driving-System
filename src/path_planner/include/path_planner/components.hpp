#pragma once
#include <node_pipeline/pipeline.hpp>
#include <path_planner/core_dispatch.hpp>
#include <utility>
namespace path_planner {
namespace np = node_pipeline;
template<class Dispatch> auto retainInputs(Dispatch dispatch) {
  return [dispatch](CoreSet& cores, auto id, const np::StageData& data,
                    np::CoreResult& result, const np::BeforeCore& before) {
    if (!dispatch(cores, id, data, result, before)) return false;
    const auto addition = std::move(result);
    result = [addition](const np::StageData& input) {
      auto next = input;
      auto added = addition(input);
      for (auto& slot : added.inputs) next.inputs.push_back(std::move(slot));
      return next;
    };
    return true;
  };
}
enum class Input { Ego, Competition, Objects, Tf, Section };
template<Input I> using InputTag = std::integral_constant<Input, I>;
enum class Stage2Input { Ego, Competition, Objects, Tf, Section, Route };
enum class Stage3Input { Ego, Competition, Objects, Tf, Section, Route, Predictions };
struct Selector {
  np::CoreSelector<Input, FirstCoreId> stage1;
  np::CoreSelector<Stage2Input, SecondCoreId> stage2;
  np::CoreSelector<Stage3Input, ThirdCoreId> stage3;
};
template<class Access> struct Validator {
  template<class Message> void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
  }
  void operator()(const Route&, const np::HealthLimits&, std::vector<np::Kind>&) const {}
  void operator()(const PredictedObjects& predictions, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (!limits.frame.empty() && std::any_of(predictions.begin(), predictions.end(),
        [&](const interfaces::Objects& objects) { return Access::frame(objects) != limits.frame; }))
      issues.push_back(np::Kind::FrameMismatch);
  }
  void operator()(const tf2_msgs::TFMessage&, const np::HealthLimits&, std::vector<np::Kind>&) const {}
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::Objects, tf2_msgs::TFMessage, interfaces::SectionAnnotation>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, Buffer::count, Validator<Access>>;
  using Stage2Checker = np::HealthChecker<Stage2Input, 6, Validator<Access>>;
  using Stage3Checker = np::HealthChecker<Stage3Input, 7, Validator<Access>>;
  using Stage2Policy = np::InputPolicy<Stage2Input, np::RemoveInput, np::RemoveInput>;
  using Stage3Policy = np::InputPolicy<Stage3Input, np::RemoveInput, np::RemoveInput>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = path_planner::Selector;
  using CoreSet = path_planner::CoreSet;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, CoreSet, HeaderInjector, IO>;
};
}  // namespace path_planner
