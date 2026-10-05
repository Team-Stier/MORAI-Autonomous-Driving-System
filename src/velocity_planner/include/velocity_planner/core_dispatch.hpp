#pragma once
#include <velocity_planner/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace velocity_planner {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::PlainVelocity60:
      if (!data.hasInputs({0, 4})) return false;
      before();
      result = node_pipeline::wrapResult(cores.plain_velocity60.process(
        data.message<interfaces::Path>(0), data.message<tf2_msgs::TFMessage>(4)), {0});
      return true;
    case CoreId::PlainVelocity100:
      if (!data.hasInputs({0, 4})) return false;
      before();
      result = node_pipeline::wrapResult(cores.plain_velocity100.process(
        data.message<interfaces::Path>(0), data.message<tf2_msgs::TFMessage>(4)), {0});
      return true;
  }
  throw std::invalid_argument("unknown core");
}
}  // namespace velocity_planner
