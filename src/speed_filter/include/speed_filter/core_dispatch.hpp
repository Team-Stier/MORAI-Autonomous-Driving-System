#pragma once
#include <speed_filter/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace speed_filter {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::SpeedLimit60:
      if (!data.hasInputs({0, 1})) return false;
      before();
      result = node_pipeline::wrapResult(cores.speed_limit60.process(
        data.message<morai_msgs::CtrlCmd>(0), data.message<interfaces::CompetitionVehicleStatus>(1)), {0});
      return true;
    case CoreId::NoSpeedLimit:
      if (!data.hasInputs({0, 1})) return false;
      before();
      result = node_pipeline::wrapResult(cores.no_speed_limit.process(
        data.message<morai_msgs::CtrlCmd>(0), data.message<interfaces::CompetitionVehicleStatus>(1)), {0});
      return true;
  }
  throw std::invalid_argument("unknown speed filter core");
}
}  // namespace speed_filter
