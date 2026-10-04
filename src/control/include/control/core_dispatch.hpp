#pragma once
#include <control/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace control {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::PathTracking:
      if (!data.hasInputs({0, 1, 2})) return false;
      before();
      result = node_pipeline::wrapResult(cores.path_tracking.process(
        data.message<interfaces::Path>(0), data.message<nav_msgs::Odometry>(1),
        data.message<interfaces::CompetitionVehicleStatus>(2)));
      return true;
  }
  throw std::invalid_argument("unknown control core");
}
}
