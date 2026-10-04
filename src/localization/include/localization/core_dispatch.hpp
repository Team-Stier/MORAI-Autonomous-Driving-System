#pragma once
#include <localization/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace localization {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::FullEkf:
      if (!data.hasInputs({0, 1, 2})) return false;
      before();
      result = node_pipeline::wrapResult(cores.full_ekf.process(
        data.message<sensor_msgs::NavSatFix>(0), data.message<sensor_msgs::Imu>(1),
        data.message<interfaces::CompetitionVehicleStatus>(2)));
      return true;
  }
  throw std::invalid_argument("unknown localization core");
}
}
