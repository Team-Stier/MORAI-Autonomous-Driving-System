#pragma once
#include <lidar_leveler/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace lidar_leveler {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::RollPitchLeveler:
      if (!data.hasInputs({0, 1, 2})) return false;
      before();
      result = node_pipeline::wrapResult(cores.roll_pitch_leveler.process(
        data.message<sensor_msgs::PointCloud2>(0), data.message<interfaces::CompetitionVehicleStatus>(1),
        data.message<tf2_msgs::TFMessage>(2)), {0});
      return true;
    case CoreId::PassThrough:
      if (!data.hasInputs({0, 2})) return false;
      before();
      result = node_pipeline::wrapResult(cores.pass_through.process(
        data.message<sensor_msgs::PointCloud2>(0), data.message<tf2_msgs::TFMessage>(2)), {0});
      return true;
  }
  throw std::invalid_argument("unknown core");
}
}  // namespace lidar_leveler
