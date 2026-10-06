#pragma once
#include <traffic_light/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace traffic_light {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::YoloTrafficLight:
      if (!data.hasInputs({0})) return false;
      before();
      result = node_pipeline::wrapResult(
          cores.yolo_traffic_light.process(data.message<sensor_msgs::CompressedImage>(0)));
      return true;
  }
  throw std::invalid_argument("unknown traffic light core");
}
}  // namespace traffic_light
