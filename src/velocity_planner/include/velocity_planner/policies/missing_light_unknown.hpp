#pragma once
#include <interfaces/TrafficLight.h>
#include <node_pipeline/extension.hpp>

namespace velocity_planner {
// Missing light와 범위 밖 light를 UNKNOWN으로 대체한다. Stale·ReceiveTimeout인 light는 Validator가 UNKNOWN 외 값을 범위 밖으로 표시한다.
struct MissingLightUnknown {
  int priority(node_pipeline::Kind kind) const {
    return kind == node_pipeline::Kind::Missing || kind == node_pipeline::Kind::OutRangePositive ? 100 : -1;
  }
  void correct(node_pipeline::EditableInput& input, node_pipeline::Kind kind) const {
    if (kind == node_pipeline::Kind::OutRangePositive) input.remove();
    else if (kind != node_pipeline::Kind::Missing) return;
    interfaces::TrafficLight light;
    light.state = interfaces::TrafficLight::UNKNOWN;
    light.header.frame_id = "camera_7_optical_frame";
    input.fill(std::move(light));
  }
};
}  // namespace velocity_planner
