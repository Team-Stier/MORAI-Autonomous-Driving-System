#pragma once
#include <interfaces/TrafficLight.h>
#include <node_pipeline/extension.hpp>

namespace velocity_planner {
struct MissingLightUnknown {
  int priority(node_pipeline::Kind kind) const { return kind == node_pipeline::Kind::Missing ? 100 : -1; }
  void correct(node_pipeline::EditableInput& input, node_pipeline::Kind kind) const {
    if (kind != node_pipeline::Kind::Missing) return;
    interfaces::TrafficLight light;
    light.state = interfaces::TrafficLight::UNKNOWN;
    light.header.frame_id = "camera_7_optical_frame";
    input.fill(std::move(light));
  }
};
}  // namespace velocity_planner
