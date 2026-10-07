#pragma once
#include <velocity_planner/types.hpp>
#include <node_pipeline/sdk.hpp>
namespace velocity_planner { namespace validators {
struct Transform {
  void operator()(const node_pipeline::InputView& inputs, node_pipeline::ProfileHealth& health) const {
    if (!inputs.present("tf")) return;
    const auto& transforms = inputs.get<tf2_msgs::TFMessage>("tf").transforms;
    if (std::none_of(transforms.begin(), transforms.end(), [](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
      })) health.issues("tf").push_back(node_pipeline::Kind::Missing);
  }
};
}}
