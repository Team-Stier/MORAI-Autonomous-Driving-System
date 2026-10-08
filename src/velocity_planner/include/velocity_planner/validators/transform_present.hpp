#pragma once
#include <node_pipeline/extension.hpp>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <vector>

namespace velocity_planner {
// 경로 변환 Core가 전제하는 map → base_link 변환이 /tf 메시지에 있는지 검사한다.
struct TransformPresent {
  void operator()(const tf2_msgs::TFMessage& tf, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (std::none_of(tf.transforms.begin(), tf.transforms.end(), [](const geometry_msgs::TransformStamped& transform) {
          return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
        }))
      issues.push_back(node_pipeline::Kind::Missing);
  }
};
}  // namespace velocity_planner
