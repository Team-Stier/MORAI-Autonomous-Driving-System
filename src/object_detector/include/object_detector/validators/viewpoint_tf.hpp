#pragma once
#include <node_pipeline/extension.hpp>
#include <tf2_msgs/TFMessage.h>
#include <cmath>
#include <vector>

namespace object_detector {
struct ViewpointTf {
  void operator()(const tf2_msgs::TFMessage& tf, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const geometry_msgs::TransformStamped* selected = nullptr;
    for (const auto& transform : tf.transforms) {
      if (transform.child_frame_id != "base_link") continue;
      if (selected) { issues.push_back(node_pipeline::Kind::Missing); return; }
      selected = &transform;
    }
    if (!selected) { issues.push_back(node_pipeline::Kind::Missing); return; }
    if (selected->header.frame_id != "map") issues.push_back(node_pipeline::Kind::FrameMismatch);
    if (selected->header.stamp != tf.transforms.front().header.stamp)
      issues.push_back(node_pipeline::Kind::InvalidStamp);
    const auto& t = selected->transform.translation;
    const auto& q = selected->transform.rotation;
    for (const double value : {t.x, t.y, t.z, q.x, q.y, q.z, q.w}) {
      if (std::isnan(value)) { issues.push_back(node_pipeline::Kind::NaN); return; }
      if (std::isinf(value)) { issues.push_back(node_pipeline::Kind::Inf); return; }
    }
    if (std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 1e-6)
      issues.push_back(node_pipeline::Kind::Missing);
  }
};
}  // namespace object_detector
