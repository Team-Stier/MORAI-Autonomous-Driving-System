#pragma once
#include <interfaces/Path.h>
#include <node_pipeline/extension.hpp>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace velocity_planner {
struct TfInputs {
  static void numbers(std::initializer_list<double> values, std::vector<node_pipeline::Kind>& issues) {
    for (double value : values) {
      if (std::isnan(value)) { issues.push_back(node_pipeline::Kind::NaN); return; }
      if (std::isinf(value)) { issues.push_back(node_pipeline::Kind::Inf); return; }
    }
  }
  static void quaternion(const geometry_msgs::Quaternion& q, std::vector<node_pipeline::Kind>& issues) {
    numbers({q.x, q.y, q.z, q.w}, issues);
    if (std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) > 1e-6)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
  void operator()(const interfaces::Path& path, const node_pipeline::HealthLimits& limits,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& point : path.points) {
      const auto& position = point.pose.pose.position;
      numbers({position.x, position.y, position.z}, issues);
      quaternion(point.pose.pose.orientation, issues);
      if (point.pose.header.frame_id != limits.frame) issues.push_back(node_pipeline::Kind::FrameMismatch);
    }
    for(const auto& pose:path.lateral_reference) {
      numbers({pose.position.x,pose.position.y,pose.position.z},issues);
      quaternion(pose.orientation,issues);
    }
  }
  void operator()(const tf2_msgs::TFMessage& transforms, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const auto match = std::find_if(transforms.transforms.begin(), transforms.transforms.end(),
      [](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
      });
    if (match == transforms.transforms.end()) { issues.push_back(node_pipeline::Kind::Missing); return; }
    const auto& translation = match->transform.translation;
    numbers({translation.x, translation.y, translation.z}, issues);
    quaternion(match->transform.rotation, issues);
  }
};
}  // namespace velocity_planner
