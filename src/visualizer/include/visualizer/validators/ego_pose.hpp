#pragma once
#include <nav_msgs/Odometry.h>
#include <node_pipeline/extension.hpp>
#include <cmath>

namespace visualizer {
struct EgoPose {
  void operator()(const nav_msgs::Odometry& input, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const auto& q = input.pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-6)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
};
}  // namespace visualizer
