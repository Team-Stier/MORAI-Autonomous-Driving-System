#pragma once
#include <interfaces/Path.h>
#include <nav_msgs/Path.h>
#include <node_pipeline/extension.hpp>
#include <cmath>

namespace visualizer {
struct PathPoints {
  void operator()(const nav_msgs::Path& input, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& point : input.poses) {
      const auto& p = point.pose.position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        issues.push_back(node_pipeline::Kind::NaN);
      if (point.header.frame_id != input.header.frame_id)
        issues.push_back(node_pipeline::Kind::FrameMismatch);
    }
  }
  void operator()(const interfaces::Path& input, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& point : input.points) {
      const auto& p = point.pose.pose.position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(point.velocity))
        issues.push_back(node_pipeline::Kind::NaN);
      if (point.pose.header.frame_id != input.header.frame_id)
        issues.push_back(node_pipeline::Kind::FrameMismatch);
      if (point.pose.header.stamp != input.header.stamp)
        issues.push_back(node_pipeline::Kind::InvalidStamp);
    }
  }
};
}  // namespace visualizer
