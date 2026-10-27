#pragma once
#include <interfaces/Path.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <vector>

namespace velocity_planner {
struct StopLineInputs {
  void operator()(const interfaces::Path& path, const node_pipeline::HealthLimits& limits,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (path.points.size() < 2) issues.push_back(node_pipeline::Kind::Missing);
    for (const auto& point : path.points) {
      const auto& position = point.pose.pose.position;
      for (const double value : {position.x, position.y, point.velocity}) {
        if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
        else if (std::isinf(value)) issues.push_back(node_pipeline::Kind::Inf);
      }
      if (point.pose.header.frame_id != limits.frame) issues.push_back(node_pipeline::Kind::FrameMismatch);
    }
  }
};
}  // namespace velocity_planner
