#pragma once
#include <interfaces/Path.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <vector>

namespace velocity_planner {
struct VelocityInputs {
  void operator()(const interfaces::Path& path, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& point : path.points) {
      if (std::isinf(point.velocity)) issues.push_back(node_pipeline::Kind::Inf);
    }
  }
};
}  // namespace velocity_planner
