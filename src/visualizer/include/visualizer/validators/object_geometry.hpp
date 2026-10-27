#pragma once
#include <interfaces/Objects.h>
#include <node_pipeline/extension.hpp>
#include <cmath>

namespace visualizer {
struct ObjectGeometry {
  void operator()(const interfaces::Objects& input, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& object : input.objects) {
      const auto& p = object.bbox_pose.position;
      const auto& q = object.bbox_pose.orientation;
      const auto& size = object.bbox_size;
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) ||
          !std::isfinite(size.x) || !std::isfinite(size.y) || !std::isfinite(size.z))
        issues.push_back(node_pipeline::Kind::NaN);
      else if (std::abs(norm - 1.0) > 1e-6 || size.x <= 0 || size.y <= 0 || size.z <= 0)
        issues.push_back(node_pipeline::Kind::OutRangePositive);
    }
  }
};
}  // namespace visualizer
