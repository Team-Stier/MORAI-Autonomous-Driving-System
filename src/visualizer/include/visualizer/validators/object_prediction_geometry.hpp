#pragma once
#include <interfaces/Objects.h>
#include <node_pipeline/extension.hpp>
#include <cmath>

namespace visualizer {
struct ObjectPredictionGeometry {
  void operator()(const interfaces::Objects& input, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& object : input.objects) {
      const auto& size = object.bbox_size;
      if (!std::isfinite(size.x) || !std::isfinite(size.y) || !std::isfinite(size.z))
        issues.push_back(node_pipeline::Kind::NaN);
      else if (size.x <= 0 || size.y <= 0 || size.z <= 0)
        issues.push_back(node_pipeline::Kind::OutRangePositive);
      const auto check_pose = [&](const geometry_msgs::Pose& pose) {
        const auto& p = pose.position;
        const auto& q = pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm))
          issues.push_back(node_pipeline::Kind::NaN);
        else if (std::abs(norm - 1.0) > 1e-6)
          issues.push_back(node_pipeline::Kind::OutRangePositive);
      };
      check_pose(object.bbox_pose);
      if (object.prediction_available)
        for (const auto& pose : object.predicted_bbox_poses) check_pose(pose);
    }
  }
};
}  // namespace visualizer
