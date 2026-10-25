#pragma once
#include <interfaces/Objects.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace path_planner {
struct BboxPredictionInputs {
  void operator()(const interfaces::Objects& objects, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& object : objects.objects) {
      if (!object.prediction_available) {
        issues.push_back(node_pipeline::Kind::Missing);
        continue;
      }
      const auto& size = object.bbox_size;
      checkFinite({size.x, size.y, size.z}, issues);
      if (size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0)
        issues.push_back(node_pipeline::Kind::OutRangeNegative);
      for (const auto& pose : object.predicted_bbox_poses) {
        const auto& position = pose.position;
        const auto& q = pose.orientation;
        checkFinite({position.x, position.y, position.z, q.x, q.y, q.z, q.w}, issues);
        const double norm_squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1.0) > 1e-6)
          issues.push_back(node_pipeline::Kind::OutRangePositive);
      }
    }
  }

 private:
  static void checkFinite(std::initializer_list<double> values,
                          std::vector<node_pipeline::Kind>& issues) {
    for (double value : values) {
      if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
      else if (!std::isfinite(value)) issues.push_back(node_pipeline::Kind::Inf);
    }
  }
};
}  // namespace path_planner
