#pragma once
#include <object_detector/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <vector>

namespace object_detector {
struct UtoniaClusters {
  void operator()(const Clusters& clusters,const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& cloud : clusters) {
      if (cloud.size()<6) issues.push_back(node_pipeline::Kind::OutRangeNegative);
      if (cloud.size()>5000) issues.push_back(node_pipeline::Kind::OutRangePositive);
      for (const auto& point : cloud) for (double value : {point.x,point.y,point.z}) {
        if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
        else if (!std::isfinite(value)) issues.push_back(node_pipeline::Kind::Inf);
        // The pinned model's fixed 5cm 16-bit grid covers this map-coordinate domain.
        else if (std::abs(value)>1000.) issues.push_back(node_pipeline::Kind::OutRangePositive);
      }
    }
  }
};
}  // namespace object_detector
