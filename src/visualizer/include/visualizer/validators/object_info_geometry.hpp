#pragma once
#include <morai_msgs/ObjectStatusList.h>
#include <nav_msgs/Odometry.h>
#include <node_pipeline/extension.hpp>
#include <cmath>

namespace visualizer {
struct ObjectInfoGeometry {
  void operator()(const nav_msgs::Odometry& input, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const auto& q = input.pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-6)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
  void operator()(const morai_msgs::ObjectStatusList& input, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto* objects : {&input.npc_list, &input.pedestrian_list, &input.obstacle_list}) {
      for (const auto& object : *objects) {
        const auto& p = object.position;
        const auto& size = object.size;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(object.heading) || !std::isfinite(size.x) ||
            !std::isfinite(size.y) || !std::isfinite(size.z))
          issues.push_back(node_pipeline::Kind::NaN);
        else if (size.x <= 0 || size.y <= 0 || size.z <= 0)
          issues.push_back(node_pipeline::Kind::OutRangePositive);
      }
    }
  }
};
}  // namespace visualizer
