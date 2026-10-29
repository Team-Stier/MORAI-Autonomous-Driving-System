#pragma once
#include <nav_msgs/Odometry.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <vector>

namespace path_planner {
struct RddfEgo {
  void operator()(const nav_msgs::Odometry& ego, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (double value : {ego.pose.pose.position.x, ego.pose.pose.position.y}) {
      if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
      else if (!std::isfinite(value)) issues.push_back(node_pipeline::Kind::Inf);
    }
  }
};
}  // namespace path_planner
