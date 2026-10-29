#pragma once
#include <interfaces/Path.h>
#include <interfaces/TrafficLight.h>
#include <node_pipeline/extension.hpp>
#include <algorithm>
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
  // Stale·ReceiveTimeout인 light는 UNKNOWN만 허용 범위다.
  void operator()(const interfaces::TrafficLight& light, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const bool late = std::any_of(issues.begin(), issues.end(), [](node_pipeline::Kind kind) {
      return kind == node_pipeline::Kind::Stale || kind == node_pipeline::Kind::ReceiveTimeout;
    });
    if (late && light.state != interfaces::TrafficLight::UNKNOWN) issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
};
}  // namespace velocity_planner
