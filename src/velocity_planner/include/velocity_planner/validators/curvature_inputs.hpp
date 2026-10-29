#pragma once
#include <interfaces/Path.h>
#include <interfaces/TrafficLight.h>
#include <node_pipeline/extension.hpp>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace velocity_planner {
struct CurvatureInputs {
  void operator()(const interfaces::Path& path, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (path.points.empty()) issues.push_back(node_pipeline::Kind::OutRangeNegative);
    for (const auto& point : path.points) {
      if (std::isinf(point.velocity)) issues.push_back(node_pipeline::Kind::Inf);
      const auto& position = point.pose.pose.position;
      for (double value : {position.x, position.y}) {
        if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
        else if (std::isinf(value)) issues.push_back(node_pipeline::Kind::Inf);
      }
      if (point.pose.header.frame_id != "map") issues.push_back(node_pipeline::Kind::FrameMismatch);
    }
  }
  void operator()(const tf2_msgs::TFMessage& transforms, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const auto match = std::find_if(transforms.transforms.begin(), transforms.transforms.end(),
      [](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
      });
    if (match == transforms.transforms.end()) { issues.push_back(node_pipeline::Kind::Missing); return; }
    const auto& translation = match->transform.translation;
    for (double value : {translation.x, translation.y}) {
      if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
      else if (std::isinf(value)) issues.push_back(node_pipeline::Kind::Inf);
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
