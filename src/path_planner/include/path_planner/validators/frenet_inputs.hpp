#pragma once
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace path_planner {
struct FrenetInputs {
  void operator()(const nav_msgs::Odometry& ego, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    checkPose(ego.pose.pose, issues);
  }

  void operator()(const interfaces::Objects& objects, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& object : objects.objects) {
      checkPose(object.bbox_pose, issues);
      const auto& size = object.bbox_size;
      checkFinite({size.x, size.y, size.z}, issues);
      if (size.x < 0.0 || size.y < 0.0 || size.z < 0.0)
        issues.push_back(node_pipeline::Kind::OutRangeNegative);
    }
  }

  void operator()(const Route& route, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (!route || route->empty()) {
      issues.push_back(node_pipeline::Kind::Missing);
      return;
    }
    for (const auto& lane : *route) {
      const bool left_valid = checkLine(lane.leftBound3d(), issues);
      const bool right_valid = checkLine(lane.rightBound3d(), issues);
      if (left_valid && right_valid) checkLine(lane.centerline3d(), issues);
    }
  }

 private:
  static bool checkFinite(std::initializer_list<double> values,
                          std::vector<node_pipeline::Kind>& issues) {
    bool valid = true;
    for (double value : values) {
      if (std::isnan(value)) {
        issues.push_back(node_pipeline::Kind::NaN);
        valid = false;
      } else if (!std::isfinite(value)) {
        issues.push_back(node_pipeline::Kind::Inf);
        valid = false;
      }
    }
    return valid;
  }

  static void checkPose(const geometry_msgs::Pose& pose, std::vector<node_pipeline::Kind>& issues) {
    const auto& position = pose.position;
    const auto& q = pose.orientation;
    checkFinite({position.x, position.y, position.z}, issues);
    if (!checkFinite({q.x, q.y, q.z, q.w}, issues)) return;
    const double norm_squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1.0) > 1e-6)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }

  template<class Line>
  static bool checkLine(const Line& line, std::vector<node_pipeline::Kind>& issues) {
    bool valid = true;
    bool nondegenerate = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
      const auto& point = line[i];
      if (!checkFinite({point.x(), point.y(), point.z()}, issues)) valid = false;
      if (i > 0 && (point.x() != line[i - 1].x() || point.y() != line[i - 1].y()))
        nondegenerate = true;
    }
    if (line.size() < 2 || !nondegenerate) {
      issues.push_back(node_pipeline::Kind::OutRangePositive);
      valid = false;
    }
    return valid;
  }
};
}  // namespace path_planner
