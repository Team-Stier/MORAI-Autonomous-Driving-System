#pragma once
#include <interfaces/Objects.h>
#include <nav_msgs/Odometry.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <unordered_set>
#include <vector>

namespace path_planner {
struct NpcHistoryInputs {
  void operator()(const nav_msgs::Odometry& ego, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const auto& p = ego.pose.pose.position;
    const auto& q = ego.pose.pose.orientation;
    const auto& v = ego.twist.twist.linear;
    const auto& w = ego.twist.twist.angular;
    checkFinite({p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z}, issues);
    checkQuaternion(q, issues);
  }
  void operator()(const interfaces::Objects& objects, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    std::unordered_set<std::uint32_t> ids;
    for (const auto& object : objects.objects) {
      const auto& p = object.position;
      const auto& b = object.bbox_pose.position;
      const auto& q = object.bbox_pose.orientation;
      const auto& s = object.bbox_size;
      checkFinite({p.x,p.y,p.z,b.x,b.y,b.z,q.x,q.y,q.z,q.w,s.x,s.y,s.z}, issues);
      checkQuaternion(q, issues);
      if (s.x < 0.0 || s.y < 0.0 || s.z < 0.0) issues.push_back(node_pipeline::Kind::OutRangeNegative);
      if (!ids.insert(object.id).second || std::abs(p.x-b.x) > 1e-6 ||
          std::abs(p.y-b.y) > 1e-6 || std::abs(p.z-b.z) > 1e-6)
        issues.push_back(node_pipeline::Kind::OutRangePositive);
    }
  }
 private:
  static void checkFinite(std::initializer_list<double> values, std::vector<node_pipeline::Kind>& issues) {
    for (double value : values) {
      if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
      else if (!std::isfinite(value)) issues.push_back(node_pipeline::Kind::Inf);
    }
  }
  static void checkQuaternion(const geometry_msgs::Quaternion& q, std::vector<node_pipeline::Kind>& issues) {
    const double norm_squared = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm_squared) || std::abs(norm_squared-1.0) > 1e-3)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
};
}  // namespace path_planner
