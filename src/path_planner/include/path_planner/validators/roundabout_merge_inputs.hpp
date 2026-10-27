#pragma once
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <unordered_set>
#include <vector>

namespace path_planner {
struct RoundaboutMergeInputs {
  void operator()(const nav_msgs::Odometry& ego, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    checkPose(ego.pose.pose, issues);
    const auto& velocity = ego.twist.twist.linear;
    const auto& angular_velocity = ego.twist.twist.angular;
    checkFinite({velocity.x, velocity.y, velocity.z,
                 angular_velocity.x, angular_velocity.y, angular_velocity.z}, issues);
  }

  void operator()(const interfaces::CompetitionVehicleStatus& status, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    checkFinite({status.velocity_x_mps, status.angular_velocity_x_radps,
                 status.angular_velocity_y_radps, status.angular_velocity_z_radps}, issues);
    if (status.velocity_x_mps < 0.0)
      issues.push_back(node_pipeline::Kind::OutRangeNegative);
  }

  void operator()(const interfaces::Objects& objects, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    checkObjects(objects, issues);
  }

  void operator()(const PredictedObjects& predictions, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const auto& reference = predictions.front();
    if (reference.header.frame_id != "map")
      issues.push_back(node_pipeline::Kind::FrameMismatch);
    if (reference.header.stamp.isZero())
      issues.push_back(node_pipeline::Kind::InvalidStamp);
    std::unordered_set<std::uint32_t> reference_ids;
    for (const auto& object : reference.objects) reference_ids.insert(object.id);
    for (const auto& objects : predictions) {
      checkObjects(objects, issues);
      if (objects.header.frame_id != reference.header.frame_id)
        issues.push_back(node_pipeline::Kind::FrameMismatch);
      if (objects.header.stamp != reference.header.stamp || objects.header.seq != reference.header.seq)
        issues.push_back(node_pipeline::Kind::InvalidStamp);
      if (objects.objects.size() != reference.objects.size())
        issues.push_back(node_pipeline::Kind::OutRangePositive);
      for (const auto& object : objects.objects)
        if (reference_ids.count(object.id) == 0)
          issues.push_back(node_pipeline::Kind::OutRangePositive);
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

  static void checkPose(const geometry_msgs::Pose& pose, std::vector<node_pipeline::Kind>& issues) {
    const auto& position = pose.position;
    const auto& q = pose.orientation;
    checkFinite({position.x, position.y, position.z, q.x, q.y, q.z, q.w}, issues);
    const double norm_squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1.0) > 1e-6)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }

  static void checkObjects(const interfaces::Objects& objects, std::vector<node_pipeline::Kind>& issues) {
    std::unordered_set<std::uint32_t> ids;
    for (const auto& object : objects.objects) {
      checkPose(object.bbox_pose, issues);
      const auto& size = object.bbox_size;
      checkFinite({size.x, size.y, size.z}, issues);
      if (size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0)
        issues.push_back(node_pipeline::Kind::OutRangeNegative);
      if (!ids.insert(object.id).second)
        issues.push_back(node_pipeline::Kind::OutRangePositive);
    }
  }
};
}  // namespace path_planner
