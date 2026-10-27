#pragma once
#include <node_pipeline/extension.hpp>
#include <interfaces/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <cmath>
#include <algorithm>
#include <initializer_list>
#include <vector>

namespace control {
// 경로 형상·목표속도와 자세 quaternion을 검사한다. 목표속도 범위는 node.yaml path range다.
struct IntegratedMpcInputs {
  static void numbers(std::initializer_list<double> values, std::vector<node_pipeline::Kind>& issues) {
    for (double value : values) {
      if (std::isnan(value)) { issues.push_back(node_pipeline::Kind::NaN); return; }
      if (std::isinf(value)) { issues.push_back(node_pipeline::Kind::Inf); return; }
    }
  }
  static void quaternion(const geometry_msgs::Quaternion& q, std::vector<node_pipeline::Kind>& issues) {
    numbers({q.x, q.y, q.z, q.w}, issues);
    if (std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 1e-6)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
  void operator()(const interfaces::Path& m, const node_pipeline::HealthLimits& limits,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (m.points.size() < 2) issues.push_back(node_pipeline::Kind::Missing);
    const bool timed=std::any_of(m.points.begin(),m.points.end(),
        [](const interfaces::PathPoint& p){return p.time_from_start.toSec()!=0;});
    if(timed && !m.points.empty() && m.points.front().time_from_start.toSec()!=0)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
    bool distinct = false;
    for (std::size_t i = 0; i < m.points.size(); ++i) {
      const auto& point = m.points[i];
      if(point.time_from_start.toSec()<0 || (timed && i>0 &&
          point.time_from_start<=m.points[i-1].time_from_start))
        issues.push_back(node_pipeline::Kind::OutRangePositive);
      const auto& position = point.pose.pose.position;
      numbers({position.x, position.y, position.z, point.velocity, point.acceleration}, issues);
      quaternion(point.pose.pose.orientation, issues);
      if (point.pose.header.frame_id != limits.frame) issues.push_back(node_pipeline::Kind::FrameMismatch);
      if (point.velocity < limits.lower) issues.push_back(node_pipeline::Kind::OutRangeNegative);
      if (point.velocity > limits.upper) issues.push_back(node_pipeline::Kind::OutRangePositive);
      if (i > 0) {
        const auto& previous = m.points[i - 1].pose.pose.position;
        distinct = distinct || std::hypot(position.x - previous.x, position.y - previous.y) > 1e-6;
      }
    }
    if(!m.lateral_reference.empty() && m.lateral_reference.size()<2)issues.push_back(node_pipeline::Kind::Missing);
    for(const auto& pose:m.lateral_reference) {
      numbers({pose.position.x,pose.position.y,pose.position.z},issues);
      quaternion(pose.orientation,issues);
    }
    const bool stationary_wait=timed && std::all_of(m.points.begin(),m.points.end(),
        [](const interfaces::PathPoint& p){return p.velocity==0 && p.acceleration==0;});
    if (!distinct && !stationary_wait) issues.push_back(node_pipeline::Kind::Missing);
  }
  void operator()(const nav_msgs::Odometry& m, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (m.child_frame_id != "base_link") issues.push_back(node_pipeline::Kind::FrameMismatch);
    quaternion(m.pose.pose.orientation, issues);
    numbers({m.twist.twist.linear.x}, issues);
  }
  void operator()(const sensor_msgs::Imu& m, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    numbers({m.linear_acceleration.x}, issues);
  }
  void operator()(const interfaces::CompetitionVehicleStatus& m, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    numbers({m.velocity_x_mps, m.steering_angle_rad}, issues);
  }
};
}  // namespace control
