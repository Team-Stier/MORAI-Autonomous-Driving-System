#pragma once
#include <node_pipeline/extension.hpp>
#include <interfaces/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace control {
// #MPC_TIME_CONTRACT
// 기존 geometry/frame/velocity 검사를 유지하며 시간 계획을 추가로 요구한다.
// 다른 Core와 공유하는 Validator를 바꾸지 않으며 점 위치가 같아도 시간은 증가해야 한다.
struct MoraiMpcInputs {
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
    // #EXACT_PATH_FRAME: 실제 변환이 없는 경로를 identity로 가정하지 않는다.
    const auto& frame=m.map_from_path_frame;
    if(frame.header.frame_id!="map" || frame.child_frame_id!=limits.frame)
      issues.push_back(node_pipeline::Kind::FrameMismatch);
    if(frame.header.stamp.isZero()) issues.push_back(node_pipeline::Kind::InvalidStamp);
    numbers({frame.transform.translation.x,frame.transform.translation.y,
             frame.transform.translation.z},issues);
    quaternion(frame.transform.rotation,issues);
    if (m.points.size() < 2) issues.push_back(node_pipeline::Kind::Missing);
    if (!m.points.empty() && m.points.front().time_from_start.toNSec() != 0)
      issues.push_back(node_pipeline::Kind::InvalidStamp);
    for (std::size_t i = 1; i < m.points.size(); ++i) {
      if (m.points[i].time_from_start <= m.points[i-1].time_from_start)
        issues.push_back(node_pipeline::Kind::InvalidStamp);
    }
    bool distinct = false;
    for (std::size_t i = 0; i < m.points.size(); ++i) {
      const auto& point = m.points[i];
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
    if (!distinct) issues.push_back(node_pipeline::Kind::Missing);
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
