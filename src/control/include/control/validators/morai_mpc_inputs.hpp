#pragma once
#include <node_pipeline/extension.hpp>
#include <interfaces/Path.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace control {
// morai_mpc.hpp의 입력 전제다. 경로 형상·목표속도·quaternion·point frame과 ego child frame·quaternion,
// 횡방향 모델 속도로 쓰는 ego twist.twist.linear.x의 NaN·Inf를 검사한다. 목표속도 범위는 node.yaml path range다.
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
    if (m.points.size() < 2) issues.push_back(node_pipeline::Kind::Missing);
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
};
}  // namespace control
