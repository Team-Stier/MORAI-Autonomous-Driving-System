#pragma once
#include <node_pipeline/components.hpp>
#include <control/core_set.hpp>
#include <cmath>
namespace control {
namespace np = node_pipeline;
enum class Input { Path, Ego, Competition };
template<Input I> using InputTag = std::integral_constant<Input, I>;
template<class Access> struct Validator {
  double maximum_speed_mps = 60.0 / 3.6;
  static void numbers(std::initializer_list<double> values, std::vector<np::Kind>& issues) {
    for (double value : values) {
      if (std::isnan(value)) { issues.push_back(np::Kind::NaN); return; }
      if (std::isinf(value)) { issues.push_back(np::Kind::Inf); return; }
    }
  }
  static void quaternion(const geometry_msgs::Quaternion& q, std::vector<np::Kind>& issues) {
    numbers({q.x, q.y, q.z, q.w}, issues);
    if (std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 1e-6)
      issues.push_back(np::Kind::OutRangePositive);
  }
  void operator()(const interfaces::Path& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
    if (m.points.size() < 2) issues.push_back(np::Kind::Missing);
    bool distinct = false;
    for (std::size_t i = 0; i < m.points.size(); ++i) {
      const auto& point = m.points[i];
      const auto& position = point.pose.pose.position;
      numbers({position.x, position.y, position.z, point.velocity, point.acceleration}, issues);
      quaternion(point.pose.pose.orientation, issues);
      if (point.pose.header.frame_id != "base_link") issues.push_back(np::Kind::FrameMismatch);
      if (point.velocity < 0) issues.push_back(np::Kind::OutRangeNegative);
      if (point.velocity > maximum_speed_mps) issues.push_back(np::Kind::OutRangePositive);
      if (i > 0) {
        const auto& previous = m.points[i-1].pose.pose.position;
        distinct = distinct || std::hypot(position.x - previous.x, position.y - previous.y) > 1e-6;
      }
    }
    if (!distinct) issues.push_back(np::Kind::Missing);
  }
  void operator()(const nav_msgs::Odometry& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (Access::frame(m) != limits.frame || m.child_frame_id != "base_link") issues.push_back(np::Kind::FrameMismatch);
    const auto& p = m.pose.pose.position;
    numbers({p.x, p.y, p.z, m.twist.twist.linear.y, m.twist.twist.angular.z}, issues);
    quaternion(m.pose.pose.orientation, issues);
  }
  void operator()(const interfaces::CompetitionVehicleStatus& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (Access::frame(m) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
    numbers({m.velocity_x_mps}, issues);
  }
};
template<class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, interfaces::Path, nav_msgs::Odometry, interfaces::CompetitionVehicleStatus>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 3, Validator<Access>>;
  using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
  using Selector = np::CoreSelector<Input, CoreId>;
  using CoreSet = control::CoreSet;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() { return {}; }
  template<class IO, class HeaderInjector>
  using Node = np::PipelineNode<Buffer, Assembler, Checker, Policy, Selector, CoreSet, HeaderInjector, IO>;
};
}
