#pragma once
#include <localization/core_dispatch.hpp>
#include <node_pipeline/pipeline.hpp>
#include <node_pipeline/ros.hpp>
#include <nav_msgs/Odometry.h>
namespace localization {
namespace np = node_pipeline;
enum class Input { Gps, Imu, Competition, Lidar };
template<Input I> using InputTag = std::integral_constant<Input, I>;
struct Access {
  template<class Message> static std::int64_t stamp(const Message& message) { return message.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& message) { return message.header.frame_id; }
};
// 센서 payload 검사는 pipeline 경계에서만 한다. Core는 정상 입력을 계산한다.
struct Validator {
  static void numbers(std::initializer_list<double> values, std::vector<np::Kind>& issues) {
    for (double value : values) {
      if (std::isnan(value)) { issues.push_back(np::Kind::NaN); return; }
      if (std::isinf(value)) { issues.push_back(np::Kind::Inf); return; }
    }
  }
  void operator()(const sensor_msgs::NavSatFix& msg, const np::HealthLimits&, std::vector<np::Kind>& issues) const {
    if (msg.header.frame_id != "gps_link") issues.push_back(np::Kind::FrameMismatch);
    numbers({msg.latitude, msg.longitude, msg.altitude}, issues);
    if (msg.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX ||
        (msg.latitude == 0 && msg.longitude == 0 && msg.altitude == 0))
      issues.push_back(np::Kind::Missing);
    if (msg.latitude < -80 || msg.latitude > 84 || msg.longitude < -180 || msg.longitude > 180)
      issues.push_back(np::Kind::OutRangePositive);
  }
  void operator()(const sensor_msgs::Imu& msg, const np::HealthLimits&, std::vector<np::Kind>& issues) const {
    if (msg.header.frame_id != "imu_link") issues.push_back(np::Kind::FrameMismatch);
    numbers({msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z}, issues);
  }
  void operator()(const interfaces::CompetitionVehicleStatus& msg, const np::HealthLimits&, std::vector<np::Kind>& issues) const {
    if (msg.header.frame_id != "base_link") issues.push_back(np::Kind::FrameMismatch);
    numbers({msg.roll_rad, msg.pitch_rad, msg.yaw_rad, msg.velocity_x_mps, msg.angular_velocity_x_radps, msg.angular_velocity_y_radps, msg.angular_velocity_z_radps}, issues);
  }
};
using Buffer = np::InputBuffer<Input, Access, sensor_msgs::NavSatFix, sensor_msgs::Imu,
                               interfaces::CompetitionVehicleStatus, sensor_msgs::PointCloud2>;
// 점군은 보관만 한다. health와 시차 검사는 EKF의 세 입력에만 적용한다.
struct Checker {
  np::HealthChecker<Input, 3, Validator> checker;
  Checker(std::array<np::HealthLimits, 3> limits, std::int64_t started) : checker(limits, started) {}
  auto healthCheck(const Data& data, const np::Context<4>& context, const np::Clocks& clocks) const {
    np::Data<sensor_msgs::NavSatFix, sensor_msgs::Imu, interfaces::CompetitionVehicleStatus> inputs;
    inputs.inputs = std::make_tuple(data.get<0>(), data.get<1>(), data.get<2>());
    np::Context<3> subset;
    subset.section = context.section;
    std::copy_n(context.evictions.begin(), 3, subset.evictions.begin());
    return checker.healthCheck(inputs, subset, clocks);
  }
};
struct Assembler : np::SnapshotAssembler<Buffer, Access> {
  Snapshot assemble(const Buffer& buffer, const np::Trigger<Input>& trigger) const {
    auto reference = trigger;
    // Periodic에서는 IMU 최신 샘플을 기준으로 조립한다. LiDAR가 기준 시각을 바꾸지 않는다.
    if (trigger.mode == np::ExecutionMode::Periodic) {
      const auto& queue = std::get<1>(buffer.queues());
      if (!queue.empty()) {
        const auto latest = std::max_element(queue.begin(), queue.end(), [](const auto& a, const auto& b) {
          return a.stamp_ns < b.stamp_ns || (a.stamp_ns == b.stamp_ns && a.id < b.id);
        });
        reference = {np::ExecutionMode::OnInput, Input::Imu, latest->id, latest->stamp_ns};
      }
    }
    auto snapshot = np::SnapshotAssembler<Buffer, Access>::assemble(buffer, reference);
    // 위치 상태의 추정 시각은 IMU 시각이다. 점군 수신 시각으로 대체하지 않는다.
    if (snapshot.data.get<1>().present()) snapshot.context.stamp_ns = snapshot.data.get<1>().sample.stamp_ns;
    snapshot.context.frame = "map";
    return snapshot;
  }
};
using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
struct Selector {
  np::CoreSelector<Input, CoreId> mapping;
  np::Selection<CoreId> select(std::uint8_t section, const np::Health<Input, 3>& health) const {
    return mapping.select(section, health);
  }
};
struct HeaderInjector {
  void injectHeader(nav_msgs::Odometry& message, const np::Context<4>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "map";
  }
};
using IO = np::RosIO<nav_msgs::Odometry>;
using Stage = np::Stage<CoreSet, Buffer::ContextType>;
using Node = np::PipelineNode<Buffer, Assembler, CoreSet, HeaderInjector, IO>;
}
