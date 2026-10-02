#pragma once
#include <control/tracking/commands.hpp>
#include <interfaces/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
#include <algorithm>
#include <cmath>
#include <utility>
namespace control {
class PathTracking {
 public:
  explicit PathTracking(tracking::Parameters parameters = {})
    : parameters_(std::move(parameters)), mpc_(parameters_.mpc), stanley_(parameters_.stanley),
      hybrid_(parameters_.hybrid), longitudinal_mpc_(parameters_.longitudinal_mpc), pid_(parameters_.pid) {}
  morai_msgs::CtrlCmd process(const interfaces::Path& path, const nav_msgs::Odometry& ego,
                             const interfaces::CompetitionVehicleStatus& status) {
    const auto stamp = path.header.stamp.toNSec();
    // ponytail: 예측 적분 간격은 설정 범위로 제한한다. 긴 공백의 상태 전파는 별도 모델이 필요하다.
    const double dt = previous_stamp_ == 0 ? parameters_.nominal_dt_sec :
      std::max(parameters_.minimum_dt_sec, std::min(parameters_.maximum_dt_sec,
        (static_cast<double>(stamp) - previous_stamp_) * 1e-9));
    previous_stamp_ = stamp;
    const double alpha = parameters_.speed_filter_time_constant_sec > 0 ?
      1 - std::exp(-dt / parameters_.speed_filter_time_constant_sec) : 1;
    filtered_speed_mps_ = has_speed_ ? filtered_speed_mps_ + alpha * (status.velocity_x_mps - filtered_speed_mps_) :
      status.velocity_x_mps;
    has_speed_ = true;
    std::vector<tracking::Point2d> points;
    points.reserve(path.points.size());
    for (const auto& point : path.points) points.push_back({point.pose.pose.position.x, point.pose.pose.position.y});
    const auto nearest = std::min_element(path.points.begin(), path.points.end(), [](const auto& a, const auto& b) {
      const auto& pa = a.pose.pose.position;
      const auto& pb = b.pose.pose.position;
      return pa.x * pa.x + pa.y * pa.y < pb.x * pb.x + pb.y * pb.y;
    });
    const double steering = tracking::calculateSteering(parameters_, mpc_, stanley_, hybrid_, points,
      filtered_speed_mps_, ego.twist.twist.linear.y, ego.twist.twist.angular.z, previous_steering_rad_, dt);
    const auto pedals = tracking::calculatePedals(parameters_, longitudinal_mpc_, pid_, nearest->velocity,
                                                 filtered_speed_mps_, dt);
    previous_steering_rad_ = steering;
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = steering / parameters_.pure_pursuit.maximum_steering_angle_rad;
    command.accel = pedals.accel;
    command.brake = pedals.brake;
    return command;
  }
 private:
  tracking::Parameters parameters_;
  tracking::MpcLateralController mpc_;
  tracking::StanleyController stanley_;
  tracking::HybridController hybrid_;
  tracking::LongitudinalMpc longitudinal_mpc_;
  tracking::LongitudinalPid pid_;
  std::uint64_t previous_stamp_ = 0;
  double previous_steering_rad_ = 0;
  double filtered_speed_mps_ = 0;
  bool has_speed_ = false;
};
}
