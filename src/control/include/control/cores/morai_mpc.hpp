#pragma once
// Declared before control::morai_mpc so control::tracking resolves morai_mpc:: to ::morai_mpc.
#include <control/libraries/tracking/parameters.hpp>
#include <control/libraries/tracking/morai_mpc/autoware_lateral.hpp>
#include <control/libraries/tracking/morai_mpc/steering_prediction.hpp>
#include <interfaces/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/console.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <utility>

namespace control {
class MoraiMpc {
 public:
  explicit MoraiMpc(morai_mpc::Parameters parameters = {})
      : parameters_(std::move(parameters)),
        lateral_(parameters_.lateral, parameters_.steering_prediction),
        longitudinal_(parameters_.longitudinal),
        steering_(parameters_.steering_prediction.delay_sec,
                  parameters_.steering_prediction.time_constant_sec) {}

  morai_msgs::CtrlCmd process(const interfaces::Path& path, const nav_msgs::Odometry& ego,
                             const interfaces::CompetitionVehicleStatus& status) {
    const auto stamp = path.header.stamp.toNSec();
    const double dt = has_previous_stamp_ ?
        std::max(parameters_.minimum_control_dt_sec,
          std::min(parameters_.maximum_control_dt_sec,
            (static_cast<double>(stamp) - static_cast<double>(previous_stamp_)) * 1e-9)) :
        parameters_.lateral.ctrl_period;
    const double time_sec = path.header.stamp.toSec();
    previous_stamp_ = stamp;
    has_previous_stamp_ = true;

    const auto nearest = std::min_element(path.points.begin(), path.points.end(),
        [](const auto& a, const auto& b) {
          const auto& pa = a.pose.pose.position;
          const auto& pb = b.pose.pose.position;
          return pa.x * pa.x + pa.y * pa.y < pb.x * pb.x + pb.y * pb.y;
        });
    // The source Autoware adapter uses observed odometry speed throughout the
    // lateral reference. The injected future velocity/acceleration profile is
    // not a longitudinal horizon reference for this controller.
    const double speed_mps = ego.twist.twist.linear.x;
    control::morai_mpc::autoware::Lane waypoints;
    waypoints.waypoints.reserve(path.points.size());
    for (const auto& point : path.points) {
      control::morai_mpc::autoware::Waypoint waypoint;
      waypoint.pose = point.pose;
      waypoint.twist.twist.linear.x = std::max(0.5, std::abs(speed_mps));
      waypoints.waypoints.push_back(waypoint);
    }
    // On numerical failure the previous command is kept.
    double requested = previous_steering_rad_;
    try {
      requested = lateral_.calculate(waypoints, speed_mps,
          steering_.advance(time_sec), previous_steering_rad_);
    } catch (const std::exception& error) {
      ROS_WARN_STREAM("morai_mpc lateral MPC failed, keeping the previous steering: " << error.what());
    }
    const double limit = parameters_.lateral.steer_lim_deg * morai_mpc::kPi / 180.0;
    const double step = parameters_.lateral.morai_steer_rate_limit_deg *
                        morai_mpc::kPi / 180.0 * dt;
    double angle = std::max(-limit, std::min(limit, requested));
    angle = std::max(previous_steering_rad_ - step,
                     std::min(previous_steering_rad_ + step, angle));
    const auto pedals = longitudinal_.update(nearest->velocity, status.velocity_x_mps, dt);
    if (pedals.valid) {
      previous_accel_ = pedals.accel;
      previous_brake_ = pedals.brake;
    } else {
      ROS_WARN_STREAM("morai_mpc longitudinal MPC failed, keeping the previous pedals: " << pedals.error);
    }

    previous_steering_rad_ = angle;
    steering_.sent(time_sec, angle * parameters_.steering_prediction.gain);
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = angle / morai_mpc::kBridgeSteeringLimitRad;
    command.accel = previous_accel_;
    command.brake = previous_brake_;
    return command;
  }

 private:
  morai_mpc::Parameters parameters_;
  morai_mpc::AutowareLateral lateral_;
  morai_mpc::LongitudinalMpc longitudinal_;
  morai_mpc::SteeringPrediction steering_;
  std::uint64_t previous_stamp_ = 0;
  bool has_previous_stamp_ = false;
  double previous_steering_rad_ = 0.0;
  double previous_accel_ = 0.0;
  double previous_brake_ = 0.0;
};
}  // namespace control
