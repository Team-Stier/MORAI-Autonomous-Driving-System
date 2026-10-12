#pragma once
#include <interfaces/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
#include <algorithm>
#include <cmath>

namespace control {
struct PpParameters {
  double wheelbase_m = 3.0;
  double lookahead_min_m = 2.0;
  double lookahead_max_m = 15.0;
  double lookahead_time_sec = 0.6;
  double steering_rate_limit_radps = 1.0471975512;
  // shortcut: fixed control interval, use measured intervals if scheduling becomes variable.
  double control_period_sec = 0.05;
  double speed_kp = 1.2;
  double speed_ki = 0.25;
  double speed_kd = 0.05;
  double integral_limit_mps_sec = 4.0;
  double acceleration_gain_mps2 = 5.74;
  double braking_gain_mps2 = 18.76;
  double maximum_accel = 0.70;
  double maximum_brake = 0.60;
  double stop_speed_mps = 0.10;
  double hold_brake = 0.20;
};

class Pp {
 public:
  explicit Pp(PpParameters parameters) : parameters_(parameters) {}
  morai_msgs::CtrlCmd process(const interfaces::Path& path, const nav_msgs::Odometry&,
                             const interfaces::CompetitionVehicleStatus& competition) {
    const auto& points = path.points;
    std::size_t nearest = 0;
    double nearest_distance = std::hypot(points[0].pose.pose.position.x,
                                         points[0].pose.pose.position.y);
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto& position = points[i].pose.pose.position;
      const double distance = std::hypot(position.x, position.y);
      if (distance < nearest_distance) {
        nearest = i;
        nearest_distance = distance;
      }
    }

    const double speed = competition.velocity_x_mps;
    const double lookahead = clamp(std::max(0.0, speed) * parameters_.lookahead_time_sec,
                                    parameters_.lookahead_min_m, parameters_.lookahead_max_m);
    double target_x = points.back().pose.pose.position.x;
    double target_y = points.back().pose.pose.position.y;
    for (std::size_t i = std::max<std::size_t>(1, nearest); i < points.size(); ++i) {
      const auto& a = points[i - 1].pose.pose.position;
      const auto& b = points[i].pose.pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double squared_length = dx * dx + dy * dy;
      if (squared_length == 0.0) continue;
      const double linear = 2.0 * (a.x * dx + a.y * dy);
      const double constant = a.x * a.x + a.y * a.y - lookahead * lookahead;
      const double discriminant = linear * linear - 4.0 * squared_length * constant;
      if (discriminant < 0.0) continue;
      const double ratio = (-linear + std::sqrt(discriminant)) / (2.0 * squared_length);
      if (ratio < 0.0 || ratio > 1.0 || a.x + ratio * dx <= 0.0) continue;
      target_x = a.x + ratio * dx;
      target_y = a.y + ratio * dy;
      break;
    }

    constexpr double bridge_steering_limit_rad = 0.6981317007977318;
    const double desired_steering = target_x > 0.0
        ? std::atan2(2.0 * parameters_.wheelbase_m * target_y,
                     target_x * target_x + target_y * target_y) : 0.0;
    const double steering_step = parameters_.steering_rate_limit_radps * parameters_.control_period_sec;
    previous_steering_rad_ = clamp(clamp(desired_steering, -bridge_steering_limit_rad,
                                         bridge_steering_limit_rad),
                                   previous_steering_rad_ - steering_step,
                                   previous_steering_rad_ + steering_step);
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = previous_steering_rad_ / bridge_steering_limit_rad;

    const double target_speed = points[nearest].velocity;
    const double speed_derivative = has_previous_speed_
        ? (speed - previous_speed_mps_) / parameters_.control_period_sec : 0.0;
    previous_speed_mps_ = speed;
    has_previous_speed_ = true;
    if (target_speed == 0.0 && speed <= parameters_.stop_speed_mps) {
      speed_integral_ = 0.0;
      command.brake = parameters_.hold_brake;
      return command;
    }

    const double error = target_speed - speed;
    const double next_integral = clamp(speed_integral_ + error * parameters_.control_period_sec,
                                       -parameters_.integral_limit_mps_sec,
                                       parameters_.integral_limit_mps_sec);
    const double proportional_derivative = parameters_.speed_kp * error
        - parameters_.speed_kd * speed_derivative;
    const double requested_acceleration = proportional_derivative + parameters_.speed_ki * next_integral;
    const double upper = parameters_.maximum_accel * parameters_.acceleration_gain_mps2;
    const double lower = -parameters_.maximum_brake * parameters_.braking_gain_mps2;
    if ((requested_acceleration >= lower && requested_acceleration <= upper) ||
        (requested_acceleration > upper && error < 0.0) ||
        (requested_acceleration < lower && error > 0.0)) {
      speed_integral_ = next_integral;
    }
    const double acceleration = clamp(proportional_derivative + parameters_.speed_ki * speed_integral_,
                                       lower, upper);
    if (acceleration >= 0.0) command.accel = acceleration / parameters_.acceleration_gain_mps2;
    else command.brake = -acceleration / parameters_.braking_gain_mps2;
    return command;
  }

 private:
  static double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
  }

  PpParameters parameters_;
  double previous_steering_rad_ = 0.0;
  double previous_speed_mps_ = 0.0;
  double speed_integral_ = 0.0;
  bool has_previous_speed_ = false;
};
}  // namespace control
