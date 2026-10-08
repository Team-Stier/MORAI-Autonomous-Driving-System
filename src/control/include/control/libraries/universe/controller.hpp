#pragma once
#include <control/libraries/universe/parameters.hpp>
#include <memory>
namespace control { namespace universe {
class Controller {
 public:
  explicit Controller(const Parameters&);
  ~Controller();
  Controller(Controller&&) noexcept;
  Controller& operator=(Controller&&) noexcept;
  std::pair<autoware_control_msgs::msg::Lateral, autoware_control_msgs::msg::Longitudinal> calculate(
    const autoware_planning_msgs::msg::Trajectory&, const nav_msgs::Odometry&,
    double measured_steering_rad, double measured_acceleration_mps2, const ros::Time&);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}}
