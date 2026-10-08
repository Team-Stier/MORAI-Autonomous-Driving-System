#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector>

namespace control { namespace morai_mpc { namespace autoware {

// Internal reference geometry for the numerical helpers; no ROS message or I/O.
struct Waypoint {
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped twist;
};

struct Lane {
  std::vector<Waypoint> waypoints;
};

}}}  // namespace control::morai_mpc::autoware
