#include "morai_path_manager/route/route_path_messages.hpp"

#include <cmath>
#include <stdexcept>

#include <geometry_msgs/PoseStamped.h>

namespace morai_path_manager {

nav_msgs::Path makePathMessage(const std::vector<RoutePoint>& points,
                               const std::string& frame_id,
                               const ros::Time& stamp) {
  if (frame_id.empty()) {
    throw std::invalid_argument("path frame_id must not be empty");
  }

  nav_msgs::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame_id;
  path.poses.reserve(points.size());
  for (const RoutePoint& point : points) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(point.yaw * 0.5);
    pose.pose.orientation.w = std::cos(point.yaw * 0.5);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace morai_path_manager
