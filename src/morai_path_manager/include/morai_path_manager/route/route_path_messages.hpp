#ifndef MORAI_PATH_MANAGER_ROUTE_PATH_MESSAGES_HPP_
#define MORAI_PATH_MANAGER_ROUTE_PATH_MESSAGES_HPP_

#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <ros/time.h>

#include "morai_path_manager/route/route_path.hpp"

namespace morai_path_manager {

nav_msgs::Path makePathMessage(const std::vector<RoutePoint>& points,
                               const std::string& frame_id,
                               const ros::Time& stamp);

}  // namespace morai_path_manager

#endif  // MORAI_PATH_MANAGER_ROUTE_PATH_MESSAGES_HPP_
