#pragma once
#include <interfaces/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>

namespace velocity_planner {
class Tf {
 public:
  interfaces::Path process(const interfaces::Path& path, const tf2_msgs::TFMessage& transforms) {
    const auto match = std::find_if(transforms.transforms.begin(), transforms.transforms.end(),
      [](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
      });
    tf2::Transform map_from_base;
    tf2::fromMsg(match->transform, map_from_base);
    const auto base_from_map = map_from_base.inverse();
    interfaces::Path message = path;
    for (auto& point : message.points) {
      tf2::Transform pose;
      tf2::fromMsg(point.pose.pose, pose);
      tf2::toMsg(base_from_map * pose, point.pose.pose);
    }
    for (auto& point : message.lateral_reference) {
      tf2::Transform pose;
      tf2::fromMsg(point, pose);
      tf2::toMsg(base_from_map * pose, point);
    }
    return message;
  }
};
}  // namespace velocity_planner
