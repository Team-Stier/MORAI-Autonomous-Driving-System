#pragma once
#include <interfaces/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>

namespace velocity_planner {
class PlainVelocity100 {
 public:
  interfaces::Path process(const interfaces::Path& path, const tf2_msgs::TFMessage& tf) {
    const auto match = std::find_if(tf.transforms.begin(), tf.transforms.end(),
      [](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
      });
    tf2::Transform map_from_base;
    tf2::fromMsg(tf.transforms.at(match - tf.transforms.begin()).transform, map_from_base);
    const auto base_from_map = map_from_base.inverse();
    interfaces::Path message;
    message.points = path.points;
    for (auto& point : message.points) {
      tf2::Transform pose;
      tf2::fromMsg(point.pose.pose, pose);
      tf2::toMsg(base_from_map * pose, point.pose.pose);
      point.velocity = 100.0 / 3.6;
      point.acceleration = 0.0;
    }
    return message;
  }
};
}  // namespace velocity_planner
