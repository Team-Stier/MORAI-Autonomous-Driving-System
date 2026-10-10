#pragma once
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <cmath>

namespace tf_publisher {
class EgoTf {
 public:
  tf2_msgs::TFMessage process(const nav_msgs::Odometry& ego) {
    tf2_msgs::TFMessage message;
    message.transforms.resize(2);
    auto& transform = message.transforms.front();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = ego.pose.pose.position.x;
    transform.transform.translation.y = ego.pose.pose.position.y;
    transform.transform.translation.z = ego.pose.pose.position.z;
    transform.transform.rotation = ego.pose.pose.orientation;
    auto& level = message.transforms[1];
    level.child_frame_id = "base_link_level";
    level.transform.translation = transform.transform.translation;
    const auto& q = ego.pose.pose.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z));
    level.transform.rotation.z = std::sin(yaw / 2);
    level.transform.rotation.w = std::cos(yaw / 2);
    return message;
  }
};
}  // namespace tf_publisher
