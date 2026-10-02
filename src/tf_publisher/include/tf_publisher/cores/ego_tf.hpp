#pragma once
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

namespace tf_publisher {
class EgoTf {
 public:
  tf2_msgs::TFMessage process(const nav_msgs::Odometry& ego) {
    tf2_msgs::TFMessage message;
    message.transforms.resize(1);
    auto& transform = message.transforms.front();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = ego.pose.pose.position.x;
    transform.transform.translation.y = ego.pose.pose.position.y;
    transform.transform.translation.z = ego.pose.pose.position.z;
    transform.transform.rotation = ego.pose.pose.orientation;
    return message;
  }
};
}  // namespace tf_publisher
