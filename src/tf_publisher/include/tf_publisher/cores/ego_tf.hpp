#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

namespace tf_publisher {
class EgoTf {
 public:
  tf2_msgs::TFMessage process(const geometry_msgs::PoseStamped& ego) {
    tf2_msgs::TFMessage message;
    message.transforms.resize(1);
    auto& transform = message.transforms.front();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = ego.pose.position.x;
    transform.transform.translation.y = ego.pose.position.y;
    transform.transform.translation.z = ego.pose.position.z;
    transform.transform.rotation = ego.pose.orientation;
    return message;
  }
};
}  // namespace tf_publisher
