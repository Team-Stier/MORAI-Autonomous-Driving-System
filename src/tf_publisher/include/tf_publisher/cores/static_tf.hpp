#pragma once
#include <tf2_msgs/TFMessage.h>
#include <array>
#include <string>
#include <utility>
#include <vector>

namespace tf_publisher {
struct StaticTransform {
  std::string child;
  std::array<double, 3> translation;
  std::array<double, 4> rotation;
};
struct StaticTfParameters {
  std::vector<StaticTransform> transforms;
};
class StaticTf {
 public:
  explicit StaticTf(StaticTfParameters parameters) : parameters_(std::move(parameters)) {}
  tf2_msgs::TFMessage process() {
    tf2_msgs::TFMessage message;
    for (const auto& mount : parameters_.transforms) {
      geometry_msgs::TransformStamped transform;
      transform.header.frame_id = "base_link";
      transform.child_frame_id = mount.child;
      transform.transform.translation.x = mount.translation[0];
      transform.transform.translation.y = mount.translation[1];
      transform.transform.translation.z = mount.translation[2];
      transform.transform.rotation.x = mount.rotation[0];
      transform.transform.rotation.y = mount.rotation[1];
      transform.transform.rotation.z = mount.rotation[2];
      transform.transform.rotation.w = mount.rotation[3];
      message.transforms.push_back(std::move(transform));
    }
    return message;
  }
 private:
  StaticTfParameters parameters_;
};
}  // namespace tf_publisher
