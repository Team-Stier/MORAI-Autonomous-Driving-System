#pragma once
#include <tf2_msgs/TFMessage.h>
#include <array>
#include <string>
#include <utility>
#include <vector>

namespace tf_publisher {
struct SensorMount {
  std::string child_frame;
  std::array<double, 3> translation_m{};
  std::array<double, 4> rotation_xyzw{{0, 0, 0, 1}};
};
struct SensorMountsParameters {
  std::vector<SensorMount> mounts;
};

// base_link → 센서 frame 장착 변환 전체를 한 메시지로 만든다.
class SensorMounts {
 public:
  explicit SensorMounts(SensorMountsParameters parameters) : parameters_(std::move(parameters)) {}
  tf2_msgs::TFMessage process() {
    tf2_msgs::TFMessage message;
    for (const auto& mount : parameters_.mounts) {
      geometry_msgs::TransformStamped transform;
      transform.child_frame_id = mount.child_frame;
      transform.transform.translation.x = mount.translation_m[0];
      transform.transform.translation.y = mount.translation_m[1];
      transform.transform.translation.z = mount.translation_m[2];
      transform.transform.rotation.x = mount.rotation_xyzw[0];
      transform.transform.rotation.y = mount.rotation_xyzw[1];
      transform.transform.rotation.z = mount.rotation_xyzw[2];
      transform.transform.rotation.w = mount.rotation_xyzw[3];
      message.transforms.push_back(transform);
    }
    return message;
  }

 private:
  SensorMountsParameters parameters_;
};
}  // namespace tf_publisher
