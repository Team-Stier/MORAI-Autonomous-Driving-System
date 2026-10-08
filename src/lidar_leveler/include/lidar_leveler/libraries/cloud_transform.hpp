#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>

namespace lidar_leveler {
// 점군 Core가 공유하는 변환 도우미다. 입력은 Validator leveling_inputs.hpp가 선언한 전제를 만족한다.

// /tf_static에서 base_link → 점군 frame 장착 변환을 꺼낸다.
inline tf2::Transform mountingTransform(const tf2_msgs::TFMessage& tf_static, const std::string& frame) {
  const auto mount = std::find_if(tf_static.transforms.begin(), tf_static.transforms.end(),
    [&](const geometry_msgs::TransformStamped& transform) {
      return transform.header.frame_id == "base_link" && transform.child_frame_id == frame;
    });
  const auto& rotation = mount->transform.rotation;
  const auto& translation = mount->transform.translation;
  return tf2::Transform(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                        tf2::Vector3(translation.x, translation.y, translation.z));
}

inline std::array<std::uint32_t, 3> xyzOffsets(const sensor_msgs::PointCloud2& cloud) {
  std::array<std::uint32_t, 3> offsets{};
  const char* names[] = {"x", "y", "z"};
  for (std::size_t axis = 0; axis < offsets.size(); ++axis)
    offsets[axis] = std::find_if(cloud.fields.begin(), cloud.fields.end(),
      [&](const sensor_msgs::PointField& field) { return field.name == names[axis]; })->offset;
  return offsets;
}

// 점마다 transform을 적용한다. keep_z가 있으면 변환 후 z가 [z_min, z_max]인 점 레코드만 앞으로 모아
// 1행 점군으로 만든다. 없으면 배치·padding·다른 필드를 그대로 둔다.
inline sensor_msgs::PointCloud2 transformCloud(const sensor_msgs::PointCloud2& cloud, const tf2::Transform& transform,
                                               bool keep_z = false, double z_min = 0, double z_max = 0) {
  auto output = cloud;
  const auto offsets = xyzOffsets(cloud);
  std::uint32_t retained = 0;
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      auto* record = output.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
                     static_cast<std::size_t>(col) * cloud.point_step;
      float xyz[3];
      for (std::size_t axis = 0; axis < 3; ++axis) std::memcpy(&xyz[axis], record + offsets[axis], sizeof(float));
      const auto point = transform * tf2::Vector3(xyz[0], xyz[1], xyz[2]);
      if (keep_z && (point.z() < z_min || point.z() > z_max)) continue;
      xyz[0] = static_cast<float>(point.x());
      xyz[1] = static_cast<float>(point.y());
      xyz[2] = static_cast<float>(point.z());
      for (std::size_t axis = 0; axis < 3; ++axis) std::memcpy(record + offsets[axis], &xyz[axis], sizeof(float));
      if (keep_z) {
        std::memmove(output.data.data() + static_cast<std::size_t>(retained) * cloud.point_step, record,
                     cloud.point_step);
        ++retained;
      }
    }
  }
  if (keep_z) {
    output.height = 1;
    output.width = retained;
    output.row_step = retained * output.point_step;
    output.data.resize(output.row_step);
  }
  return output;
}
}  // namespace lidar_leveler
