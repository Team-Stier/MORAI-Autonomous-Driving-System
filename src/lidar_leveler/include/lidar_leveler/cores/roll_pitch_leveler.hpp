#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace lidar_leveler {
class RollPitchLeveler {
 public:
  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud,
                                  const interfaces::CompetitionVehicleStatus& competition,
                                  const tf2_msgs::TFMessage& tf_static) {
    const auto mount = std::find_if(tf_static.transforms.begin(), tf_static.transforms.end(),
      [&](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "base_link" && transform.child_frame_id == cloud.header.frame_id;
      });
    const auto& transform = tf_static.transforms.at(mount - tf_static.transforms.begin()).transform;
    const auto& rotation = transform.rotation;
    const auto& translation = transform.translation;
    const tf2::Transform mounting_tf(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
      tf2::Vector3(translation.x, translation.y, translation.z));

    // R_world_body = Rz(yaw) Ry(pitch) Rx(roll), R_world_level = Rz(yaw).
    // R_level_body = R_world_level^T R_world_body = Ry(pitch) Rx(roll).
    // Positive RPY maps body coordinates into the yaw-following level frame.
    tf2::Quaternion leveling_rotation;
    leveling_rotation.setRPY(competition.roll_rad, competition.pitch_rad, 0.0);
    leveling_rotation.normalize();

    auto output = cloud;
    const auto fieldOffset = [&](const char* name) {
      const auto field = std::find_if(output.fields.begin(), output.fields.end(),
        [&](const sensor_msgs::PointField& value) { return value.name == name; });
      return output.fields.at(field - output.fields.begin()).offset;
    };
    const std::array<std::uint32_t, 3> offsets = {{fieldOffset("x"), fieldOffset("y"), fieldOffset("z")}};
    // XYZ are native FLOAT32. row_step includes row padding; point_step includes point padding.
    for (std::size_t row = 0; row < output.height; ++row) {
      for (std::size_t column = 0; column < output.width; ++column) {
        auto* point_data = output.data.data() + row * output.row_step + column * output.point_step;
        std::array<float, 3> coordinates;
        for (std::size_t axis = 0; axis < coordinates.size(); ++axis)
          std::memcpy(&coordinates[axis], point_data + offsets[axis], sizeof(float));
        const auto point_base = mounting_tf * tf2::Vector3(coordinates[0], coordinates[1], coordinates[2]);
        const auto point_level = tf2::quatRotate(leveling_rotation, point_base);
        coordinates = {{static_cast<float>(point_level.x()), static_cast<float>(point_level.y()),
          static_cast<float>(point_level.z())}};
        for (std::size_t axis = 0; axis < coordinates.size(); ++axis)
          std::memcpy(point_data + offsets[axis], &coordinates[axis], sizeof(float));
      }
    }
    return output;
  }
};
}  // namespace lidar_leveler
