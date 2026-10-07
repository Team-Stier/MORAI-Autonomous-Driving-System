#pragma once
#include <lidar_leveler/types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <cstring>

namespace lidar_leveler {
class PassThrough {
 public:
  explicit PassThrough(PassThroughParameters parameters = {}) : parameters_(parameters) {}

  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud,
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
    auto output = cloud;
    const auto field_offset = [&](const char* name) {
      return std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const sensor_msgs::PointField& field) { return field.name == name; })->offset;
    };
    const std::uint32_t offsets[] = {field_offset("x"), field_offset("y"), field_offset("z")};
    std::uint32_t retained = 0;
    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        auto* record = output.data.data() + static_cast<std::size_t>(row) * cloud.row_step + col * cloud.point_step;
        float coordinates[3];
        for (std::size_t axis = 0; axis < 3; ++axis)
          std::memcpy(&coordinates[axis], record + offsets[axis], sizeof(float));
        const auto point = mounting_tf * tf2::Vector3(coordinates[0], coordinates[1], coordinates[2]);
        if (point.z() >= parameters_.z_min_m && point.z() <= parameters_.z_max_m) {
          coordinates[0] = point.x();
          coordinates[1] = point.y();
          coordinates[2] = point.z();
          for (std::size_t axis = 0; axis < 3; ++axis)
            std::memcpy(record + offsets[axis], &coordinates[axis], sizeof(float));
          std::memmove(output.data.data() + static_cast<std::size_t>(retained) * cloud.point_step,
                       record, cloud.point_step);
          ++retained;
        }
      }
    }
    output.height = 1;
    output.width = retained;
    output.row_step = retained * output.point_step;
    output.data.resize(output.row_step);
    return output;
  }

 private:
  PassThroughParameters parameters_;
};
}  // namespace lidar_leveler
