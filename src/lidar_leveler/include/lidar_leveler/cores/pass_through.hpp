#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>

namespace lidar_leveler {
class PassThrough {
 public:
  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud,
                                  const interfaces::CompetitionVehicleStatus& competition,
                                  const tf2_msgs::TFMessage& tf_static) {
    const auto mount = std::find_if(tf_static.transforms.begin(), tf_static.transforms.end(),
      [&](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "base_link" && transform.child_frame_id == cloud.header.frame_id;
      });
    const auto& transform = mount->transform;
    const auto& rotation = transform.rotation;
    const auto& translation = transform.translation;
    const tf2::Transform mounting_tf(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
      tf2::Vector3(translation.x, translation.y, translation.z));
    tf2::Quaternion level_from_body;
    level_from_body.setRPY(competition.roll_rad, competition.pitch_rad, 0);
    const tf2::Transform level_from_sensor =
      tf2::Transform(level_from_body, tf2::Vector3(0, 0, 0)) * mounting_tf;
    auto output = cloud;
    sensor_msgs::PointCloud2Iterator<float> x(output, "x"), y(output, "y"), z(output, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      const auto point = level_from_sensor * tf2::Vector3(*x, *y, *z);
      *x = point.x();
      *y = point.y();
      *z = point.z();
    }
    return output;
  }
};
}  // namespace lidar_leveler
