#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <object_detector/libraries/types.hpp>
#include <tf2_msgs/TFMessage.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <tuple>
#include <utility>

namespace object_detector {
class ClusterTf {
 public:
  std::tuple<Clusters, interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const Clusters& clusters, const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& tf) {
    const auto transform = std::find_if(tf.transforms.begin(), tf.transforms.end(),
      [](const geometry_msgs::TransformStamped& value) {
        return value.header.frame_id == "map" && value.child_frame_id == "base_link_level";
      });
    const auto& t = transform->transform;
    const Eigen::Quaterniond rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    const Eigen::Vector3d translation(t.translation.x, t.translation.y, t.translation.z);
    auto output = clusters;
    for (auto& cluster : output) {
      for (auto& point : cluster) {
        const Eigen::Vector3d position = rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
        point.x = position.x(); point.y = position.y(); point.z = position.z();
      }
    }
    return std::make_tuple(std::move(output), competition, tf);
  }
};
}  // namespace object_detector
