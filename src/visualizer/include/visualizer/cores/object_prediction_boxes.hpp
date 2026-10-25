#pragma once
#include <interfaces/Objects.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <array>

namespace visualizer {
class ObjectPredictionBoxes {
 public:
  visualization_msgs::Marker process(const interfaces::Objects& input) {
    visualization_msgs::Marker marker;
    marker.ns = "objects";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = input.objects.empty() ? visualization_msgs::Marker::DELETE
                                          : visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.12;
    marker.color.a = 0.8;
    marker.lifetime = ros::Duration(0.5);
    marker.points.reserve(input.objects.size() * 264);
    marker.colors.reserve(input.objects.size() * 264);
    for (const auto& object : input.objects) {
      std_msgs::ColorRGBA color;
      color.r = 0.95f;
      color.g = 0.15f;
      color.b = color.a = 1;
      appendBox(marker, object.bbox_pose, object.bbox_size, color);
      if (object.prediction_available) {
        for (unsigned i = 0; i < object.predicted_bbox_poses.size(); ++i) {
          const float progress = i / 9.0f;
          color.r = progress;
          color.g = 1.0f - 0.65f * progress;
          color.b = 0.15f;
          appendBox(marker, object.predicted_bbox_poses[i], object.bbox_size, color);
        }
      }
    }
    return marker;
  }

 private:
  static void appendBox(visualization_msgs::Marker& marker, const geometry_msgs::Pose& pose,
                        const geometry_msgs::Vector3& size, const std_msgs::ColorRGBA& color) {
    const auto& q = pose.orientation;
    const tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    std::array<geometry_msgs::Point, 8> corners;
    for (unsigned i = 0; i < corners.size(); ++i) {
      const auto rotated = tf2::quatRotate(rotation, tf2::Vector3(
          (i & 1 ? 0.5 : -0.5) * size.x, (i & 2 ? 0.5 : -0.5) * size.y,
          (i & 4 ? 0.5 : -0.5) * size.z));
      corners[i].x = pose.position.x + rotated.x();
      corners[i].y = pose.position.y + rotated.y();
      corners[i].z = pose.position.z + rotated.z();
    }
    for (unsigned i = 0; i < corners.size(); ++i) {
      for (unsigned axis = 0; axis < 3; ++axis) {
        const unsigned neighbor = i ^ (1u << axis);
        if (neighbor <= i) continue;
        marker.points.push_back(corners[i]);
        marker.points.push_back(corners[neighbor]);
        marker.colors.push_back(color);
        marker.colors.push_back(color);
      }
    }
  }
};
}  // namespace visualizer
