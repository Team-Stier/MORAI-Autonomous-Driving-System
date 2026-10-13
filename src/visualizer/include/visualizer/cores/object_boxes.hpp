#pragma once
#include <interfaces/Objects.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <array>
#include <string>

namespace visualizer {
struct ObjectBoxesParameters {
  int prediction_horizon = -1;
  bool flatten_z = false;
};

class ObjectBoxes {
 public:
  explicit ObjectBoxes(ObjectBoxesParameters parameters = {}) : parameters_(parameters) {}

  visualization_msgs::Marker process(const interfaces::Objects& input) {
    using Marker = visualization_msgs::Marker;
    const bool prediction = parameters_.prediction_horizon >= 0;
    const float progress = prediction ? parameters_.prediction_horizon / 9.0f : 0.0f;
    Marker marker;
    marker.ns = prediction ? "predicted_objects_" + std::to_string(parameters_.prediction_horizon) : "objects";
    marker.id = 0;
    marker.type = Marker::LINE_LIST;
    marker.action = input.objects.empty() ? Marker::DELETE : Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = prediction ? 0.08 : 0.12;
    marker.color.r = prediction ? progress : 0.95f;
    marker.color.g = prediction ? 1.0f - 0.65f * progress : 0.15f;
    marker.color.b = prediction ? 0.15f : 1.0f;
    marker.color.a = prediction ? 0.25f : 0.4f;
    marker.frame_locked = false;
    marker.lifetime = ros::Duration(0.5);
    marker.points.reserve(input.objects.size() * 24);
    for (const auto& object : input.objects) {
      const auto& pose = object.bbox_pose;
      const auto& size = object.bbox_size;
      const auto& q = pose.orientation;
      const tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
      std::array<geometry_msgs::Point, 8> corners;
      for (unsigned i = 0; i < corners.size(); ++i) {
        const tf2::Vector3 local((i & 1 ? 0.5 : -0.5) * size.x,
                                (i & 2 ? 0.5 : -0.5) * size.y,
                                (i & 4 ? 0.5 : -0.5) * size.z);
        const auto rotated = tf2::quatRotate(rotation, local);
        corners[i].x = pose.position.x + rotated.x();
        corners[i].y = pose.position.y + rotated.y();
        corners[i].z = (parameters_.flatten_z ? size.z / 2.0 : pose.position.z) + rotated.z();
      }
      for (unsigned i = 0; i < corners.size(); ++i) {
        for (unsigned axis = 0; axis < 3; ++axis) {
          const unsigned neighbor = i ^ (1u << axis);
          if (neighbor <= i) continue;
          marker.points.push_back(corners[i]);
          marker.points.push_back(corners[neighbor]);
        }
      }
    }
    return marker;
  }

 private:
  ObjectBoxesParameters parameters_;
};
}  // namespace visualizer
