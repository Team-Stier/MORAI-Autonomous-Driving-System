#pragma once
#include <morai_msgs/ObjectStatusList.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

namespace visualizer {
struct ObjectInfoParameters {
  double yaw_offset_rad = -0.023509;
  std::array<double, 3> offset_xyz{{0, 0, 0}};
  std::vector<std::array<double, 9>> profiles;
};

class ObjectInfo {
 public:
  explicit ObjectInfo(ObjectInfoParameters parameters) : parameters_(std::move(parameters)) {}
  visualization_msgs::Marker process(const morai_msgs::ObjectStatusList& message,
                                     const nav_msgs::Odometry& ego_gt) {
    visualization_msgs::Marker marker;
    marker.ns = "morai_objects";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.color.r = 0.15;
    marker.color.g = 0.45;
    marker.color.b = marker.color.a = 1;
    marker.lifetime = ros::Duration(0.5);
    for (const auto* list : {&message.npc_list, &message.pedestrian_list, &message.obstacle_list})
      for (const auto& object : *list) appendObject(marker, object, ego_gt.pose.pose);
    if (marker.points.empty()) marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  }

 private:
  static geometry_msgs::Point point(double x, double y, double z) {
    geometry_msgs::Point result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
  }

  void appendObject(visualization_msgs::Marker& marker, const morai_msgs::ObjectStatus& object,
                    const geometry_msgs::Pose& ego) const {
    const auto& q = ego.orientation;
    const auto inverse = tf2::Quaternion(q.x, q.y, q.z, q.w).inverse();
    const auto to_base = [&](const geometry_msgs::Point& world) {
      const auto local = tf2::quatRotate(inverse, tf2::Vector3(world.x - ego.position.x,
          world.y - ego.position.y, world.z - ego.position.z));
      return point(local.x(), local.y(), local.z());
    };
    const std::array<double, 9>* profile = nullptr;
    std::size_t matches = 0;
    for (const auto& candidate : parameters_.profiles) {
      // shortcut: size identifies the calibrated vehicles; use model IDs when equal-size assets are added.
      if ((object.type == -1 || object.type == 1)
          && std::abs(candidate[0] - object.size.x) <= 0.01
          && std::abs(candidate[1] - object.size.y) <= 0.01
          && std::abs(candidate[2] - object.size.z) <= 0.01) {
        profile = &candidate;
        ++matches;
      }
    }
    // shortcut: only Ego has full orientation; add actor RPY for sloped NPC overlays.
    const double yaw = object.heading * 0.017453292519943295 + parameters_.yaw_offset_rad;
    const auto rotation = object.type == -1 ? tf2::Quaternion(q.x, q.y, q.z, q.w)
        : tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw);
    const auto origin = point(object.position.x + parameters_.offset_xyz[0],
                              object.position.y + parameters_.offset_xyz[1],
                              object.position.z + parameters_.offset_xyz[2]);
    if (matches == 1) {
      std::array<geometry_msgs::Point, 8> corners;
      for (std::size_t i = 0; i < corners.size(); ++i) {
        const double x = (*profile)[3] + (i & 1 ? 0.5 : -0.5) * (*profile)[6];
        const double y = (*profile)[4] + (i & 2 ? 0.5 : -0.5) * (*profile)[7];
        const double z = (*profile)[5] + (i & 4 ? 0.5 : -0.5) * (*profile)[8];
        const auto corner = tf2::quatRotate(rotation, tf2::Vector3(x, y, z));
        corners[i] = to_base(point(origin.x + corner.x(), origin.y + corner.y(), origin.z + corner.z()));
      }
      for (std::size_t i = 0; i < corners.size(); ++i) {
        for (const std::size_t axis : {1u, 2u, 4u}) {
          if (i & axis) continue;
          marker.points.push_back(corners[i]);
          marker.points.push_back(corners[i | axis]);
        }
      }
    } else {
      marker.points.push_back(to_base(point(origin.x - 0.5, origin.y, origin.z)));
      marker.points.push_back(to_base(point(origin.x + 0.5, origin.y, origin.z)));
      marker.points.push_back(to_base(point(origin.x, origin.y - 0.5, origin.z)));
      marker.points.push_back(to_base(point(origin.x, origin.y + 0.5, origin.z)));
      marker.points.push_back(to_base(origin));
      const auto direction = tf2::quatRotate(rotation, tf2::Vector3(2.5, 0, 0));
      marker.points.push_back(to_base(point(origin.x + direction.x(), origin.y + direction.y(), origin.z + direction.z())));
    }
  }

  ObjectInfoParameters parameters_;
};
}  // namespace visualizer
