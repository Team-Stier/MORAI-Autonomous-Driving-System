#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <utility>
#include <vector>

namespace object_detector {
struct ModelTemplate {
  std::string label;
  // Surface and collider share model-local ROS axes, metres, and baked asset scale.
  pcl::PointCloud<pcl::PointXYZ> points;
  // True only for a verified physics-collider enclosure; never a trigger proxy.
  bool collider_available = false;
  geometry_msgs::Pose collider_pose;
  geometry_msgs::Vector3 collider_size;
  // Set only for packaged assets with identical decoded surface and physics geometry.
  std::string semantic_label;
};

struct ModelBank {
  static const std::vector<ModelTemplate>& models() {
    static const std::vector<ModelTemplate> templates = [] {
#include <object_detector/models/npc_models.inc>
      std::vector<ModelTemplate> result;
      result.reserve(sizeof(models) / sizeof(models[0]));
      for (const auto& asset : models) {
        ModelTemplate model;
        model.label = asset.label;
        for (const auto& alias : aliases)
          if (model.label == alias.resource) model.semantic_label = alias.label;
        model.points.reserve(asset.count);
        for (std::size_t i = asset.begin; i < asset.begin + asset.count; ++i) {
          const auto& point = points[i];
          model.points.emplace_back(point[0], point[1], point[2]);
        }
        model.collider_available = asset.collider_available;
        model.collider_pose.position.x = asset.center[0];
        model.collider_pose.position.y = asset.center[1];
        model.collider_pose.position.z = asset.center[2];
        model.collider_pose.orientation.x = asset.orientation[0];
        model.collider_pose.orientation.y = asset.orientation[1];
        model.collider_pose.orientation.z = asset.orientation[2];
        model.collider_pose.orientation.w = asset.orientation[3];
        model.collider_size.x = asset.size[0];
        model.collider_size.y = asset.size[1];
        model.collider_size.z = asset.size[2];
        result.push_back(std::move(model));
      }
      return result;
    }();
    return templates;
  }
};
}  // namespace object_detector
