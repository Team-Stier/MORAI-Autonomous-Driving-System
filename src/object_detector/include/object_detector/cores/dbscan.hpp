#pragma once
#include <object_detector/types.hpp>
#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

namespace object_detector {
class Dbscan {
 public:
  explicit Dbscan(DbscanParameters parameters) : parameters_(parameters) {}

  interfaces::Objects process(const sensor_msgs::PointCloud2& cloud) {
    Cloud xyz;
    pcl::fromROSMsg(cloud, xyz);
    const auto filtered = downsample(cropRoi(xyz));
    interfaces::Objects output;
    if (filtered->empty()) return output;

    for (const auto& cluster : findClusters(filtered)) {
      const auto size = static_cast<int>(cluster.size());
      if (size >= parameters_.min_cluster_size && size <= parameters_.max_cluster_size)
        output.objects.push_back(makeObject(*filtered, cluster));
    }
    return output;
  }

 private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  using Tree = pcl::KdTreeFLANN<pcl::PointXYZ>;
  enum class PointState { Unvisited, Noise, Assigned };

  Cloud::Ptr cropRoi(const Cloud& cloud) const {
    Cloud::Ptr cropped(new Cloud);
    for (const auto& point : cloud) {
      if (point.x >= parameters_.roi_x_min && point.x <= parameters_.roi_x_max &&
          point.y >= parameters_.roi_y_min && point.y <= parameters_.roi_y_max &&
          point.z >= parameters_.roi_z_min && point.z <= parameters_.roi_z_max)
        cropped->push_back(point);
    }
    return cropped;
  }

  Cloud::Ptr downsample(const Cloud::Ptr& cloud) const {
    Cloud::Ptr filtered(new Cloud);
    if (cloud->empty()) return filtered;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    const auto leaf = static_cast<float>(parameters_.voxel_leaf_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    return filtered;
  }

  std::vector<int> neighborsWithin(Tree& tree, int index) const {
    std::vector<int> neighbors;
    std::vector<float> squared_distances;
    // PCL searches the indexed point at distance zero too: MinPts includes self.
    tree.radiusSearch(index, parameters_.dbscan_eps, neighbors, squared_distances);
    return neighbors;
  }

  std::vector<int> expandCluster(Tree& tree, int seed, std::vector<int> neighbors,
                                 std::vector<PointState>& states) const {
    std::vector<int> cluster{seed};
    states[seed] = PointState::Assigned;
    for (std::size_t cursor = 0; cursor < cluster.size(); ++cursor) {
      if (cursor != 0) neighbors = neighborsWithin(tree, cluster[cursor]);
      // Only core points expand; border points remain in the cluster.
      if (static_cast<int>(neighbors.size()) < parameters_.dbscan_min_points) continue;
      for (const auto index : neighbors) {
        if (states[index] == PointState::Assigned) continue;
        // Earlier noise can become a border point. Assign on enqueue to avoid duplicates.
        states[index] = PointState::Assigned;
        cluster.push_back(index);
      }
    }
    return cluster;
  }

  std::vector<std::vector<int>> findClusters(const Cloud::Ptr& cloud) const {
    Tree tree;
    tree.setInputCloud(cloud);
    std::vector<PointState> states(cloud->size(), PointState::Unvisited);
    std::vector<std::vector<int>> clusters;
    for (std::size_t index = 0; index < cloud->size(); ++index) {
      if (states[index] != PointState::Unvisited) continue;
      auto neighbors = neighborsWithin(tree, static_cast<int>(index));
      if (static_cast<int>(neighbors.size()) < parameters_.dbscan_min_points) {
        states[index] = PointState::Noise;
        continue;
      }
      clusters.push_back(expandCluster(tree, static_cast<int>(index), neighbors, states));
    }
    return clusters;
  }

  interfaces::Object makeObject(const Cloud& cloud, const std::vector<int>& cluster) const {
    auto minimum = cloud[cluster.front()];
    auto maximum = minimum;
    for (const auto index : cluster) {
      const auto& point = cloud[index];
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      minimum.z = std::min(minimum.z, point.z);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
      maximum.z = std::max(maximum.z, point.z);
    }
    interfaces::Object object;
    // AABB uses the extrema midpoint, not the point centroid; sizes are full lengths.
    object.position.x = (static_cast<double>(minimum.x) + maximum.x) / 2.0;
    object.position.y = (static_cast<double>(minimum.y) + maximum.y) / 2.0;
    object.position.z = (static_cast<double>(minimum.z) + maximum.z) / 2.0;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.x = 0.0;
    object.bbox_pose.orientation.y = 0.0;
    object.bbox_pose.orientation.z = 0.0;
    object.bbox_pose.orientation.w = 1.0;
    object.bbox_size.x = static_cast<double>(maximum.x) - minimum.x;
    object.bbox_size.y = static_cast<double>(maximum.y) - minimum.y;
    object.bbox_size.z = static_cast<double>(maximum.z) - minimum.z;
    object.label = "unknown";
    object.velocity_available = false;
    object.acceleration_available = false;
    return object;
  }

  DbscanParameters parameters_;
};
}  // namespace object_detector
