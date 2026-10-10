#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <tf2_msgs/TFMessage.h>
#include <object_detector/libraries/types.hpp>
#include <tuple>
#include <sensor_msgs/PointCloud2.h>
#include <utility>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

namespace object_detector {
struct DbscanParameters {
  // Synthetic starting values; competition tuning is supplied separately.
  double voxel_leaf_size = 0.04;
  double dbscan_eps = 0.18;
  int dbscan_min_points = 4;
  int min_cluster_size = 5;
  int max_cluster_size = 1000;
};

class Dbscan {
 public:
  explicit Dbscan(DbscanParameters parameters) : parameters_(parameters) {}

  std::tuple<Clusters, interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const sensor_msgs::PointCloud2& cloud, const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& tf) {
    Cloud::Ptr xyz(new Cloud);
    pcl::fromROSMsg(cloud, *xyz);
    const auto filtered = downsample(xyz);
    Clusters output;
    if (filtered->empty()) return std::make_tuple(std::move(output), competition, tf);

    for (const auto& cluster : findClusters(filtered)) {
      const auto size = static_cast<int>(cluster.size());
      if (size >= parameters_.min_cluster_size && size <= parameters_.max_cluster_size) {
        Cloud cluster_cloud;
        cluster_cloud.reserve(cluster.size());
        for (const auto index : cluster) cluster_cloud.push_back((*filtered)[index]);
        output.push_back(std::move(cluster_cloud));
      }
    }
    return std::make_tuple(std::move(output), competition, tf);
  }

 private:
  using Tree = pcl::KdTreeFLANN<pcl::PointXYZ>;
  enum class PointState { Unvisited, Noise, Assigned };

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

  DbscanParameters parameters_;
};
}  // namespace object_detector
