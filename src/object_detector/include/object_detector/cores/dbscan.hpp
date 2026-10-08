#pragma once
#include <object_detector/libraries/oriented_box.hpp>
#include <interfaces/Objects.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

namespace object_detector {
struct DbscanParameters {
  double roi_x_min = 0.0;
  double roi_x_max = 20.0;
  double roi_y_min = -5.0;
  double roi_y_max = 5.0;
  double roi_z_min = 0.2;
  double roi_z_max = 2.0;
  double voxel_leaf_size = 0.04;
  double dbscan_eps = 0.18;
  int dbscan_min_points = 4;
  int min_cluster_size = 5;
  int max_cluster_size = 1000;
};

// base_link 점군을 ROI·voxel로 줄인 뒤 DBSCAN 군집마다 OBB 객체를 만든다.
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
        output.objects.push_back(bbox::makeObb(*filtered, cluster));
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


  DbscanParameters parameters_;
};
}  // namespace object_detector
