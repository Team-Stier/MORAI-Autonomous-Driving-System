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
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <limits>
#include <cstring>

namespace object_detector {
struct ObjectClusteringParameters {
  bool emit_acquisition_time = false;
  double voxel_leaf_size = 0.075;
  double dbscan_eps = 0.6;
  int dbscan_min_points = 3;
  int min_cluster_size = 6;
  int max_cluster_size = 5000;
  double minimum_cluster_height_m = 0.1;
  double sparse_cluster_clearance_m = 0.3;
  double ground_distance_m = 0.08;
  double ground_seed_max_z_m = 0.0;
  double ground_cell_m = 0.5;
  double ground_region_m = 5.0;
  double ground_support_radius_m = 12.0;
  double ground_max_tilt_rad = 0.35;
  int ground_min_support = 12;
  int ground_iterations = 128;
};

// Team DBSCAN kernel; preprocessing, XY integration and tuning: workspace maintainer.
class ObjectClustering {
 public:
  explicit ObjectClustering(ObjectClusteringParameters parameters) : parameters_(parameters) {}

  std::tuple<Clusters, interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const sensor_msgs::PointCloud2& cloud, const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& tf) {
    Cloud::Ptr xyz(new Cloud);
    pcl::fromROSMsg(cloud, *xyz);
    std::vector<float> point_times;
    Tree acquisition_tree;
    if(parameters_.emit_acquisition_time) {
      const auto offset=std::find_if(cloud.fields.begin(),cloud.fields.end(),
          [](const sensor_msgs::PointField& field){return field.name=="time";})->offset;
      point_times.reserve(xyz->size());
      for(std::uint32_t row=0;row<cloud.height;++row)for(std::uint32_t col=0;col<cloud.width;++col) {
        float time;std::memcpy(&time,cloud.data.data()+row*cloud.row_step+col*cloud.point_step+offset,4);
        point_times.push_back(time);
      }
      if(!xyz->empty())acquisition_tree.setInputCloud(xyz);
    }
    const auto filtered = removeGround(teamVoxelDownsample(xyz));
    Clusters output;
    if (filtered->empty()) return std::make_tuple(std::move(output), competition, tf);

    for (const auto& cluster : findClusters(filtered)) {
      const auto size = static_cast<int>(cluster.size());
      if (size >= parameters_.min_cluster_size && size <= parameters_.max_cluster_size) {
        Cloud cluster_cloud;
        cluster_cloud.reserve(cluster.size());
        for (const auto index : cluster) cluster_cloud.push_back((*filtered)[index]);
        const auto extrema = std::minmax_element(cluster_cloud.begin(), cluster_cloud.end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.z < b.z; });
        bool elevated=false;
        for(const auto& point:cluster_cloud) {
          const Cell cell{static_cast<int>(std::floor(point.x/parameters_.ground_region_m)),
                          static_cast<int>(std::floor(point.y/parameters_.ground_region_m))};
          const auto& plane=ground_planes_.find(cell)->second;
          if(plane.support==0 || plane.normal.dot(point.getVector3fMap().cast<double>())+plane.offset>=
              parameters_.sparse_cluster_clearance_m) {elevated=true;break;}
        }
        // Keep sparse high returns with their observed extent. Only reject thin
        // fragments whose support places them close to the estimated road surface.
        if(elevated || extrema.second->z-extrema.first->z>=parameters_.minimum_cluster_height_m) {
          if(parameters_.emit_acquisition_time) {
            double sum=0;std::vector<int> nearest(1);std::vector<float> distances(1);
            for(const auto& point:cluster_cloud) {
              acquisition_tree.nearestKSearch(point,1,nearest,distances);
              sum+=point_times[nearest.front()];
            }
            // PCLHeader uses microseconds; this is the cluster's actual mean
            // acquisition time, not a replacement ROS/Pipeline output stamp.
            const double time=cloud.header.stamp.toSec()+sum/cluster_cloud.size();
            cluster_cloud.header.stamp=static_cast<std::uint64_t>(std::llround(time*1e6));
            cluster_cloud.header.frame_id=cloud.header.frame_id;
          }
          output.push_back(std::move(cluster_cloud));
        }
      }
    }
    return std::make_tuple(std::move(output), competition, tf);
  }

 private:
  using Tree = pcl::KdTreeFLANN<pcl::PointXYZ>;
  enum class PointState { Unvisited, Noise, Assigned };

  Cloud::Ptr teamVoxelDownsample(const Cloud::Ptr& cloud) const {
    Cloud::Ptr filtered(new Cloud);
    if (cloud->empty()) return filtered;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    const auto leaf = static_cast<float>(parameters_.voxel_leaf_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    return filtered;
  }

  std::vector<int> teamRadiusNeighbors(Tree& tree, int index) const {
    std::vector<int> neighbors;
    std::vector<float> squared_distances;
    // PCL searches the indexed point at distance zero too: MinPts includes self.
    tree.radiusSearch(index, parameters_.dbscan_eps, neighbors, squared_distances);
    return neighbors;
  }

  std::vector<int> expandTeamDbscanCluster(Tree& tree, int seed, std::vector<int> neighbors,
                                 std::vector<PointState>& states) const {
    std::vector<int> cluster{seed};
    states[seed] = PointState::Assigned;
    for (std::size_t cursor = 0; cursor < cluster.size(); ++cursor) {
      if (cursor != 0) neighbors = teamRadiusNeighbors(tree, cluster[cursor]);
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
    Cloud::Ptr horizontal(new Cloud(*cloud));
    for (auto& point : *horizontal) point.z = 0;
    Tree tree;
    tree.setInputCloud(horizontal);
    std::vector<PointState> states(cloud->size(), PointState::Unvisited);
    std::vector<std::vector<int>> clusters;
    for (std::size_t index = 0; index < cloud->size(); ++index) {
      if (states[index] != PointState::Unvisited) continue;
      auto neighbors = teamRadiusNeighbors(tree, static_cast<int>(index));
      if (static_cast<int>(neighbors.size()) < parameters_.dbscan_min_points) {
        states[index] = PointState::Noise;
        continue;
      }
      clusters.push_back(expandTeamDbscanCluster(tree, static_cast<int>(index), neighbors, states));
    }
    return clusters;
  }

  struct Plane {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double offset = 0;
    std::size_t support = 0;
  };

  Plane fitGround(const Cloud& seeds) const {
    Plane best;
    if (seeds.size() < static_cast<std::size_t>(parameters_.ground_min_support)) return best;
    std::mt19937 generator(17);
    std::uniform_int_distribution<std::size_t> index(0, seeds.size() - 1);
    double best_error = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < parameters_.ground_iterations; ++iteration) {
      const Eigen::Vector3d a = seeds[index(generator)].getVector3fMap().cast<double>();
      const Eigen::Vector3d b = seeds[index(generator)].getVector3fMap().cast<double>();
      const Eigen::Vector3d c = seeds[index(generator)].getVector3fMap().cast<double>();
      Eigen::Vector3d normal = (b - a).cross(c - a);
      if (normal.norm() < 1e-8) continue;
      normal.normalize();
      if (normal.z() < 0) normal = -normal;
      if (normal.z() < std::cos(parameters_.ground_max_tilt_rad)) continue;
      const double offset = -normal.dot(a);
      std::size_t count = 0;
      double error = 0;
      for (const auto& point : seeds) {
        const double distance = std::abs(normal.dot(point.getVector3fMap().cast<double>()) + offset);
        if (distance <= parameters_.ground_distance_m) { ++count; error += distance; }
      }
      if (count > best.support || (count == best.support && error < best_error)) {
        best = {normal, offset, count};
        best_error = error;
      }
    }
    if (best.support < static_cast<std::size_t>(parameters_.ground_min_support)) return Plane{};
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    std::vector<Eigen::Vector3d> inliers;
    for (const auto& point : seeds) {
      const Eigen::Vector3d value = point.getVector3fMap().cast<double>();
      if (std::abs(best.normal.dot(value) + best.offset) <= parameters_.ground_distance_m) {
        mean += value;
        inliers.push_back(value);
      }
    }
    mean /= inliers.size();
    for (const auto& value : inliers) covariance += (value - mean) * (value - mean).transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    // A plane needs support in two independent directions.
    if (eigen.eigenvalues()[1] < 0.01) return Plane{};
    {
      Eigen::Vector3d normal = eigen.eigenvectors().col(0);
      if (normal.z() < 0) normal = -normal;
      if (normal.z() >= std::cos(parameters_.ground_max_tilt_rad)) {
        best.normal = normal;
        best.offset = -normal.dot(mean);
      }
    }
    return best;
  }

  Cloud::Ptr removeGround(const Cloud::Ptr& points) {
    ground_planes_.clear();
    std::map<Cell, pcl::PointXYZ> lowest;
    for (const auto& point : *points) {
      if (point.z > parameters_.ground_seed_max_z_m) continue;
      const Cell cell{static_cast<int>(std::floor(point.x / parameters_.ground_cell_m)),
                      static_cast<int>(std::floor(point.y / parameters_.ground_cell_m))};
      const auto found = lowest.find(cell);
      if (found == lowest.end() || point.z < found->second.z) lowest[cell] = point;
    }
    Cloud seeds;
    for (const auto& entry : lowest) seeds.push_back(entry.second);
    const auto broad_plane = fitGround(seeds);
    std::map<Cell, std::vector<std::size_t>> regions;
    for (std::size_t i = 0; i < points->size(); ++i) {
      const auto& point = (*points)[i];
      regions[{static_cast<int>(std::floor(point.x / parameters_.ground_region_m)),
               static_cast<int>(std::floor(point.y / parameters_.ground_region_m))}].push_back(i);
    }
    Cloud::Ptr result(new Cloud);
    for (const auto& region : regions) {
      const double x = (region.first.first + 0.5) * parameters_.ground_region_m;
      const double y = (region.first.second + 0.5) * parameters_.ground_region_m;
      Cloud support;
      for (const auto& seed : seeds)
        if (std::hypot(seed.x - x, seed.y - y) <= parameters_.ground_support_radius_m)
          support.push_back(seed);
      auto plane = fitGround(support);
      // Far VLP16 returns can lack local ground support. Use the same scan's
      // broader road surface model rather than extrapolating a narrow support strip.
      if (plane.support == 0) plane = broad_plane;
      ground_planes_.emplace(region.first,plane);
      for (const auto i : region.second) {
        const auto& point = (*points)[i];
        // Unsupported regions stay unclassified; do not invent a ground plane.
        if (plane.support == 0 ||
            plane.normal.dot(point.getVector3fMap().cast<double>()) + plane.offset >
                parameters_.ground_distance_m) result->push_back(point);
      }
    }
    return result;
  }

  using Cell = std::pair<int,int>;
  std::map<Cell,Plane> ground_planes_;
  ObjectClusteringParameters parameters_;
};
}  // namespace object_detector
