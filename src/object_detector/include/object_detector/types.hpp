#pragma once
#include <node_pipeline/components.hpp>
#include <interfaces/Objects.h>
#include <interfaces/SectionAnnotation.h>
#include <sensor_msgs/PointCloud2.h>
namespace object_detector {
using Data = node_pipeline::Data<sensor_msgs::PointCloud2>;
using Message = interfaces::Objects;
struct DbscanParameters {
  // Synthetic starting values; competition tuning is supplied separately.
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
}  // namespace object_detector
