#pragma once
#include <object_detector/types.hpp>
#include <ros/node_handle.h>
#include <cmath>
#include <stdexcept>

namespace object_detector {
inline DbscanParameters loadDbscanParameters(ros::NodeHandle& params) {
  DbscanParameters config;
  params.param("dbscan/roi_x_min", config.roi_x_min, config.roi_x_min);
  params.param("dbscan/roi_x_max", config.roi_x_max, config.roi_x_max);
  params.param("dbscan/roi_y_min", config.roi_y_min, config.roi_y_min);
  params.param("dbscan/roi_y_max", config.roi_y_max, config.roi_y_max);
  params.param("dbscan/roi_z_min", config.roi_z_min, config.roi_z_min);
  params.param("dbscan/roi_z_max", config.roi_z_max, config.roi_z_max);
  params.param("dbscan/voxel_leaf_size", config.voxel_leaf_size, config.voxel_leaf_size);
  params.param("dbscan/dbscan_eps", config.dbscan_eps, config.dbscan_eps);
  params.param("dbscan/dbscan_min_points", config.dbscan_min_points, config.dbscan_min_points);
  params.param("dbscan/min_cluster_size", config.min_cluster_size, config.min_cluster_size);
  params.param("dbscan/max_cluster_size", config.max_cluster_size, config.max_cluster_size);
  if (!std::isfinite(config.roi_x_min) || !std::isfinite(config.roi_x_max) ||
      !std::isfinite(config.roi_y_min) || !std::isfinite(config.roi_y_max) ||
      !std::isfinite(config.roi_z_min) || !std::isfinite(config.roi_z_max) ||
      config.roi_x_min > config.roi_x_max || config.roi_y_min > config.roi_y_max ||
      config.roi_z_min > config.roi_z_max ||
      !std::isfinite(config.voxel_leaf_size) || config.voxel_leaf_size <= 0 ||
      !std::isfinite(config.dbscan_eps) || config.dbscan_eps <= 0 ||
      config.dbscan_min_points <= 0 || config.min_cluster_size <= 0 ||
      config.max_cluster_size < config.min_cluster_size)
    throw std::invalid_argument("invalid DBSCAN parameters");
  return config;
}
}  // namespace object_detector
