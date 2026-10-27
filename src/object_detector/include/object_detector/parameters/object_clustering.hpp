#pragma once
#include <object_detector/cores/object_clustering.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace object_detector {
inline ObjectClusteringParameters loadObjectClusteringParameters(const node_pipeline::ParamReader& params) {
  ObjectClusteringParameters config;
  if(params.has("emit_acquisition_time"))config.emit_acquisition_time=params.flag("emit_acquisition_time");
  config.voxel_leaf_size = params.number("voxel_leaf_size");
  config.dbscan_eps = params.number("dbscan_eps");
  config.dbscan_min_points = static_cast<int>(params.integer("dbscan_min_points"));
  config.min_cluster_size = static_cast<int>(params.integer("min_cluster_size"));
  config.max_cluster_size = static_cast<int>(params.integer("max_cluster_size"));
  config.sparse_cluster_clearance_m = params.number("sparse_cluster_clearance_m");
  config.minimum_cluster_height_m = params.number("minimum_cluster_height_m");
  config.ground_distance_m = params.number("ground_distance_m");
  config.ground_seed_max_z_m = params.number("ground_seed_max_z_m");
  config.ground_cell_m = params.number("ground_cell_m");
  config.ground_region_m = params.number("ground_region_m");
  config.ground_support_radius_m = params.number("ground_support_radius_m");
  config.ground_max_tilt_rad = params.number("ground_max_tilt_rad");
  config.ground_min_support = static_cast<int>(params.integer("ground_min_support"));
  config.ground_iterations = static_cast<int>(params.integer("ground_iterations"));
  if (!std::isfinite(config.voxel_leaf_size) || config.voxel_leaf_size <= 0 ||
      !std::isfinite(config.dbscan_eps) || config.dbscan_eps <= 0 ||
      config.dbscan_min_points <= 0 || config.min_cluster_size <= 0 ||
      config.max_cluster_size < config.min_cluster_size)
    throw std::invalid_argument("invalid DBSCAN parameters");
  if (!std::isfinite(config.sparse_cluster_clearance_m) || config.sparse_cluster_clearance_m<=0)
    throw std::invalid_argument("invalid sparse cluster clearance");
  if (!std::isfinite(config.minimum_cluster_height_m) || config.minimum_cluster_height_m < 0)
    throw std::invalid_argument("invalid minimum observed cluster height");
  for (double value : {config.ground_distance_m, config.ground_cell_m, config.ground_region_m,
                       config.ground_support_radius_m})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid ground length");
  if (!std::isfinite(config.ground_seed_max_z_m) || !std::isfinite(config.ground_max_tilt_rad) ||
      config.ground_max_tilt_rad <= 0 || config.ground_max_tilt_rad >= 1.5707963267948966 ||
      config.ground_min_support < 3 || config.ground_iterations < 1)
    throw std::invalid_argument("invalid ground fit parameters");
  return config;
}
}  // namespace object_detector
