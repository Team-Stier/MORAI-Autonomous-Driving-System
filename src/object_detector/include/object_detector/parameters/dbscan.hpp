#pragma once
#include <object_detector/cores/dbscan.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace object_detector {
inline DbscanParameters loadDbscanParameters(const node_pipeline::ParamReader& params) {
  DbscanParameters config;
  config.voxel_leaf_size = params.number("voxel_leaf_size");
  config.dbscan_eps = params.number("dbscan_eps");
  config.dbscan_min_points = static_cast<int>(params.integer("dbscan_min_points"));
  config.min_cluster_size = static_cast<int>(params.integer("min_cluster_size"));
  config.max_cluster_size = static_cast<int>(params.integer("max_cluster_size"));
  if (!std::isfinite(config.voxel_leaf_size) || config.voxel_leaf_size <= 0 ||
      !std::isfinite(config.dbscan_eps) || config.dbscan_eps <= 0 ||
      config.dbscan_min_points <= 0 || config.min_cluster_size <= 0 ||
      config.max_cluster_size < config.min_cluster_size)
    throw std::invalid_argument("invalid DBSCAN parameters");
  return config;
}
}  // namespace object_detector
