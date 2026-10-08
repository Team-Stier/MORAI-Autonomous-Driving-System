#pragma once
#include <object_detector/cores/dbscan.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace object_detector {
inline DbscanParameters loadDbscanParameters(const node_pipeline::ParamReader& params) {
  DbscanParameters config;
  config.roi_x_min = params.number("roi_x_min");
  config.roi_x_max = params.number("roi_x_max");
  config.roi_y_min = params.number("roi_y_min");
  config.roi_y_max = params.number("roi_y_max");
  config.roi_z_min = params.number("roi_z_min");
  config.roi_z_max = params.number("roi_z_max");
  config.voxel_leaf_size = params.number("voxel_leaf_size");
  config.dbscan_eps = params.number("dbscan_eps");
  const auto dbscan_min_points = params.integer("dbscan_min_points");
  const auto min_cluster_size = params.integer("min_cluster_size");
  const auto max_cluster_size = params.integer("max_cluster_size");
  if (dbscan_min_points > INT32_MAX || min_cluster_size > INT32_MAX || max_cluster_size > INT32_MAX ||
      dbscan_min_points < INT32_MIN || min_cluster_size < INT32_MIN || max_cluster_size < INT32_MIN)
    throw std::invalid_argument("invalid DBSCAN parameters");
  config.dbscan_min_points = static_cast<int>(dbscan_min_points);
  config.min_cluster_size = static_cast<int>(min_cluster_size);
  config.max_cluster_size = static_cast<int>(max_cluster_size);
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
