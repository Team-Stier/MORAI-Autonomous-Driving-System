#pragma once
#include <object_detector/cores/dbscan.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace object_detector {
inline DbscanParameters loadDbscanParameters(const node_pipeline::ParamReader& params) {
  DbscanParameters parameters;
  parameters.roi_x_min = params.number("roi_x_min");
  parameters.roi_x_max = params.number("roi_x_max");
  parameters.roi_y_min = params.number("roi_y_min");
  parameters.roi_y_max = params.number("roi_y_max");
  parameters.roi_z_min = params.number("roi_z_min");
  parameters.roi_z_max = params.number("roi_z_max");
  parameters.voxel_leaf_size = params.number("voxel_leaf_size");
  parameters.dbscan_eps = params.number("dbscan_eps");
  const auto min_points = params.integer("dbscan_min_points");
  const auto min_cluster = params.integer("min_cluster_size");
  const auto max_cluster = params.integer("max_cluster_size");
  const auto finite_range = [](double lower, double upper) {
    return std::isfinite(lower) && std::isfinite(upper) && lower <= upper;
  };
  const auto positive_int = [](std::int64_t value) { return value > 0 && value <= INT32_MAX; };
  if (!finite_range(parameters.roi_x_min, parameters.roi_x_max) ||
      !finite_range(parameters.roi_y_min, parameters.roi_y_max) ||
      !finite_range(parameters.roi_z_min, parameters.roi_z_max) ||
      !std::isfinite(parameters.voxel_leaf_size) || parameters.voxel_leaf_size <= 0 ||
      !std::isfinite(parameters.dbscan_eps) || parameters.dbscan_eps <= 0 ||
      !positive_int(min_points) || !positive_int(min_cluster) || !positive_int(max_cluster) ||
      max_cluster < min_cluster)
    throw std::invalid_argument("invalid DBSCAN parameters");
  parameters.dbscan_min_points = static_cast<int>(min_points);
  parameters.min_cluster_size = static_cast<int>(min_cluster);
  parameters.max_cluster_size = static_cast<int>(max_cluster);
  return parameters;
}
}  // namespace object_detector
