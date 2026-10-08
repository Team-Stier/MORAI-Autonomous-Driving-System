#pragma once
#include <object_detector/cores/cluster_match.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>

namespace object_detector {
inline ClusterMatchParameters loadClusterMatchParameters(const node_pipeline::ParamReader& parameters) {
  ClusterMatchParameters config;
  const auto integer = [&parameters](const char* key) {
    const auto value = parameters.integer(key);
    if (value < INT32_MIN || value > INT32_MAX) throw std::invalid_argument("invalid cluster_match settings");
    return static_cast<int>(value);
  };
  config.voxel_leaf_m = parameters.number("voxel_leaf_m");
  config.cluster_tolerance_m = parameters.number("cluster_tolerance_m");
  config.min_cluster_points = integer("min_cluster_points");
  config.max_cluster_points = integer("max_cluster_points");
  config.ground_distance_m = parameters.number("ground_distance_m");
  config.ground_max_tilt_rad = parameters.number("ground_max_tilt_rad");
  config.ground_seed_max_z_m = parameters.number("ground_seed_max_z_m");
  config.ground_cell_m = parameters.number("ground_cell_m");
  config.ground_region_m = parameters.number("ground_region_m");
  config.ground_min_inliers = integer("ground_min_inliers");
  config.normal_radius_m = parameters.number("normal_radius_m");
  config.feature_radius_m = parameters.number("feature_radius_m");
  config.registration_iterations = integer("registration_iterations");
  config.icp_iterations = integer("icp_iterations");
  config.correspondence_distance_m = parameters.number("correspondence_distance_m");
  config.min_inlier_fraction = parameters.number("min_inlier_fraction");
  config.max_rmse_m = parameters.number("max_rmse_m");
  config.ambiguity_score_margin = parameters.number("ambiguity_score_margin");
  config.min_pose_shape_ratio = parameters.number("min_pose_shape_ratio");
  config.model_max_tilt_rad = parameters.number("model_max_tilt_rad");
  config.exclude_ego = parameters.flag("exclude_ego");
  for (double value : {config.voxel_leaf_m, config.cluster_tolerance_m, config.ground_distance_m,
       config.ground_cell_m, config.ground_region_m, config.normal_radius_m, config.feature_radius_m,
       config.correspondence_distance_m, config.max_rmse_m})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("cluster_match lengths must be finite and positive");
  for (double value : {config.ground_max_tilt_rad, config.model_max_tilt_rad,
       config.ambiguity_score_margin, config.min_pose_shape_ratio})
    if (!std::isfinite(value) || value < 0) throw std::invalid_argument("invalid cluster_match coefficient");
  if (!std::isfinite(config.ground_seed_max_z_m) || !std::isfinite(config.min_inlier_fraction) ||
      config.min_inlier_fraction <= 0 || config.min_inlier_fraction > 1 ||
      config.ground_max_tilt_rad > 1.5707963267948966 || config.model_max_tilt_rad > 1.5707963267948966 ||
      config.min_pose_shape_ratio > 1 || config.min_cluster_points < 3 ||
      config.max_cluster_points < config.min_cluster_points || config.ground_min_inliers < 3 ||
      config.registration_iterations < 1 || config.icp_iterations < 1)
    throw std::invalid_argument("invalid cluster_match settings");
  return config;
}
}  // namespace object_detector
