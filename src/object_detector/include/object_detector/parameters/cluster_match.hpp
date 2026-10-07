#pragma once
#include <object_detector/cores/cluster_match.hpp>
#include <node_pipeline/sdk.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace object_detector { namespace parameters {
struct ClusterMatch {
  ClusterMatchParameters operator()(const node_pipeline::Parameters& parameters) const {
    ClusterMatchParameters config;
    config.voxel_leaf_m = parameters.get("voxel_leaf_m", config.voxel_leaf_m);
    config.cluster_tolerance_m = parameters.get("cluster_tolerance_m", config.cluster_tolerance_m);
    config.min_cluster_points = parameters.get("min_cluster_points", config.min_cluster_points);
    config.max_cluster_points = parameters.get("max_cluster_points", config.max_cluster_points);
    config.ground_distance_m = parameters.get("ground_distance_m", config.ground_distance_m);
    config.ground_max_tilt_rad = parameters.get("ground_max_tilt_rad", config.ground_max_tilt_rad);
    config.ground_seed_max_z_m = parameters.get("ground_seed_max_z_m", config.ground_seed_max_z_m);
    config.ground_cell_m = parameters.get("ground_cell_m", config.ground_cell_m);
    config.ground_region_m = parameters.get("ground_region_m", config.ground_region_m);
    config.ground_min_inliers = parameters.get("ground_min_inliers", config.ground_min_inliers);
    config.normal_radius_m = parameters.get("normal_radius_m", config.normal_radius_m);
    config.feature_radius_m = parameters.get("feature_radius_m", config.feature_radius_m);
    config.registration_iterations = parameters.get("registration_iterations", config.registration_iterations);
    config.icp_iterations = parameters.get("icp_iterations", config.icp_iterations);
    config.correspondence_distance_m = parameters.get("correspondence_distance_m", config.correspondence_distance_m);
    config.min_inlier_fraction = parameters.get("min_inlier_fraction", config.min_inlier_fraction);
    config.max_rmse_m = parameters.get("max_rmse_m", config.max_rmse_m);
    config.ambiguity_score_margin = parameters.get("ambiguity_score_margin", config.ambiguity_score_margin);
    config.min_pose_shape_ratio = parameters.get("min_pose_shape_ratio", config.min_pose_shape_ratio);
    config.model_max_tilt_rad = parameters.get("model_max_tilt_rad", config.model_max_tilt_rad);
    config.exclude_ego = parameters.get("exclude_ego", config.exclude_ego);
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
};
}}  // namespace object_detector::parameters
