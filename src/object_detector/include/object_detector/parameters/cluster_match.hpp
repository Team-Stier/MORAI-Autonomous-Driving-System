#pragma once
#include <object_detector/cores/cluster_match.hpp>
#include <object_detector/libraries/npc_models.hpp>
#include <node_pipeline/extension.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

namespace object_detector {
inline ClusterMatchParameters loadClusterMatchParameters(const node_pipeline::ParamReader& params) {
  ClusterMatchParameters config;
  config.voxel_leaf_m = params.number("voxel_leaf_m");
  config.cluster_tolerance_m = params.number("cluster_tolerance_m");
  config.ground_distance_m = params.number("ground_distance_m");
  config.ground_max_tilt_rad = params.number("ground_max_tilt_rad");
  config.ground_seed_max_z_m = params.number("ground_seed_max_z_m");
  config.ground_cell_m = params.number("ground_cell_m");
  config.ground_region_m = params.number("ground_region_m");
  config.normal_radius_m = params.number("normal_radius_m");
  config.feature_radius_m = params.number("feature_radius_m");
  config.correspondence_distance_m = params.number("correspondence_distance_m");
  config.min_inlier_fraction = params.number("min_inlier_fraction");
  config.max_rmse_m = params.number("max_rmse_m");
  config.ambiguity_score_margin = params.number("ambiguity_score_margin");
  config.min_pose_shape_ratio = params.number("min_pose_shape_ratio");
  config.model_max_tilt_rad = params.number("model_max_tilt_rad");
  config.exclude_ego = params.flag("exclude_ego");
  const auto integer = [&](const char* key, std::int64_t minimum) {
    const auto value = params.integer(key);
    if (value < minimum || value > INT32_MAX) throw std::invalid_argument(std::string("invalid cluster_match ") + key);
    return static_cast<int>(value);
  };
  config.min_cluster_points = integer("min_cluster_points", 3);
  config.max_cluster_points = integer("max_cluster_points", config.min_cluster_points);
  config.ground_min_inliers = integer("ground_min_inliers", 3);
  config.registration_iterations = integer("registration_iterations", 1);
  config.icp_iterations = integer("icp_iterations", 1);
  const auto box = [&](const char* key, std::array<double, 3>& value) {
    const auto numbers = params.numbers(key);
    if (numbers.size() != 3) throw std::invalid_argument(std::string("cluster_match ") + key + " must have 3 values");
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(numbers[axis])) throw std::invalid_argument(std::string("cluster_match ") + key + " must be finite");
      value[axis] = numbers[axis];
    }
  };
  box("ego_min_m", config.ego_min_m);
  box("ego_max_m", config.ego_max_m);
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
      config.min_pose_shape_ratio > 1)
    throw std::invalid_argument("invalid cluster_match settings");
  for (std::size_t axis = 0; axis < 3; ++axis)
    if (config.ego_min_m[axis] > config.ego_max_m[axis]) throw std::invalid_argument("cluster_match ego box is inverted");
  const std::string bytes = params.asset("models_file");
  config.models = NpcModelReader(bytes).read();
  return config;
}
}  // namespace object_detector
