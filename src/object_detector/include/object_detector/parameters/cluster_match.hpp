#pragma once
#include <object_detector/cores/cluster_match.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace object_detector {
// 시작 시 little-endian NPCM v1 정적 모델 자산을 해석한다.
class NpcModelReader {
 public:
  explicit NpcModelReader(const std::string& bytes) : bytes_(bytes) {}

  std::vector<ModelTemplate> read() {
    if (take(4) != "NPCM" || u32() != 1) throw std::invalid_argument("models_file is not an NPCM v1 asset");
    const std::uint32_t model_count = u32();
    const std::uint32_t point_count = u32();
    struct Entry { ModelTemplate model; std::uint32_t begin, count; };
    std::vector<Entry> entries;
    entries.reserve(model_count);
    for (std::uint32_t i = 0; i < model_count; ++i) {
      Entry entry;
      entry.model.label = take(u16());
      entry.model.semantic_label = take(u16());
      entry.begin = u32();
      entry.count = u32();
      entry.model.collider_available = take(1)[0] != 0;
      auto& pose = entry.model.collider_pose;
      pose.position.x = f64();
      pose.position.y = f64();
      pose.position.z = f64();
      pose.orientation.x = f64();
      pose.orientation.y = f64();
      pose.orientation.z = f64();
      pose.orientation.w = f64();
      entry.model.collider_size.x = f64();
      entry.model.collider_size.y = f64();
      entry.model.collider_size.z = f64();
      if (static_cast<std::uint64_t>(entry.begin) + entry.count > point_count)
        throw std::invalid_argument("models_file point range exceeds data: " + entry.model.label);
      entries.push_back(std::move(entry));
    }
    const std::size_t points_offset = offset_;
    if (bytes_.size() - points_offset != static_cast<std::size_t>(point_count) * 3 * sizeof(float))
      throw std::invalid_argument("models_file point data size mismatch");
    std::vector<ModelTemplate> models;
    models.reserve(entries.size());
    for (auto& entry : entries) {
      if (entry.count == 0) continue;
      entry.model.points.reserve(entry.count);
      for (std::uint32_t i = entry.begin; i < entry.begin + entry.count; ++i) {
        float xyz[3];
        std::memcpy(xyz, bytes_.data() + points_offset + static_cast<std::size_t>(i) * sizeof(xyz), sizeof(xyz));
        entry.model.points.emplace_back(xyz[0], xyz[1], xyz[2]);
      }
      models.push_back(std::move(entry.model));
    }
    return models;
  }

 private:
  std::string take(std::size_t size) {
    if (bytes_.size() - offset_ < size) throw std::invalid_argument("models_file is truncated");
    std::string value = bytes_.substr(offset_, size);
    offset_ += size;
    return value;
  }
  template<class T> T scalar() {
    const auto raw = take(sizeof(T));
    T value;
    std::memcpy(&value, raw.data(), sizeof(T));
    return value;
  }
  std::uint16_t u16() { return scalar<std::uint16_t>(); }
  std::uint32_t u32() { return scalar<std::uint32_t>(); }
  double f64() { return scalar<double>(); }

  const std::string& bytes_;
  std::size_t offset_ = 0;
};

inline ClusterMatchParameters loadClusterMatchParameters(const node_pipeline::ParamReader& params) {
  ClusterMatchParameters config;
  config.voxel_leaf_m = params.number("voxel_leaf_m");
  config.cluster_tolerance_m = params.number("cluster_tolerance_m");
  config.min_cluster_points = static_cast<int>(params.integer("min_cluster_points"));
  config.max_cluster_points = static_cast<int>(params.integer("max_cluster_points"));
  config.ground_distance_m = params.number("ground_distance_m");
  config.ground_max_tilt_rad = params.number("ground_max_tilt_rad");
  config.ground_seed_max_z_m = params.number("ground_seed_max_z_m");
  config.ground_cell_m = params.number("ground_cell_m");
  config.ground_region_m = params.number("ground_region_m");
  config.ground_min_inliers = static_cast<int>(params.integer("ground_min_inliers"));
  config.normal_radius_m = params.number("normal_radius_m");
  config.feature_radius_m = params.number("feature_radius_m");
  config.registration_iterations = static_cast<int>(params.integer("registration_iterations"));
  config.icp_iterations = static_cast<int>(params.integer("icp_iterations"));
  config.correspondence_distance_m = params.number("correspondence_distance_m");
  config.min_inlier_fraction = params.number("min_inlier_fraction");
  config.max_rmse_m = params.number("max_rmse_m");
  config.ambiguity_score_margin = params.number("ambiguity_score_margin");
  config.min_pose_shape_ratio = params.number("min_pose_shape_ratio");
  config.model_max_tilt_rad = params.number("model_max_tilt_rad");
  config.exclude_ego = params.flag("exclude_ego");
  const auto ego_min_m = params.numbers("ego_min_m");
  for (std::size_t axis = 0; axis < 3; ++axis) config.ego_min_m[axis] = ego_min_m[axis];
  const auto ego_max_m = params.numbers("ego_max_m");
  for (std::size_t axis = 0; axis < 3; ++axis) config.ego_max_m[axis] = ego_max_m[axis];
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
  const auto bytes = params.asset("models_file");
  config.models = NpcModelReader(bytes).read();
  return config;
}
}  // namespace object_detector
