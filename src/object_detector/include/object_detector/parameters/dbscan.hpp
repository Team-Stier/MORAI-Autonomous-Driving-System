#pragma once
#include <object_detector/cores/dbscan.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace object_detector {
// #C5_PARAMETERS
// 최신 Pipeline의 ParamReader로 이 Core의 YAML 계수만 읽어 생성자에 값으로 넘긴다.
// 거리·ROI는 m, MinPts·cluster 크기는 voxel 처리 이후 점 개수다.
// 누락·타입 오류는 ParamReader가 거부하며 잘못된 값으로 synthetic 기본값에 복귀하지 않는다.
// 이 로더는 설정 검증만 수행하고 점군 검사·ROI 계산·clustering·frame 변환은 하지 않는다.
// RDDF ROI와 합칠 때 고정 범위의 책임을 먼저 합의하고 #ROI_INTEGRATION을 따른다.
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
  if (!std::isfinite(parameters.roi_x_min) || !std::isfinite(parameters.roi_x_max) ||
      !std::isfinite(parameters.roi_y_min) || !std::isfinite(parameters.roi_y_max) ||
      !std::isfinite(parameters.roi_z_min) || !std::isfinite(parameters.roi_z_max) ||
      parameters.roi_x_min > parameters.roi_x_max || parameters.roi_y_min > parameters.roi_y_max ||
      parameters.roi_z_min > parameters.roi_z_max ||
      !std::isfinite(parameters.voxel_leaf_size) || parameters.voxel_leaf_size <= 0 ||
      !std::isfinite(parameters.dbscan_eps) || parameters.dbscan_eps <= 0 ||
      min_points <= 0 || min_points > std::numeric_limits<int>::max() ||
      min_cluster <= 0 || min_cluster > std::numeric_limits<int>::max() ||
      max_cluster < min_cluster || max_cluster > std::numeric_limits<int>::max())
    throw std::invalid_argument("invalid DBSCAN parameters");
  parameters.dbscan_min_points = static_cast<int>(min_points);
  parameters.min_cluster_size = static_cast<int>(min_cluster);
  parameters.max_cluster_size = static_cast<int>(max_cluster);
  return parameters;
}
}  // namespace object_detector
