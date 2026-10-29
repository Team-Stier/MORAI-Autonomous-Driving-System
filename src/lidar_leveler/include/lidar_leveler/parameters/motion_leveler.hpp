#pragma once
#include <lidar_leveler/cores/motion_leveler.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
namespace lidar_leveler {
inline MotionLevelerParameters loadMotionLevelerParameters(const node_pipeline::ParamReader& params) {
  MotionLevelerParameters p;
  if(params.has("self_mask_padding_m"))p.self_mask_padding_m=params.number("self_mask_padding_m");
  if(!std::isfinite(p.self_mask_padding_m) || p.self_mask_padding_m<0 || p.self_mask_padding_m>.1)
    throw std::invalid_argument("invalid self mask padding");
  p.history_duration_sec=params.number("history_duration_sec");
  p.maximum_extrapolation_sec=params.number("maximum_extrapolation_sec");
  p.compensate_translation=params.flag("compensate_translation");
  p.minimum_plane_improvement_ratio=params.number("minimum_plane_improvement_ratio");
  if(!std::isfinite(p.history_duration_sec) || p.history_duration_sec<0.3 || p.history_duration_sec>3 ||
      !std::isfinite(p.maximum_extrapolation_sec) || p.maximum_extrapolation_sec<0 || p.maximum_extrapolation_sec>0.1)
    throw std::invalid_argument("invalid motion leveling history");
  if(!std::isfinite(p.minimum_plane_improvement_ratio) || p.minimum_plane_improvement_ratio<=0 || p.minimum_plane_improvement_ratio>1)
    throw std::invalid_argument("invalid plane improvement ratio");
  return p;
}
}  // namespace lidar_leveler
