#pragma once
#include <path_planner/cores/integrated_prediction.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline IntegratedPredictionParameters loadIntegratedPredictionParameters(const node_pipeline::ParamReader& reader) {
  IntegratedPredictionParameters p;
  p.continuity_gap_sec=reader.number("continuity_gap_sec");
  p.continuity_gap_frames=reader.number("continuity_gap_frames");
  p.fit_window_sec=reader.number("fit_window_sec");
  p.fit_window_frames=reader.number("fit_window_frames");
  if(!std::isfinite(p.continuity_gap_sec) || p.continuity_gap_sec<=0 ||
     !std::isfinite(p.continuity_gap_frames) || p.continuity_gap_frames<0 ||
     !std::isfinite(p.fit_window_sec) || p.fit_window_sec<=0 ||
     !std::isfinite(p.fit_window_frames) || p.fit_window_frames<0)
    throw std::invalid_argument("invalid integrated prediction sample limits");
  return p;
}
}  // namespace path_planner
