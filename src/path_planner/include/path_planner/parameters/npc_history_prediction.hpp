#pragma once
#include <path_planner/cores/npc_history_prediction.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace path_planner {
inline NpcHistoryPredictionParameters loadNpcHistoryPredictionParameters(const node_pipeline::ParamReader& params) {
  NpcHistoryPredictionParameters result;
  result.nominal_fit_slack_m = params.number("nominal_fit_slack_m");
  result.path_rms_limit_m = params.number("path_rms_limit_m");
  if (!std::isfinite(result.nominal_fit_slack_m) || result.nominal_fit_slack_m < 0.0 ||
      !std::isfinite(result.path_rms_limit_m) || result.path_rms_limit_m <= 0.0)
    throw std::invalid_argument("invalid npc_history_prediction fit limits");
  std::istringstream asset(params.asset("trajectory_file"));
  std::string format;
  std::size_t count = 0;
  if (!(asset >> format >> count) || format != "npc_bbox_trajectory_v1" || count != 774)
    throw std::invalid_argument("invalid npc trajectory model header");
  result.curves.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    NpcHistoryPredictionParameters::Curve curve;
    std::size_t points = 0;
    if (!(asset >> curve.model_label >> curve.actor_label >> curve.length_m >> curve.nominal_rate_mps >>
          curve.history_sec >> points) || points != 512 || !std::isfinite(curve.length_m) || curve.length_m <= 0.0 ||
        !std::isfinite(curve.nominal_rate_mps) || curve.nominal_rate_mps < 0.0 ||
        (curve.history_sec != 10.0 && curve.history_sec != 20.0 && curve.history_sec != 30.0))
      throw std::invalid_argument("invalid npc trajectory curve");
    curve.points.reserve(points);
    for (std::size_t j = 0; j < points; ++j) {
      double x, y, z;
      if (!(asset >> x >> y >> z) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        throw std::invalid_argument("invalid npc trajectory point");
      curve.points.emplace_back(x, y, z);
    }
    double length = 0.0;
    for (std::size_t j = 0; j < points; ++j) {
      const Eigen::Vector3d delta = curve.points[(j+1)%points]-curve.points[j];
      if (delta.head<2>().squaredNorm() <= 1e-16)
        throw std::invalid_argument("degenerate npc trajectory segment");
      length += delta.norm();
    }
    if (std::abs(length-curve.length_m) > 1e-7)
      throw std::invalid_argument("npc trajectory length mismatch");
    result.curves.push_back(std::move(curve));
  }
  if (asset >> format) throw std::invalid_argument("unexpected npc trajectory data");
  return result;
}
}  // namespace path_planner
