#pragma once
#include <path_planner/cores/roundabout_merge.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace path_planner {
inline RoundaboutMergeParameters loadRoundaboutMergeParameters(const node_pipeline::ParamReader& params) {
  RoundaboutMergeParameters p;
  p.local_path_length_m = params.number("local_path_length_m");
  p.desired_speed_mps = params.number("desired_speed_mps");
  p.max_acceleration_mps2 = params.number("max_acceleration_mps2");
  p.max_deceleration_mps2 = params.number("max_deceleration_mps2");
  p.max_jerk_mps3 = params.number("max_jerk_mps3");
  p.max_lateral_acceleration_mps2 = params.number("max_lateral_acceleration_mps2");
  p.time_step_sec = params.number("time_step_sec");
  p.command_step_sec = params.number("command_step_sec");
  p.collision_margin_m = params.number("collision_margin_m");
  p.prediction_uncertainty_mps = params.number("prediction_uncertainty_mps");
  p.vehicle_front_m = params.number("vehicle_front_m");
  p.vehicle_rear_m = params.number("vehicle_rear_m");
  p.vehicle_half_width_m = params.number("vehicle_half_width_m");
  const auto beam_width = params.integer("beam_width");
  if (beam_width < 1 || beam_width > 4096)
    throw std::invalid_argument("beam_width must be between 1 and 4096");
  p.beam_width = static_cast<int>(beam_width);
  for (double value : {p.local_path_length_m, p.desired_speed_mps, p.max_acceleration_mps2,
                       p.max_deceleration_mps2, p.max_jerk_mps3, p.max_lateral_acceleration_mps2,
                       p.time_step_sec, p.command_step_sec, p.vehicle_front_m, p.vehicle_rear_m,
                       p.vehicle_half_width_m})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("roundabout_merge parameters must be finite and positive");
  for (double value : {p.collision_margin_m, p.prediction_uncertainty_mps})
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("roundabout_merge margins must be finite and nonnegative");
  if (p.time_step_sec < 0.05 || p.time_step_sec > 0.5 || p.command_step_sec < 0.01 ||
      p.command_step_sec > p.time_step_sec)
    throw std::invalid_argument("roundabout_merge time steps are unbounded or inconsistent");

  std::istringstream asset(params.asset("global_path_file"));
  std::string line;
  bool has_segment = false;
  while (std::getline(asset, line)) {
    std::istringstream row(line);
    row >> std::ws;
    if (row.eof()) continue;
    geometry_msgs::Point point;
    if (!(row >> point.x >> point.y >> point.z) ||
        !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::invalid_argument("global_path_file rows must contain finite x y z in map meters");
    row >> std::ws;
    if (!row.eof()) throw std::invalid_argument("unexpected global_path_file columns");
    if (!p.global_path.empty()) {
      const auto& previous = p.global_path.back();
      has_segment = has_segment || point.x != previous.x || point.y != previous.y;
    }
    p.global_path.push_back(point);
  }
  if (!has_segment) throw std::invalid_argument("global_path_file needs a nonzero XY segment");
  return p;
}
}  // namespace path_planner
