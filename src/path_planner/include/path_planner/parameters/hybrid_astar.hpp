#pragma once
#include <path_planner/cores/hybrid_astar.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <unordered_set>

namespace path_planner {
inline HybridAstarParameters loadHybridAstarParameters(const node_pipeline::ParamReader& params) {
  HybridAstarParameters p;
  p.path_length_m = params.number("path_length_m");
  p.sample_step_m = params.number("sample_step_m");
  p.primitive_length_m = params.number("primitive_length_m");
  p.grid_resolution_m = params.number("grid_resolution_m");
  p.max_steer_rad = params.number("max_steer_rad");
  p.wheelbase_m = params.number("wheelbase_m");
  p.vehicle_front_m = params.number("vehicle_front_m");
  p.vehicle_rear_m = params.number("vehicle_rear_m");
  p.vehicle_half_width_m = params.number("vehicle_half_width_m");
  p.collision_margin_m = params.number("collision_margin_m");
  p.road_margin_m = params.number("road_margin_m");
  p.transition_lengths_m = params.numbers("transition_lengths_m");
  p.steering_candidates_deg = params.numbers("steering_candidates_deg");
  const auto yaw_bins = params.integer("yaw_bins");
  const auto max_expansions = params.integer("max_expansions");
  if (yaw_bins < 8 || yaw_bins > 720)
    throw std::invalid_argument("yaw_bins must be between 8 and 720");
  if (max_expansions < 1 || max_expansions > 100000)
    throw std::invalid_argument("max_expansions must be between 1 and 100000");
  p.yaw_bins = static_cast<int>(yaw_bins);
  p.max_expansions = static_cast<int>(max_expansions);
  for (double value : {p.path_length_m, p.sample_step_m, p.primitive_length_m,
                       p.grid_resolution_m, p.max_steer_rad, p.wheelbase_m,
                       p.vehicle_front_m, p.vehicle_rear_m, p.vehicle_half_width_m})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("hybrid_astar lengths and steering must be finite and positive");
  for (double value : {p.collision_margin_m, p.road_margin_m})
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("hybrid_astar margins must be finite and nonnegative");
  if (p.path_length_m > 500.0 || p.sample_step_m < 0.05 ||
      p.sample_step_m > p.primitive_length_m || p.primitive_length_m > 5.0 ||
      p.primitive_length_m > p.path_length_m || p.grid_resolution_m < 0.05 ||
      p.grid_resolution_m > 5.0 || p.max_steer_rad > 0.698132)
    throw std::invalid_argument("hybrid_astar length, sampling, grid, or steering limits are inconsistent");
  if (p.transition_lengths_m.empty() || p.transition_lengths_m.size() > 128)
    throw std::invalid_argument("transition_lengths_m needs between 1 and 128 candidates");
  for (double value : p.transition_lengths_m)
    if (!std::isfinite(value) || value <= 0.0 || value > p.path_length_m)
      throw std::invalid_argument("transition_lengths_m must be positive and within path_length_m");
  if (p.steering_candidates_deg.empty() || p.steering_candidates_deg.size() > 33)
    throw std::invalid_argument("steering_candidates_deg needs between 1 and 33 candidates");
  std::unordered_set<double> steering_candidates;
  for (double value : p.steering_candidates_deg) {
    if (!std::isfinite(value) || std::abs(value * 3.14159265358979323846 / 180.0) > p.max_steer_rad)
      throw std::invalid_argument("steering_candidates_deg must be finite and within max_steer_rad");
    if (!steering_candidates.insert(value).second)
      throw std::invalid_argument("steering_candidates_deg must be unique");
  }
  if (steering_candidates.count(0.0) == 0)
    throw std::invalid_argument("steering_candidates_deg must contain the straight primitive 0");
  return p;
}
}  // namespace path_planner
