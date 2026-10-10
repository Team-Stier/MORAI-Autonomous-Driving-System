#pragma once
#include <path_planner/cores/frenet_speed_path.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace path_planner {
inline FrenetSpeedPathParameters loadFrenetSpeedPathParameters(const node_pipeline::ParamReader& params) {
  FrenetSpeedPathParameters p;
  p.path_length_m = params.number("path_length_m");
  p.sample_step_m = params.number("sample_step_m");
  p.reference_step_m = params.number("reference_step_m");
  p.transition_lengths_m = params.numbers("transition_lengths_m");
  p.lateral_offsets_m = params.numbers("lateral_offsets_m");
  p.vehicle_front_m = params.number("vehicle_front_m");
  p.vehicle_rear_m = params.number("vehicle_rear_m");
  p.vehicle_half_width_m = params.number("vehicle_half_width_m");
  p.desired_speed_mps = params.number("desired_speed_mps");
  p.max_acceleration_mps2 = params.number("max_acceleration_mps2");
  p.max_deceleration_mps2 = params.number("max_deceleration_mps2");
  p.max_jerk_mps3 = params.number("max_jerk_mps3");
  p.max_lateral_acceleration_mps2 = params.number("max_lateral_acceleration_mps2");
  p.time_step_sec = params.number("time_step_sec");
  p.command_step_sec = params.number("command_step_sec");
  p.collision_margin_m = params.number("collision_margin_m");
  p.prediction_uncertainty_mps = params.number("prediction_uncertainty_mps");
  const auto beam_width = params.integer("beam_width");
  if (beam_width < 1 || beam_width > 4096)
    throw std::invalid_argument("beam_width must be between 1 and 4096");
  p.beam_width = static_cast<int>(beam_width);
  for (double value : {p.path_length_m, p.sample_step_m, p.reference_step_m,
                       p.vehicle_front_m, p.vehicle_rear_m, p.vehicle_half_width_m,
                       p.desired_speed_mps, p.max_acceleration_mps2, p.max_deceleration_mps2,
                       p.max_jerk_mps3, p.max_lateral_acceleration_mps2,
                       p.time_step_sec, p.command_step_sec})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("frenet_speed_path parameters must be finite and positive");
  if (p.transition_lengths_m.empty() || p.lateral_offsets_m.empty())
    throw std::invalid_argument("Frenet candidate parameter lists must be nonempty");
  for (double value : p.transition_lengths_m)
    if (!std::isfinite(value) || value <= 0.0 || value > p.path_length_m)
      throw std::invalid_argument("transition_lengths_m must be finite, positive, and within path_length_m");
  for (double value : p.lateral_offsets_m)
    if (!std::isfinite(value))
      throw std::invalid_argument("lateral_offsets_m must be finite");
  for (double value : {p.collision_margin_m, p.prediction_uncertainty_mps})
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("frenet_speed_path margins must be finite and nonnegative");
  if (p.time_step_sec < 0.05 || p.time_step_sec > 0.5 || p.command_step_sec < 0.01 ||
      p.command_step_sec > p.time_step_sec)
    throw std::invalid_argument("frenet_speed_path time steps are unbounded or inconsistent");
  return p;
}
}  // namespace path_planner
