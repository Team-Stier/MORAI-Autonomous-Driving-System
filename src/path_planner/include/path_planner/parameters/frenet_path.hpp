#pragma once
#include <path_planner/cores/frenet_path.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace path_planner {
inline FrenetPathParameters loadFrenetPathParameters(const node_pipeline::ParamReader& params) {
  FrenetPathParameters parameters;
  parameters.path_length_m = params.number("path_length_m");
  parameters.sample_step_m = params.number("sample_step_m");
  parameters.reference_step_m = params.number("reference_step_m");
  parameters.transition_lengths_m = params.numbers("transition_lengths_m");
  parameters.lateral_offsets_m = params.numbers("lateral_offsets_m");
  parameters.vehicle_front_m = params.number("vehicle_front_m");
  parameters.vehicle_rear_m = params.number("vehicle_rear_m");
  parameters.vehicle_half_width_m = params.number("vehicle_half_width_m");
  for (double value : {parameters.path_length_m, parameters.sample_step_m, parameters.reference_step_m,
                       parameters.vehicle_front_m,
                       parameters.vehicle_rear_m, parameters.vehicle_half_width_m})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("Frenet path lengths must be finite and positive");
  if (parameters.transition_lengths_m.empty() || parameters.lateral_offsets_m.empty())
    throw std::invalid_argument("Frenet candidate parameter lists must be nonempty");
  for (double value : parameters.transition_lengths_m)
    if (!std::isfinite(value) || value <= 0.0 || value > parameters.path_length_m)
      throw std::invalid_argument("transition_lengths_m must be finite, positive, and within path_length_m");
  for (double value : parameters.lateral_offsets_m)
    if (!std::isfinite(value))
      throw std::invalid_argument("lateral_offsets_m must be finite");
  return parameters;
}
}  // namespace path_planner
