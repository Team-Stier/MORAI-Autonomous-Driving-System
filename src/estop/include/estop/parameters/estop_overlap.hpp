#pragma once
#include <estop/cores/estop_overlap.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace estop {
inline EstopOverlapParameters loadEstopOverlapParameters(const node_pipeline::ParamReader& params) {
  EstopOverlapParameters p;
  p.vehicle_front_m = params.number("vehicle_front_m");
  p.vehicle_rear_m = params.number("vehicle_rear_m");
  p.vehicle_half_width_m = params.number("vehicle_half_width_m");
  p.longitudinal_margin_m = params.number("longitudinal_margin_m");
  p.lateral_margin_m = params.number("lateral_margin_m");
  p.object_margin_m = params.number("object_margin_m");
  p.time_tolerance_sec = params.number("time_tolerance_sec");
  const auto samples = params.integer("time_samples");
  p.horizon_sec = params.number("horizon_sec");
  p.horizon_m = params.number("horizon_m");
  p.minimum_object_speed_mps = params.number("minimum_object_speed_mps");
  p.minimum_path_speed_mps = params.number("minimum_path_speed_mps");
  p.ignore_behind = params.flag("ignore_behind");
  p.release_after_sec = params.number("release_after_sec");
  for (double value : {p.vehicle_front_m, p.vehicle_rear_m, p.vehicle_half_width_m, p.horizon_sec, p.horizon_m,
                       p.minimum_path_speed_mps})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("estop vehicle size, horizon and minimum path speed must be finite and positive");
  for (double value : {p.longitudinal_margin_m, p.lateral_margin_m, p.object_margin_m, p.time_tolerance_sec,
                       p.minimum_object_speed_mps, p.release_after_sec})
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("estop margins, tolerances and release time must be finite and nonnegative");
  if (samples < 1 || samples > 101) throw std::invalid_argument("time_samples must be between 1 and 101");
  p.time_samples = static_cast<int>(samples);
  return p;
}
}  // namespace estop
