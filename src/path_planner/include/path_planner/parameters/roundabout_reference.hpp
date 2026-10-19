#pragma once
#include <path_planner/cores/roundabout_reference.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace path_planner {
inline RoundaboutReferenceParameters loadRoundaboutReferenceParameters(const node_pipeline::ParamReader& params) {
  RoundaboutReferenceParameters p;
  p.reference_start_m = params.number("reference_start_m");
  p.reference_end_m = params.number("reference_end_m");
  p.control_step_m = params.number("control_step_m");
  p.integration_step_m = params.number("integration_step_m");
  p.sample_step_m = params.number("sample_step_m");
  p.local_path_length_m = params.number("local_path_length_m");
  p.entry_shift_m = params.number("entry_shift_m");
  p.entry_shift_center_m = params.number("entry_shift_center_m");
  p.entry_shift_half_length_m = params.number("entry_shift_half_length_m");
  p.exit_shift_m = params.number("exit_shift_m");
  p.exit_shift_center_m = params.number("exit_shift_center_m");
  p.exit_shift_half_length_m = params.number("exit_shift_half_length_m");
  for (double value : {p.reference_start_m, p.reference_end_m, p.control_step_m, p.integration_step_m,
                       p.sample_step_m, p.local_path_length_m, p.entry_shift_m,
                       p.entry_shift_center_m, p.entry_shift_half_length_m, p.exit_shift_m,
                       p.exit_shift_center_m, p.exit_shift_half_length_m})
    if (!std::isfinite(value)) throw std::invalid_argument("roundabout_reference parameters must be finite");
  if (p.reference_start_m < 0 || p.reference_end_m <= p.reference_start_m ||
      p.reference_end_m - p.reference_start_m > 500 || p.control_step_m < 0.5 || p.control_step_m > 5 ||
      p.integration_step_m < 0.01 || p.integration_step_m > 0.1 ||
      p.sample_step_m < 0.1 || p.sample_step_m > 1 ||
      p.local_path_length_m < 1 || p.local_path_length_m > 100 ||
      p.entry_shift_m < 0 || p.entry_shift_m > 0.5 || p.entry_shift_half_length_m < 2 ||
      p.entry_shift_center_m - p.entry_shift_half_length_m < p.reference_start_m ||
      p.entry_shift_center_m + p.entry_shift_half_length_m > p.reference_end_m ||
      p.exit_shift_m < 0 || p.exit_shift_m > 0.5 || p.exit_shift_half_length_m < 2 ||
      p.exit_shift_center_m - p.exit_shift_half_length_m < p.reference_start_m ||
      p.exit_shift_center_m + p.exit_shift_half_length_m > p.reference_end_m ||
      p.reference_end_m - p.reference_start_m < p.control_step_m)
    throw std::invalid_argument("roundabout_reference parameter range is invalid");
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
      throw std::invalid_argument("roundabout_reference parameters must be finite and positive");
  for (double value : {p.collision_margin_m, p.prediction_uncertainty_mps})
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("roundabout_reference margins must be finite and nonnegative");
  if (p.time_step_sec < 0.05 || p.time_step_sec > 0.5 || p.command_step_sec < 0.01 ||
      p.command_step_sec > 1.0)
    throw std::invalid_argument("roundabout_reference time steps are unbounded or inconsistent");

  p.planning_horizon_sec = params.number("planning_horizon_sec");
  if (!std::isfinite(p.planning_horizon_sec) || p.planning_horizon_sec < 3.0 || p.planning_horizon_sec > 5.0)
    throw std::invalid_argument("planning_horizon_sec must be between 3 and 5");

  p.following_time_gap_sec = params.number("following_time_gap_sec");
  if (!std::isfinite(p.following_time_gap_sec) || p.following_time_gap_sec < 0.5 ||
      p.following_time_gap_sec > 3.0)
    throw std::invalid_argument("following_time_gap_sec must be between 0.5 and 3.0");

  p.entered_min_speed_mps = params.number("entered_min_speed_mps");
  if (!std::isfinite(p.entered_min_speed_mps) || p.entered_min_speed_mps < 0.1 ||
      p.entered_min_speed_mps > p.desired_speed_mps || p.entered_min_speed_mps > 2.0)
    throw std::invalid_argument("entered_min_speed_mps must be positive and within cruise speed");
  p.approach_speed_mps = params.number("approach_speed_mps");
  p.curve_speed_ratio = params.number("curve_speed_ratio");
  p.entry_line_station_m = params.number("entry_line_station_m");
  p.entry_stop_margin_m = params.number("entry_stop_margin_m");
  p.entry_decision_distance_m = params.number("entry_decision_distance_m");
  p.entry_check_length_m = params.number("entry_check_length_m");
  p.entry_gap_confirmation_sec = params.number("entry_gap_confirmation_sec");
  for (double value : {p.approach_speed_mps, p.curve_speed_ratio, p.entry_line_station_m, p.entry_stop_margin_m,
                       p.entry_decision_distance_m, p.entry_check_length_m, p.entry_gap_confirmation_sec})
    if (!std::isfinite(value)) throw std::invalid_argument("entry parameters must be finite");
  if (p.approach_speed_mps < 0.5 || p.approach_speed_mps > p.desired_speed_mps ||
      p.curve_speed_ratio < 0.5 || p.curve_speed_ratio > 1.0 ||
      p.entry_line_station_m <= p.reference_start_m + p.vehicle_front_m + p.entry_stop_margin_m ||
      p.entry_line_station_m >= p.reference_end_m ||
      p.entry_stop_margin_m < 0.3 || p.entry_stop_margin_m > 3.0 ||
      p.entry_decision_distance_m < 1.0 || p.entry_decision_distance_m > 5.0 ||
      p.entry_check_length_m < p.vehicle_front_m + p.entry_stop_margin_m ||
      p.entry_check_length_m > 20.0 || p.entry_check_length_m > p.local_path_length_m ||
      p.entry_gap_confirmation_sec < 0.1 || p.entry_gap_confirmation_sec > 1.0)
    throw std::invalid_argument("entry parameter range is invalid");

  std::istringstream asset(params.asset("global_path_file"));
  std::string line;
  double length_m = 0.0;
  while (std::getline(asset, line)) {
    std::istringstream row(line);
    row >> std::ws;
    if (row.eof()) continue;
    geometry_msgs::Point point;
    if (!(row >> point.x >> point.y >> point.z) ||
        !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::invalid_argument("global_path_file requires finite map x y z");
    row >> std::ws;
    if (!row.eof()) throw std::invalid_argument("unexpected global_path_file columns");
    if (!p.global_path.empty()) {
      const auto& previous = p.global_path.back();
      length_m += std::hypot(point.x - previous.x, point.y - previous.y);
    }
    p.global_path.push_back(point);
  }
  if (length_m < p.reference_end_m)
    throw std::invalid_argument("global_path_file does not cover the roundabout reference interval");
  return p;
}
}  // namespace path_planner
