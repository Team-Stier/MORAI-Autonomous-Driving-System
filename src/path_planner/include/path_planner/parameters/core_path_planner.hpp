#pragma once
#include <path_planner/cores/core_path_planner.hpp>
#include <node_pipeline/extension.hpp>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace path_planner {
inline CorePathPlannerParameters loadCorePathPlannerParameters(const node_pipeline::ParamReader& params) {
  CorePathPlannerParameters parameters;
  auto& path = parameters.path;
  path.vehicle.front_m = params.number("vehicle/front_m");
  path.vehicle.rear_m = params.number("vehicle/rear_m");
  path.vehicle.width_m = params.number("vehicle/width_m");
  path.horizon_m = params.number("horizon_m");
  path.min_horizon_m = params.number("min_horizon_m");
  path.road_boundary_step_m = params.number("road_boundary_step_m");
  path.dp_step_m = params.number("dp_step_m");
  path.lateral_step_m = params.number("lateral_step_m");
  path.dp_slope_sample = params.number("dp_slope_sample");
  path.output_step_m = params.number("output_step_m");
  path.corridor_half_width_m = params.number("corridor_half_width_m");
  path.obstacle_margin_m = params.number("obstacle_margin_m");
  path.road_margin_m = params.number("road_margin_m");
  path.max_curvature_inv_m = params.number("max_curvature_inv_m");
  path.max_lateral_accel_mps2 = params.number("max_lateral_accel_mps2");
  path.max_speed_mps = params.number("max_speed_mps");
  path.max_lateral_slope = params.number("max_lateral_slope");
  path.max_initial_heading_error_rad = params.number("max_initial_heading_error_rad");
  path.max_initial_distance_m = params.number("max_initial_distance_m");
  path.reference_weight = params.number("reference_weight");
  path.dp_tracking_weight = params.number("dp_tracking_weight");
  path.slope_weight = params.number("slope_weight");
  path.bending_weight = params.number("bending_weight");
  path.jerk_weight = params.number("jerk_weight");
  path.previous_weight = params.number("previous_weight");
  path.qp_rho = params.number("qp_rho");
  path.qp_tolerance = params.number("qp_tolerance");
  path.max_acceleration_mps2 = params.number("max_acceleration_mps2");
  path.max_deceleration_mps2 = params.number("max_deceleration_mps2");
  path.max_longitudinal_jerk_mps3 = params.number("max_longitudinal_jerk_mps3");
  path.speed_time_step_sec = params.number("speed_time_step_sec");
  path.trajectory_output_step_sec = params.number("trajectory_output_step_sec");
  path.speed_tracking_weight = params.number("speed_tracking_weight");
  path.acceleration_weight = params.number("acceleration_weight");
  path.longitudinal_jerk_weight = params.number("longitudinal_jerk_weight");
  path.trajectory_horizon_m = params.number("trajectory_horizon_m");
  const auto count = [&](const std::string& key) {
    const auto value = params.integer(key);
    if (value < 1 || value > 1000000) throw std::invalid_argument(key + " must be in [1, 1000000]");
    return value;
  };
  path.qp_max_iterations = static_cast<int>(count("qp_max_iterations"));
  path.max_lateral_samples = static_cast<std::size_t>(count("max_lateral_samples"));
  path.speed_beam_width = static_cast<std::size_t>(count("speed_beam_width"));
  // PathGenerationCore 생성자가 나머지 값의 범위를 시작 시 검사한다.
  // 통합 탐색 이산화 한도는 process()에서만 검사되므로 시작 시 확인한다.
  if (path.trajectory_horizon_m / path.output_step_m > 500 || path.trajectory_horizon_m / path.dp_step_m > 100)
    throw std::invalid_argument("trajectory_horizon_m exceeds the integrated planner discretization");
  return parameters;
}
}  // namespace path_planner
