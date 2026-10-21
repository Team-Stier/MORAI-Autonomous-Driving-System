#pragma once
#include <path_planner/cores/core_path_planner.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
#include <sstream>

namespace path_planner {
inline PathParameters loadCorePathPlannerParameters(const node_pipeline::ParamReader& params) {
  PathParameters p;
  if (params.has("vehicle/front_m")) p.vehicle.front_m = params.number("vehicle/front_m");
  if (params.has("vehicle/rear_m")) p.vehicle.rear_m = params.number("vehicle/rear_m");
  if (params.has("vehicle/width_m")) p.vehicle.width_m = params.number("vehicle/width_m");
  if (params.has("horizon_m")) p.horizon_m = params.number("horizon_m");
  if (params.has("min_horizon_m")) p.min_horizon_m = params.number("min_horizon_m");
  if (params.has("road_boundary_step_m")) p.road_boundary_step_m = params.number("road_boundary_step_m");
  if (params.has("dp_step_m")) p.dp_step_m = params.number("dp_step_m");
  if (params.has("lateral_step_m")) p.lateral_step_m = params.number("lateral_step_m");
  if (params.has("dp_slope_sample")) p.dp_slope_sample = params.number("dp_slope_sample");
  if (params.has("output_step_m")) p.output_step_m = params.number("output_step_m");
  if (params.has("corridor_half_width_m")) p.corridor_half_width_m = params.number("corridor_half_width_m");
  if (params.has("obstacle_margin_m")) p.obstacle_margin_m = params.number("obstacle_margin_m");
  if (params.has("road_margin_m")) p.road_margin_m = params.number("road_margin_m");
  if (params.has("pedestrian_caution_speed_mps")) p.pedestrian_caution_speed_mps = params.number("pedestrian_caution_speed_mps");
  if (params.has("pedestrian_caution_distance_m")) p.pedestrian_caution_distance_m = params.number("pedestrian_caution_distance_m");
  if (params.has("pedestrian_near_preferred_speed_mps")) p.pedestrian_near_preferred_speed_mps = params.number("pedestrian_near_preferred_speed_mps");
  if (params.has("max_curvature_inv_m")) p.max_curvature_inv_m = params.number("max_curvature_inv_m");
  if (params.has("max_lateral_accel_mps2")) p.max_lateral_accel_mps2 = params.number("max_lateral_accel_mps2");
  if (params.has("preferred_lateral_accel_mps2")) p.preferred_lateral_accel_mps2 = params.number("preferred_lateral_accel_mps2");
  if (params.has("max_speed_mps")) p.max_speed_mps = params.number("max_speed_mps");
  if (params.has("normal_speed_parameters_file")) {
    p.normal_speed_limit_mps=node_pipeline::ParamReader::fromString(
        params.asset("normal_speed_parameters_file")).number("configured_target_speed_mps");
    p.exception_target_speed_mps=node_pipeline::ParamReader::fromString(
        params.asset("exception_speed_parameters_file")).number("configured_target_speed_mps");
    p.speed_limit_response_sec=params.number("speed_limit_response_sec");
    std::istringstream links(params.asset("speed_exemption_links_file"));std::string id;
    while(links>>id)p.speed_exemption_link_ids.insert(id);
    if(!std::isfinite(p.normal_speed_limit_mps) || p.normal_speed_limit_mps<=0 ||
       !std::isfinite(p.exception_target_speed_mps) || p.exception_target_speed_mps<p.normal_speed_limit_mps ||
       !std::isfinite(p.speed_limit_response_sec) || p.speed_limit_response_sec<0 ||
       p.speed_exemption_link_ids.empty())
      throw std::invalid_argument("invalid mission speed limits");
  }
  if (params.has("max_lateral_slope")) p.max_lateral_slope = params.number("max_lateral_slope");
  if (params.has("max_initial_heading_error_rad")) p.max_initial_heading_error_rad = params.number("max_initial_heading_error_rad");
  if (params.has("max_initial_distance_m")) p.max_initial_distance_m = params.number("max_initial_distance_m");
  if (params.has("reference_weight")) p.reference_weight = params.number("reference_weight");
  if (params.has("dp_tracking_weight")) p.dp_tracking_weight = params.number("dp_tracking_weight");
  if (params.has("slope_weight")) p.slope_weight = params.number("slope_weight");
  if (params.has("bending_weight")) p.bending_weight = params.number("bending_weight");
  if (params.has("jerk_weight")) p.jerk_weight = params.number("jerk_weight");
  if (params.has("previous_weight")) p.previous_weight = params.number("previous_weight");
  if (params.has("qp_rho")) p.qp_rho = params.number("qp_rho");
  if (params.has("qp_tolerance")) p.qp_tolerance = params.number("qp_tolerance");
  if (params.has("qp_max_iterations")) {
    const auto value = params.integer("qp_max_iterations");
    if (value < 1 || value > 2147483647) throw std::invalid_argument("invalid qp_max_iterations");
    p.qp_max_iterations = static_cast<int>(value);
  }
  if (params.has("max_lateral_samples")) {
    const auto value = params.integer("max_lateral_samples");
    if (value < 1) throw std::invalid_argument("invalid max_lateral_samples");
    p.max_lateral_samples = static_cast<std::size_t>(value);
  }
  if (params.has("max_acceleration_mps2")) p.max_acceleration_mps2 = params.number("max_acceleration_mps2");
  if (params.has("max_deceleration_mps2")) p.max_deceleration_mps2 = params.number("max_deceleration_mps2");
  if (params.has("max_longitudinal_jerk_mps3")) p.max_longitudinal_jerk_mps3 = params.number("max_longitudinal_jerk_mps3");
  if (params.has("speed_time_step_sec")) p.speed_time_step_sec = params.number("speed_time_step_sec");
  if (params.has("trajectory_output_step_sec")) p.trajectory_output_step_sec = params.number("trajectory_output_step_sec");
  if (params.has("speed_beam_width")) {
    const auto value = params.integer("speed_beam_width");
    if (value < 1 || value > 4096) throw std::invalid_argument("invalid speed_beam_width");
    p.speed_beam_width = static_cast<std::size_t>(value);
  }
  if (params.has("speed_tracking_weight")) p.speed_tracking_weight = params.number("speed_tracking_weight");
  if (params.has("acceleration_weight")) p.acceleration_weight = params.number("acceleration_weight");
  if (params.has("longitudinal_jerk_weight")) p.longitudinal_jerk_weight = params.number("longitudinal_jerk_weight");
  if (params.has("following_time_headway_sec")) p.following_time_headway_sec = params.number("following_time_headway_sec");
  if (params.has("following_min_gap_m")) p.following_min_gap_m = params.number("following_min_gap_m");
  if (params.has("trajectory_horizon_m")) p.trajectory_horizon_m = params.number("trajectory_horizon_m");
  for (double x : {p.vehicle.front_m,p.vehicle.rear_m,p.vehicle.width_m,p.horizon_m,p.dp_step_m,
                  p.lateral_step_m,p.dp_slope_sample,p.output_step_m,p.corridor_half_width_m,p.max_curvature_inv_m,
                  p.max_lateral_accel_mps2,p.max_speed_mps,p.max_lateral_slope,p.max_initial_heading_error_rad,
                  p.max_initial_distance_m,p.qp_rho,p.qp_tolerance,p.min_horizon_m,p.road_boundary_step_m,
                  p.max_acceleration_mps2,p.max_deceleration_mps2,p.max_longitudinal_jerk_mps3,
                  p.speed_time_step_sec,p.trajectory_output_step_sec,p.trajectory_horizon_m,
                  p.following_time_headway_sec})
    if (!std::isfinite(x) || x<=0.0) throw std::invalid_argument("nonpositive/nonfinite path parameter");
  for (double x : {p.obstacle_margin_m,p.road_margin_m,p.reference_weight,p.dp_tracking_weight,
                  p.slope_weight,p.bending_weight,p.jerk_weight,p.previous_weight,
                  p.speed_tracking_weight,p.acceleration_weight,p.longitudinal_jerk_weight,
                  p.following_min_gap_m,p.preferred_lateral_accel_mps2,
                  p.pedestrian_caution_speed_mps,p.pedestrian_caution_distance_m})
    if (!std::isfinite(x) || x<0.0) throw std::invalid_argument("negative/nonfinite path weight or margin");
  if (p.preferred_lateral_accel_mps2>p.max_lateral_accel_mps2 ||
      p.horizon_m/p.output_step_m>500 || p.horizon_m/p.dp_step_m>100 ||
      p.output_step_m>0.5 || p.road_boundary_step_m>0.5 || p.min_horizon_m>p.horizon_m ||
      p.qp_max_iterations<1 ||
      p.max_lateral_samples<1 || p.reference_weight+p.dp_tracking_weight<=0.0 ||
      p.max_initial_heading_error_rad>=1.0 || p.dp_slope_sample>p.max_lateral_slope ||
      p.speed_time_step_sec<.1 || p.speed_time_step_sec>1 || p.trajectory_output_step_sec<.01 ||
      p.trajectory_output_step_sec>.1 || p.speed_beam_width<1 || p.speed_beam_width>4096)
    throw std::invalid_argument("unbounded or inconsistent planner discretization");
  if (p.trajectory_horizon_m/p.output_step_m>600 || p.trajectory_horizon_m/p.dp_step_m>100)
    throw std::invalid_argument("unbounded integrated planner discretization");
  return p;
}
}  // namespace path_planner
