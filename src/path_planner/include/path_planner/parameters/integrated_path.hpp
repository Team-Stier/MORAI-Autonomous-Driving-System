#pragma once
#include <path_planner/cores/integrated_path.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace integrated_path_detail {
namespace path_planner {
inline IntegratedPathTrajectoryParameters loadIntegratedPathTrajectoryParameters(const node_pipeline::ParamReader& r) {
  IntegratedPathTrajectoryParameters p;
  auto& s=p.solver;
  if(r.has("integrated_publish_lateral_reference"))p.publish_lateral_reference=r.flag("integrated_publish_lateral_reference");
  if(r.has("integrated_curve_preview_m"))p.curve_preview_m=r.number("integrated_curve_preview_m");
  if(r.has("integrated_curve_spacing_m"))p.curve_spacing_m=r.number("integrated_curve_spacing_m");
  if(r.has("integrated_curve_deceleration_mps2"))p.curve_deceleration_mps2=r.number("integrated_curve_deceleration_mps2");
  if(r.has("integrated_minimum_curve_speed_mps"))p.minimum_curve_speed_mps=r.number("integrated_minimum_curve_speed_mps");
  if(!std::isfinite(p.curve_preview_m)||p.curve_preview_m<=0||p.curve_preview_m>100||
     !std::isfinite(p.curve_spacing_m)||p.curve_spacing_m<.5||p.curve_spacing_m>p.curve_preview_m||
     !std::isfinite(p.curve_deceleration_mps2)||p.curve_deceleration_mps2<=0||
     !std::isfinite(p.minimum_curve_speed_mps)||p.minimum_curve_speed_mps<0)
    throw std::invalid_argument("invalid integrated reference curvature preview");
  if(r.has("integrated_reference_piece_m"))s.preferred_reference_piece_m=r.number("integrated_reference_piece_m");
  if(!std::isfinite(s.preferred_reference_piece_m) || s.preferred_reference_piece_m<0 || (s.preferred_reference_piece_m>0 && s.preferred_reference_piece_m<2) || s.preferred_reference_piece_m>25)
    throw std::invalid_argument("invalid integrated reference piece length");
  s.speed_beam_width=r.integer("integrated_speed_beam_width");
  if(r.has("integrated_interaction_speed_searches"))s.maximum_speed_searches=r.integer("integrated_interaction_speed_searches");
  if(s.maximum_speed_searches<1 || s.maximum_speed_searches>16)
    throw std::invalid_argument("invalid integrated interaction search budget");
  s.recovery_speed_beam_width=r.has("integrated_recovery_speed_beam_width") ?
      r.integer("integrated_recovery_speed_beam_width") : 64;
  if(s.recovery_speed_beam_width<s.speed_beam_width || s.recovery_speed_beam_width>128)
    throw std::invalid_argument("invalid integrated recovery search budget");
  s.qp_max_iterations=r.integer("integrated_qp_iterations");
  if(r.has("integrated_recovery_reentry_response_sec"))p.recovery_response_sec=r.number("integrated_recovery_reentry_response_sec");
  if(!std::isfinite(p.recovery_response_sec) || p.recovery_response_sec<0 || p.recovery_response_sec>.3)
    throw std::invalid_argument("invalid integrated reentry response");
  if(r.has("integrated_road_margin_m"))s.road_margin_m=r.number("integrated_road_margin_m");
  if(r.has("integrated_maximum_lateral_slope"))s.max_lateral_slope=r.number("integrated_maximum_lateral_slope");
  if(!std::isfinite(s.road_margin_m)||s.road_margin_m<.05||s.road_margin_m>.4||
     !std::isfinite(s.max_lateral_slope)||s.max_lateral_slope<.2||s.max_lateral_slope>.8)
    throw std::invalid_argument("invalid integrated geometry clearance");
  s.reference_weight=r.number("integrated_reference_weight");
  s.previous_weight=r.number("integrated_previous_weight");
  p.corridor_half_width_m=r.number("integrated_corridor_half_width_m");
  p.horizon_sec=r.number("integrated_horizon_sec");
  if(r.has("integrated_recovery_deceleration_mps2"))p.recovery_deceleration_mps2=r.number("integrated_recovery_deceleration_mps2");
  if(r.has("integrated_recovery_jerk_mps3"))p.recovery_jerk_mps3=r.number("integrated_recovery_jerk_mps3");
  if(!std::isfinite(p.recovery_deceleration_mps2) || p.recovery_deceleration_mps2<s.max_deceleration_mps2 || p.recovery_deceleration_mps2>8. ||
      !std::isfinite(p.recovery_jerk_mps3) || p.recovery_jerk_mps3<s.max_longitudinal_jerk_mps3 || p.recovery_jerk_mps3>8.)
    throw std::invalid_argument("invalid integrated recovery dynamics");
  if(r.has("integrated_dynamic_zone_speed_mps"))p.dynamic_zone_speed_mps=r.number("integrated_dynamic_zone_speed_mps");
  if(r.has("integrated_dynamic_zone_approach_m"))p.dynamic_zone_approach_m=r.number("integrated_dynamic_zone_approach_m");
  if(!std::isfinite(p.dynamic_zone_speed_mps) || p.dynamic_zone_speed_mps<=0 ||
      !std::isfinite(p.dynamic_zone_approach_m) || p.dynamic_zone_approach_m<0)
    throw std::invalid_argument("invalid integrated dynamic zone cap");
  s.max_speed_mps=r.number("integrated_maximum_speed_mps");
  s.max_lateral_accel_mps2=r.number("integrated_maximum_lateral_acceleration_mps2");
  if(r.has("integrated_recovery_lateral_acceleration_mps2"))p.recovery_lateral_acceleration_mps2=r.number("integrated_recovery_lateral_acceleration_mps2");
  if(!std::isfinite(p.recovery_lateral_acceleration_mps2) || p.recovery_lateral_acceleration_mps2<s.max_lateral_accel_mps2 || p.recovery_lateral_acceleration_mps2>6.)
    throw std::invalid_argument("invalid integrated recovery lateral bound");
  s.preferred_lateral_accel_mps2=r.has("integrated_preferred_lateral_acceleration_mps2") ?
      r.number("integrated_preferred_lateral_acceleration_mps2") : s.max_lateral_accel_mps2;
  if(!std::isfinite(s.preferred_lateral_accel_mps2) || s.preferred_lateral_accel_mps2<=0 ||
      s.preferred_lateral_accel_mps2>s.max_lateral_accel_mps2)
    throw std::invalid_argument("invalid integrated preferred lateral acceleration");
  s.following_min_gap_m=r.number("integrated_following_gap_m");
  s.following_time_headway_sec=r.number("integrated_following_headway_sec");
  s.following_low_speed_headway_sec=s.following_time_headway_sec;
  s.trajectory_horizon_m=150;s.normal_speed_limit_mps=0;
  s.horizon_m=25;s.dp_step_m=4;s.output_step_m=.5;s.qp_tolerance=1e-4;
  s.max_initial_distance_m=5;s.max_curvature_inv_m=.25;
  for(double value:{s.max_speed_mps,s.max_lateral_accel_mps2,p.corridor_half_width_m,p.horizon_sec})
    if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid integrated planner parameter");
  if(s.speed_beam_width<1 || s.speed_beam_width>128 || s.qp_max_iterations<1 || s.qp_max_iterations>3000 ||
      p.horizon_sec!=5.)throw std::invalid_argument("invalid integrated search budget or prediction interval");
  return p;
}
inline PathPlannerMissionParameters loadPathPlannerMissionParameters(const node_pipeline::ParamReader& params) {
  PathPlannerMissionParameters value;
  if (params.has("reference_smoothing_enabled")) value.reference_smoothing_enabled=params.flag("reference_smoothing_enabled");
  if(params.has("refined_curve_right_bias_m"))value.refined_curve_right_bias_m=params.number("refined_curve_right_bias_m");
  if(!std::isfinite(value.refined_curve_right_bias_m) || value.refined_curve_right_bias_m<0 || value.refined_curve_right_bias_m>.6)
    throw std::invalid_argument("invalid curve right bias");
  if(params.has("refined_curve_lanelet_ids"))for(double id:params.numbers("refined_curve_lanelet_ids")) {
    if(!std::isfinite(id) || id!=std::floor(id))throw std::invalid_argument("invalid refined curve lane ID");
    value.refined_curve_lanelet_ids.push_back(static_cast<lanelet::Id>(id));
  }
  if (params.has("dynamic_zone_enabled")) value.dynamic_zone_enabled=params.flag("dynamic_zone_enabled");
  if (params.has("dynamic_zone_min_x_m")) value.dynamic_zone_min_x_m=params.number("dynamic_zone_min_x_m");
  if (params.has("dynamic_zone_max_x_m")) value.dynamic_zone_max_x_m=params.number("dynamic_zone_max_x_m");
  if (params.has("dynamic_zone_min_y_m")) value.dynamic_zone_min_y_m=params.number("dynamic_zone_min_y_m");
  if (params.has("dynamic_zone_max_y_m")) value.dynamic_zone_max_y_m=params.number("dynamic_zone_max_y_m");
  if (params.has("dynamic_zone_minimum_detection_m")) value.dynamic_zone_minimum_detection_m=params.number("dynamic_zone_minimum_detection_m");
  if (params.has("dynamic_zone_reaction_sec")) value.dynamic_zone_reaction_sec=params.number("dynamic_zone_reaction_sec");
  if (params.has("dynamic_zone_braking_mps2")) value.dynamic_zone_braking_mps2=params.number("dynamic_zone_braking_mps2");
  if (params.has("dynamic_zone_clear_sec")) value.dynamic_zone_clear_sec=params.number("dynamic_zone_clear_sec");
  for (double number : {value.dynamic_zone_min_x_m,value.dynamic_zone_max_x_m,
      value.dynamic_zone_min_y_m,value.dynamic_zone_max_y_m,value.dynamic_zone_minimum_detection_m,
      value.dynamic_zone_reaction_sec,value.dynamic_zone_braking_mps2,value.dynamic_zone_clear_sec})
    if (!std::isfinite(number)) throw std::invalid_argument("nonfinite dynamic zone parameter");
  if (value.dynamic_zone_min_x_m>=value.dynamic_zone_max_x_m || value.dynamic_zone_min_y_m>=value.dynamic_zone_max_y_m ||
      value.dynamic_zone_minimum_detection_m<=0 || value.dynamic_zone_reaction_sec<0 ||
      value.dynamic_zone_braking_mps2<=0 || value.dynamic_zone_clear_sec<0)
    throw std::invalid_argument("invalid dynamic zone parameter");

  if (params.has("use_boundary_midpoint_centerline"))
    value.use_boundary_midpoint_centerline=params.flag("use_boundary_midpoint_centerline");
  if (params.has("centerline_step_m")) value.centerline_step_m=params.number("centerline_step_m");
  if (!std::isfinite(value.centerline_step_m) || value.centerline_step_m<0.05 || value.centerline_step_m>1)
    throw std::invalid_argument("centerline_step_m must be between 0.05 and 1 m");
  value.local_path_length_m = params.number("local_path_length_m");
  if (!std::isfinite(value.local_path_length_m) || value.local_path_length_m <= 0) throw std::invalid_argument("invalid local_path_length_m");
  value.sample_step_m = params.number("sample_step_m");
  if (!std::isfinite(value.sample_step_m) || value.sample_step_m <= 0) throw std::invalid_argument("invalid sample_step_m");
  value.wheelbase_m=params.number("wheelbase_m");
  if(!std::isfinite(value.wheelbase_m) || value.wheelbase_m<=0) throw std::invalid_argument("invalid wheelbase_m");
  value.vehicle_front_m = params.number("vehicle_front_m");
  if (!std::isfinite(value.vehicle_front_m) || value.vehicle_front_m <= 0) throw std::invalid_argument("invalid vehicle_front_m");
  value.vehicle_rear_m = params.number("vehicle_rear_m");
  if (!std::isfinite(value.vehicle_rear_m) || value.vehicle_rear_m <= 0) throw std::invalid_argument("invalid vehicle_rear_m");
  value.vehicle_half_width_m = params.number("vehicle_half_width_m");
  if (!std::isfinite(value.vehicle_half_width_m) || value.vehicle_half_width_m <= 0) throw std::invalid_argument("invalid vehicle_half_width_m");
  value.collision_margin_m = params.number("collision_margin_m");
  if (!std::isfinite(value.collision_margin_m) || value.collision_margin_m <= 0) throw std::invalid_argument("invalid collision_margin_m");
  value.static_speed_mps = params.number("static_speed_mps");
  if (!std::isfinite(value.static_speed_mps) || value.static_speed_mps <= 0) throw std::invalid_argument("invalid static_speed_mps");
  value.dynamic_speed_mps = params.number("dynamic_speed_mps");
  if (!std::isfinite(value.dynamic_speed_mps) || value.dynamic_speed_mps <= 0) throw std::invalid_argument("invalid dynamic_speed_mps");
  value.dynamic_confirmation_sec = params.number("dynamic_confirmation_sec");
  value.static_confirmation_sec = params.number("static_confirmation_sec");
  if (!std::isfinite(value.dynamic_confirmation_sec) || value.dynamic_confirmation_sec <= 0 ||
      !std::isfinite(value.static_confirmation_sec) || value.static_confirmation_sec <= 0)
    throw std::invalid_argument("invalid object motion confirmation duration");
  value.track_expiry_sec = params.number("track_expiry_sec");
  if (!std::isfinite(value.track_expiry_sec) || value.track_expiry_sec <= 0) throw std::invalid_argument("invalid track_expiry_sec");
  value.dynamic_prediction_sec = params.number("dynamic_prediction_sec");
  if (!std::isfinite(value.dynamic_prediction_sec) || value.dynamic_prediction_sec <= 0) throw std::invalid_argument("invalid dynamic_prediction_sec");
  value.avoidance_speed_mps = params.number("avoidance_speed_mps");
  if (!std::isfinite(value.avoidance_speed_mps) || value.avoidance_speed_mps <= 0) throw std::invalid_argument("invalid avoidance_speed_mps");
  value.maximum_lateral_acceleration_mps2=params.number("maximum_lateral_acceleration_mps2");
  if(!std::isfinite(value.maximum_lateral_acceleration_mps2) || value.maximum_lateral_acceleration_mps2<=0)
    throw std::invalid_argument("invalid maximum_lateral_acceleration_mps2");
  value.avoidance_lead_m=params.number("avoidance_lead_m");
  if(!std::isfinite(value.avoidance_lead_m) || value.avoidance_lead_m<0 || value.avoidance_lead_m>5)
    throw std::invalid_argument("avoidance_lead_m must be between 0 and 5 m");
  value.transition_length_m = params.number("transition_length_m");
  if (!std::isfinite(value.transition_length_m) || value.transition_length_m <= 0) throw std::invalid_argument("invalid transition_length_m");
  value.return_transition_length_m=params.number("return_transition_length_m");
  if(!std::isfinite(value.return_transition_length_m) || value.return_transition_length_m<=0 ||
      value.return_transition_length_m>value.local_path_length_m)
    throw std::invalid_argument("invalid return_transition_length_m");
  value.maximum_curvature_per_m=params.number("maximum_curvature_per_m");
  value.maximum_lane_offset_m=params.number("maximum_lane_offset_m");
  value.maximum_join_gap_m=params.number("maximum_join_gap_m");
  value.completion_lateral_m=params.number("completion_lateral_m");
  value.completion_heading_rad=params.number("completion_heading_rad");
  for(double number:{value.maximum_curvature_per_m,value.maximum_lane_offset_m,
                     value.maximum_join_gap_m,value.completion_lateral_m,value.completion_heading_rad})
    if(!std::isfinite(number) || number<=0) throw std::invalid_argument("invalid avoidance geometry limit");
  if(value.maximum_curvature_per_m>std::tan(0.6981317007977318)/value.wheelbase_m || value.completion_heading_rad>0.5)
    throw std::invalid_argument("avoidance geometry exceeds vehicle limits");
  value.stop_buffer_m = params.number("stop_buffer_m");
  if (!std::isfinite(value.stop_buffer_m) || value.stop_buffer_m <= 0) throw std::invalid_argument("invalid stop_buffer_m");
  value.stopping_deceleration_mps2 = params.number("stopping_deceleration_mps2");
  if (!std::isfinite(value.stopping_deceleration_mps2) || value.stopping_deceleration_mps2 <= 0) throw std::invalid_argument("invalid stopping_deceleration_mps2");
  if (value.static_speed_mps >= value.dynamic_speed_mps || value.sample_step_m > 1 ||
      value.local_path_length_m > 300 || value.dynamic_prediction_sec > 5 ||
      value.transition_length_m > value.local_path_length_m)
    throw std::invalid_argument("invalid  Lanelet planner limits");
  return value;
}

inline IntegratedPathParameters loadIntegratedPathParameters(const node_pipeline::ParamReader& reader) {
  IntegratedPathParameters p;
  p.mission=loadPathPlannerMissionParameters(reader);
  p.trajectory=loadIntegratedPathTrajectoryParameters(reader);
  return p;
}
}  // namespace path_planner
}  // namespace integrated_path_detail

namespace path_planner {
using ::integrated_path_detail::path_planner::loadIntegratedPathParameters;
}  // namespace path_planner
