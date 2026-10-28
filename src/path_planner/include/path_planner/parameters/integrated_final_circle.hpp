#pragma once
#include <path_planner/cores/integrated_final_circle.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace integrated_final_circle_detail {
namespace path_planner {
inline PathPlannerMissionParameters loadFinalCircleMissionParameters(const node_pipeline::ParamReader& params) {
  PathPlannerMissionParameters value;
  if (params.has("reference_smoothing_enabled")) value.reference_smoothing_enabled=params.flag("reference_smoothing_enabled");
  if(params.has("refined_curve_smoothing_sigma_m"))value.refined_curve_smoothing_sigma_m=params.number("refined_curve_smoothing_sigma_m");
  if(!std::isfinite(value.refined_curve_smoothing_sigma_m) || value.refined_curve_smoothing_sigma_m<1 ||
     value.refined_curve_smoothing_sigma_m>3)throw std::invalid_argument("invalid refined curve smoothing sigma");
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

inline IntegratedFinalCircleParameters loadIntegratedFinalCircleParameters(const node_pipeline::ParamReader& reader) {
  return loadFinalCircleMissionParameters(reader);
}
}  // namespace path_planner
}  // namespace integrated_final_circle_detail

namespace path_planner {
using ::integrated_final_circle_detail::path_planner::loadIntegratedFinalCircleParameters;
}  // namespace path_planner
