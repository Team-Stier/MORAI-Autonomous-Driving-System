#pragma once
#include <path_planner/cores/tunnel_stop_avoid.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
inline TunnelStopAvoidParameters loadTunnelStopAvoidParameters(const node_pipeline::ParamReader& params) {
  TunnelStopAvoidParameters p;
  p.tunnel_region_enabled=params.flag("tunnel_region_enabled");
  p.tunnel_origin_x_m=params.number("tunnel_origin_x_m");
  if(!std::isfinite(p.tunnel_origin_x_m))throw std::invalid_argument("invalid tunnel_origin_x_m");
  p.tunnel_origin_y_m=params.number("tunnel_origin_y_m");
  if(!std::isfinite(p.tunnel_origin_y_m))throw std::invalid_argument("invalid tunnel_origin_y_m");
  p.tunnel_axis_yaw_rad=params.number("tunnel_axis_yaw_rad");
  if(!std::isfinite(p.tunnel_axis_yaw_rad))throw std::invalid_argument("invalid tunnel_axis_yaw_rad");
  p.tunnel_min_station_m=params.number("tunnel_min_station_m");
  if(!std::isfinite(p.tunnel_min_station_m))throw std::invalid_argument("invalid tunnel_min_station_m");
  p.tunnel_max_station_m=params.number("tunnel_max_station_m");
  if(!std::isfinite(p.tunnel_max_station_m))throw std::invalid_argument("invalid tunnel_max_station_m");
  p.tunnel_right_wall_m=params.number("tunnel_right_wall_m");
  if(!std::isfinite(p.tunnel_right_wall_m))throw std::invalid_argument("invalid tunnel_right_wall_m");
  p.tunnel_left_wall_m=params.number("tunnel_left_wall_m");
  if(!std::isfinite(p.tunnel_left_wall_m))throw std::invalid_argument("invalid tunnel_left_wall_m");
  if(p.tunnel_min_station_m>=p.tunnel_max_station_m || p.tunnel_right_wall_m>=p.tunnel_left_wall_m)
    throw std::invalid_argument("invalid tunnel region bounds");
  p.approach_speed_mps=params.number("approach_speed_mps");
  if(!std::isfinite(p.approach_speed_mps) || p.approach_speed_mps<=0)throw std::invalid_argument("invalid approach_speed_mps");
  p.avoidance_speed_mps=params.number("avoidance_speed_mps");
  if(!std::isfinite(p.avoidance_speed_mps) || p.avoidance_speed_mps<=0)throw std::invalid_argument("invalid avoidance_speed_mps");
  p.detection_distance_m=params.number("detection_distance_m");
  if(!std::isfinite(p.detection_distance_m) || p.detection_distance_m<=0)throw std::invalid_argument("invalid detection_distance_m");
  p.stop_gap_m=params.number("stop_gap_m");
  if(!std::isfinite(p.stop_gap_m) || p.stop_gap_m<=0)throw std::invalid_argument("invalid stop_gap_m");
  p.stopping_deceleration_mps2=params.number("stopping_deceleration_mps2");
  if(!std::isfinite(p.stopping_deceleration_mps2) || p.stopping_deceleration_mps2<=0)throw std::invalid_argument("invalid stopping_deceleration_mps2");
  p.stopping_response_sec=params.number("stopping_response_sec");
  if(!std::isfinite(p.stopping_response_sec) || p.stopping_response_sec<=0)throw std::invalid_argument("invalid stopping_response_sec");
  p.stopped_speed_mps=params.number("stopped_speed_mps");
  if(!std::isfinite(p.stopped_speed_mps) || p.stopped_speed_mps<=0)throw std::invalid_argument("invalid stopped_speed_mps");
  p.stopped_confirmation_sec=params.number("stopped_confirmation_sec");
  if(!std::isfinite(p.stopped_confirmation_sec) || p.stopped_confirmation_sec<=0)throw std::invalid_argument("invalid stopped_confirmation_sec");
  p.vehicle_front_m=params.number("vehicle_front_m");
  if(!std::isfinite(p.vehicle_front_m) || p.vehicle_front_m<=0)throw std::invalid_argument("invalid vehicle_front_m");
  p.vehicle_rear_m=params.number("vehicle_rear_m");
  if(!std::isfinite(p.vehicle_rear_m) || p.vehicle_rear_m<=0)throw std::invalid_argument("invalid vehicle_rear_m");
  p.vehicle_width_m=params.number("vehicle_width_m");
  if(!std::isfinite(p.vehicle_width_m) || p.vehicle_width_m<=0)throw std::invalid_argument("invalid vehicle_width_m");
  p.wheelbase_m=params.number("wheelbase_m");
  if(!std::isfinite(p.wheelbase_m) || p.wheelbase_m<=0)throw std::invalid_argument("invalid wheelbase_m");
  p.obstacle_margin_m=params.number("obstacle_margin_m");
  if(!std::isfinite(p.obstacle_margin_m) || p.obstacle_margin_m<=0)throw std::invalid_argument("invalid obstacle_margin_m");
  p.road_margin_m=params.number("road_margin_m");
  if(!std::isfinite(p.road_margin_m) || p.road_margin_m<=0)throw std::invalid_argument("invalid road_margin_m");
  p.minimum_object_extent_m=params.number("minimum_object_extent_m");
  if(!std::isfinite(p.minimum_object_extent_m) || p.minimum_object_extent_m<=0)throw std::invalid_argument("invalid minimum_object_extent_m");
  p.dynamic_minimum_extent_m=params.number("dynamic_minimum_extent_m");
  if(!std::isfinite(p.dynamic_minimum_extent_m) || p.dynamic_minimum_extent_m<=0)throw std::invalid_argument("invalid dynamic_minimum_extent_m");
  p.dynamic_lateral_speed_mps=params.number("dynamic_lateral_speed_mps");
  if(!std::isfinite(p.dynamic_lateral_speed_mps) || p.dynamic_lateral_speed_mps<=0)throw std::invalid_argument("invalid dynamic_lateral_speed_mps");
  p.dynamic_direction_ratio=params.number("dynamic_direction_ratio");
  if(!std::isfinite(p.dynamic_direction_ratio) || p.dynamic_direction_ratio<=0)throw std::invalid_argument("invalid dynamic_direction_ratio");
  p.dynamic_displacement_m=params.number("dynamic_displacement_m");
  if(!std::isfinite(p.dynamic_displacement_m) || p.dynamic_displacement_m<=0)throw std::invalid_argument("invalid dynamic_displacement_m");
  p.dynamic_span_sec=params.number("dynamic_span_sec");
  if(!std::isfinite(p.dynamic_span_sec) || p.dynamic_span_sec<=0)throw std::invalid_argument("invalid dynamic_span_sec");
  p.stationary_speed_mps=params.number("stationary_speed_mps");
  if(!std::isfinite(p.stationary_speed_mps) || p.stationary_speed_mps<=0)throw std::invalid_argument("invalid stationary_speed_mps");
  p.stationary_span_sec=params.number("stationary_span_sec");
  if(!std::isfinite(p.stationary_span_sec) || p.stationary_span_sec<=0)throw std::invalid_argument("invalid stationary_span_sec");
  p.history_sec=params.number("history_sec");
  if(!std::isfinite(p.history_sec) || p.history_sec<=0)throw std::invalid_argument("invalid history_sec");
  p.observation_freshness_sec=params.number("observation_freshness_sec");
  if(!std::isfinite(p.observation_freshness_sec) || p.observation_freshness_sec<=0)throw std::invalid_argument("invalid observation_freshness_sec");
  p.clear_margin_m=params.number("clear_margin_m");
  if(!std::isfinite(p.clear_margin_m) || p.clear_margin_m<=0)throw std::invalid_argument("invalid clear_margin_m");
  p.clear_confirmation_sec=params.number("clear_confirmation_sec");
  if(!std::isfinite(p.clear_confirmation_sec) || p.clear_confirmation_sec<=0)throw std::invalid_argument("invalid clear_confirmation_sec");
  p.return_length_m=params.number("return_length_m");
  if(!std::isfinite(p.return_length_m) || p.return_length_m<=0)throw std::invalid_argument("invalid return_length_m");
  p.return_clearance_m=params.number("return_clearance_m");
  if(!std::isfinite(p.return_clearance_m) || p.return_clearance_m<=0)throw std::invalid_argument("invalid return_clearance_m");
  p.steering_limit_rad=params.number("steering_limit_rad");
  if(!std::isfinite(p.steering_limit_rad) || p.steering_limit_rad<=0)throw std::invalid_argument("invalid steering_limit_rad");
  p.steering_rate_radps=params.number("steering_rate_radps");
  if(!std::isfinite(p.steering_rate_radps) || p.steering_rate_radps<=0)throw std::invalid_argument("invalid steering_rate_radps");
  p.lateral_acceleration_mps2=params.number("lateral_acceleration_mps2");
  if(!std::isfinite(p.lateral_acceleration_mps2) || p.lateral_acceleration_mps2<=0)throw std::invalid_argument("invalid lateral_acceleration_mps2");
  p.reference_smoothing_m=params.number("reference_smoothing_m");
  if(!std::isfinite(p.reference_smoothing_m) || p.reference_smoothing_m<=0)throw std::invalid_argument("invalid reference_smoothing_m");
  p.stationary_oscillation_ratio=params.number("stationary_oscillation_ratio");
  if(!std::isfinite(p.stationary_oscillation_ratio) || p.stationary_oscillation_ratio<=0)throw std::invalid_argument("invalid stationary_oscillation_ratio");
  p.stationary_longitudinal_span_m=params.number("stationary_longitudinal_span_m");
  if(!std::isfinite(p.stationary_longitudinal_span_m) || p.stationary_longitudinal_span_m<=0)throw std::invalid_argument("invalid stationary_longitudinal_span_m");
  p.stationary_lateral_span_m=params.number("stationary_lateral_span_m");
  if(!std::isfinite(p.stationary_lateral_span_m) || p.stationary_lateral_span_m<=0)throw std::invalid_argument("invalid stationary_lateral_span_m");
  p.cruise_speed_mps=params.number("cruise_speed_mps");
  if(!std::isfinite(p.cruise_speed_mps) || p.cruise_speed_mps<=0)throw std::invalid_argument("invalid cruise_speed_mps");
  p.reacquisition_speed_mps=params.number("reacquisition_speed_mps");
  if(!std::isfinite(p.reacquisition_speed_mps) || p.reacquisition_speed_mps<=0)throw std::invalid_argument("invalid reacquisition_speed_mps");
  p.unclassified_timeout_sec=params.number("unclassified_timeout_sec");
  if(!std::isfinite(p.unclassified_timeout_sec) || p.unclassified_timeout_sec<=0)throw std::invalid_argument("invalid unclassified_timeout_sec");
  return p;
}
}  // namespace path_planner
