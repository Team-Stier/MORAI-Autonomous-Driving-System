#pragma once
#include <path_planner/cores/integrated_highway.hpp>
#include <node_pipeline/extension.hpp>

#include <cmath>
#include <stdexcept>
namespace path_planner {
inline HighwayMissionParameters loadHighwayMissionParameters(const node_pipeline::ParamReader& reader) {
  HighwayMissionParameters p;
  if (reader.has("reference_smoothing_enabled")) p.reference_smoothing_enabled=reader.flag("reference_smoothing_enabled");
  p.local_path_length_m=reader.number("local_path_length_m");
  if(!std::isfinite(p.local_path_length_m) || p.local_path_length_m<=0)throw std::invalid_argument("invalid highway local_path_length_m");
  p.sample_step_m=reader.number("sample_step_m");
  if(!std::isfinite(p.sample_step_m) || p.sample_step_m<=0)throw std::invalid_argument("invalid highway sample_step_m");
  p.cruise_speed_mps=reader.number("cruise_speed_mps");
  if(!std::isfinite(p.cruise_speed_mps) || p.cruise_speed_mps<=0)throw std::invalid_argument("invalid highway cruise_speed_mps");
  p.transition_length_m=reader.number("transition_length_m");
  if(!std::isfinite(p.transition_length_m) || p.transition_length_m<=0)throw std::invalid_argument("invalid highway transition_length_m");
  p.maximum_curvature_per_m=reader.number("maximum_curvature_per_m");
  if(!std::isfinite(p.maximum_curvature_per_m) || p.maximum_curvature_per_m<=0)throw std::invalid_argument("invalid highway maximum_curvature_per_m");
  p.maximum_lateral_acceleration_mps2=reader.number("maximum_lateral_acceleration_mps2");
  if(!std::isfinite(p.maximum_lateral_acceleration_mps2) || p.maximum_lateral_acceleration_mps2<=0)throw std::invalid_argument("invalid highway maximum_lateral_acceleration_mps2");
  p.maximum_lane_offset_m=reader.number("maximum_lane_offset_m");
  if(!std::isfinite(p.maximum_lane_offset_m) || p.maximum_lane_offset_m<=0)throw std::invalid_argument("invalid highway maximum_lane_offset_m");
  p.maximum_join_gap_m=reader.number("maximum_join_gap_m");
  if(!std::isfinite(p.maximum_join_gap_m) || p.maximum_join_gap_m<=0)throw std::invalid_argument("invalid highway maximum_join_gap_m");
  p.vehicle_front_m=reader.number("vehicle_front_m");
  if(!std::isfinite(p.vehicle_front_m) || p.vehicle_front_m<=0)throw std::invalid_argument("invalid highway vehicle_front_m");
  p.vehicle_rear_m=reader.number("vehicle_rear_m");
  if(!std::isfinite(p.vehicle_rear_m) || p.vehicle_rear_m<=0)throw std::invalid_argument("invalid highway vehicle_rear_m");
  p.vehicle_half_width_m=reader.number("vehicle_half_width_m");
  if(!std::isfinite(p.vehicle_half_width_m) || p.vehicle_half_width_m<=0)throw std::invalid_argument("invalid highway vehicle_half_width_m");
  p.front_gap_m=reader.number("front_gap_m");
  if(!std::isfinite(p.front_gap_m) || p.front_gap_m<=0)throw std::invalid_argument("invalid highway front_gap_m");
  p.rear_gap_m=reader.number("rear_gap_m");
  if(!std::isfinite(p.rear_gap_m) || p.rear_gap_m<=0)throw std::invalid_argument("invalid highway rear_gap_m");
  p.gap_confirmation_sec=reader.number("gap_confirmation_sec");
  if(!std::isfinite(p.gap_confirmation_sec) || p.gap_confirmation_sec<=0)throw std::invalid_argument("invalid highway gap_confirmation_sec");
  p.completion_lateral_m=reader.number("completion_lateral_m");
  if(!std::isfinite(p.completion_lateral_m) || p.completion_lateral_m<=0)throw std::invalid_argument("invalid highway completion_lateral_m");
  p.completion_heading_rad=reader.number("completion_heading_rad");
  if(!std::isfinite(p.completion_heading_rad) || p.completion_heading_rad<=0)throw std::invalid_argument("invalid highway completion_heading_rad");
  if(reader.has("following_gap_recovery_rate_mps"))p.following_gap_recovery_rate_mps=reader.number("following_gap_recovery_rate_mps");
  if(!std::isfinite(p.following_gap_recovery_rate_mps) || p.following_gap_recovery_rate_mps<=0 || p.following_gap_recovery_rate_mps>2)
    throw std::invalid_argument("invalid following gap recovery rate");
  p.following_gap_m=reader.number("following_gap_m");
  if(!std::isfinite(p.following_gap_m) || p.following_gap_m<=0)throw std::invalid_argument("invalid highway following_gap_m");
  p.following_time_sec=reader.number("following_time_sec");
  if(!std::isfinite(p.following_time_sec) || p.following_time_sec<=0)throw std::invalid_argument("invalid highway following_time_sec");
  p.stopping_deceleration_mps2=reader.number("stopping_deceleration_mps2");
  if(!std::isfinite(p.stopping_deceleration_mps2) || p.stopping_deceleration_mps2<=0)throw std::invalid_argument("invalid highway stopping_deceleration_mps2");
  p.stop_buffer_m=reader.number("stop_buffer_m");
  if(!std::isfinite(p.stop_buffer_m) || p.stop_buffer_m<=0)throw std::invalid_argument("invalid highway stop_buffer_m");
  p.acc_acceleration_mps2=reader.number("acc_acceleration_mps2");
  p.acc_deceleration_mps2=reader.number("acc_deceleration_mps2");
  if(!std::isfinite(p.acc_acceleration_mps2) || p.acc_acceleration_mps2<=0 ||
      !std::isfinite(p.acc_deceleration_mps2) || p.acc_deceleration_mps2<=0)throw std::invalid_argument("invalid ACC acceleration bounds");
  p.entry_speed_mps=reader.number("entry_speed_mps");
  if(!std::isfinite(p.entry_speed_mps) || p.entry_speed_mps<=0)throw std::invalid_argument("invalid highway entry_speed_mps");
  p.rear_headway_sec=reader.number("rear_headway_sec");
  if(!std::isfinite(p.rear_headway_sec) || p.rear_headway_sec<=0)throw std::invalid_argument("invalid highway rear_headway_sec");
  p.rear_clear_sec=reader.number("rear_clear_sec");
  if(!std::isfinite(p.rear_clear_sec) || p.rear_clear_sec<=0)throw std::invalid_argument("invalid highway rear_clear_sec");
  p.highway_end_y_m=reader.number("highway_end_y_m");
  if(!std::isfinite(p.highway_end_y_m))throw std::invalid_argument("invalid highway end");
  p.exit_speed_mps=reader.has("exit_speed_mps")?reader.number("exit_speed_mps"):p.entry_speed_mps;
  if(!std::isfinite(p.exit_speed_mps) || p.exit_speed_mps<=0 || p.exit_speed_mps>60.0/3.6)
    throw std::invalid_argument("invalid highway exit_speed_mps");
  p.mission_end_y_m=reader.number("mission_end_y_m");
  if(!std::isfinite(p.mission_end_y_m))throw std::invalid_argument("invalid highway mission end");
  const double count=reader.number("mission_change_count");
  if(!std::isfinite(count) || count<1 || count>4 || count!=std::floor(count))throw std::invalid_argument("invalid highway change count");
  p.mission_change_count=static_cast<unsigned>(count);
  return p;
}

inline IntegratedHighwayParameters loadIntegratedHighwayParameters(const node_pipeline::ParamReader& reader) {
  IntegratedHighwayParameters p;
  p.mission=loadHighwayMissionParameters(reader);
  p.publication_horizon_sec=reader.number("publication_horizon_sec");
  p.minimum_publication_length_m=reader.number("minimum_publication_length_m");
  if(!std::isfinite(p.publication_horizon_sec) || p.publication_horizon_sec<3.5 || p.publication_horizon_sec>5. ||
      !std::isfinite(p.minimum_publication_length_m) || p.minimum_publication_length_m<47.)
    throw std::invalid_argument("invalid highway publication extent");
  return p;
}
}
