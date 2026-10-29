#pragma once
#include <path_planner/cores/roundabout_rolling_entry.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <pugixml.hpp>
#include <set>
#include <stdexcept>
#include <string>

namespace path_planner {
inline RoundaboutRollingEntryParameters loadRoundaboutRollingEntryParameters(const node_pipeline::ParamReader& params) {
  RoundaboutRollingEntryParameters p;
  p.control_step_m=params.number("control_step_m");p.integration_step_m=params.number("integration_step_m");
  p.sample_step_m=params.number("sample_step_m");p.local_path_length_m=params.number("local_path_length_m");
  p.reference_before_entry_m=params.number("reference_before_entry_m");
  p.reference_after_entry_m=params.number("reference_after_entry_m");
  if(!std::isfinite(p.control_step_m) || p.control_step_m<.5 || p.control_step_m>5 ||
     !std::isfinite(p.integration_step_m) || p.integration_step_m<.01 || p.integration_step_m>.1 ||
     !std::isfinite(p.sample_step_m) || p.sample_step_m<.1 || p.sample_step_m>1 ||
     !std::isfinite(p.local_path_length_m) || p.local_path_length_m<1 || p.local_path_length_m>100 ||
     !std::isfinite(p.reference_before_entry_m) || p.reference_before_entry_m<10 || p.reference_before_entry_m>150 ||
     !std::isfinite(p.reference_after_entry_m) || p.reference_after_entry_m<40 || p.reference_after_entry_m>200)
    throw std::invalid_argument("invalid lanelet reference geometry parameters");
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
      throw std::invalid_argument("roundabout_rolling_entry parameters must be finite and positive");
  for (double value : {p.collision_margin_m, p.prediction_uncertainty_mps})
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("roundabout_rolling_entry margins must be finite and nonnegative");
  if (p.time_step_sec < 0.05 || p.time_step_sec > 0.5 || p.command_step_sec < 0.01 ||
      p.command_step_sec > 1.0)
    throw std::invalid_argument("roundabout_rolling_entry time steps are unbounded or inconsistent");

  p.planning_horizon_sec = params.number("planning_horizon_sec");
  if (!std::isfinite(p.planning_horizon_sec) || p.planning_horizon_sec < 3.0 || p.planning_horizon_sec > 5.0)
    throw std::invalid_argument("planning_horizon_sec must be between 3 and 5");

  p.following_time_gap_sec = params.number("following_time_gap_sec");
  if (!std::isfinite(p.following_time_gap_sec) || p.following_time_gap_sec < 0.5 ||
      p.following_time_gap_sec > 3.0)
    throw std::invalid_argument("following_time_gap_sec must be between 0.5 and 3.0");

  p.rolling_response_sec = params.number("rolling_response_sec");
  if (!std::isfinite(p.rolling_response_sec) || p.rolling_response_sec < 0.0 || p.rolling_response_sec > 1.5)
    throw std::invalid_argument("rolling_response_sec must be between 0 and 1.5");
  p.slow_launch_response_sec = params.number("slow_launch_response_sec");
  p.slow_launch_acceleration_mps2 = params.number("slow_launch_acceleration_mps2");
  p.slow_launch_jerk_mps3 = params.number("slow_launch_jerk_mps3");
  p.fast_launch_response_sec = params.number("fast_launch_response_sec");
  p.fast_launch_acceleration_mps2 = params.number("fast_launch_acceleration_mps2");
  p.fast_launch_jerk_mps3 = params.number("fast_launch_jerk_mps3");
  p.fast_launch_command_step_sec = params.number("fast_launch_command_step_sec");
  for (double value : {p.slow_launch_response_sec, p.slow_launch_acceleration_mps2, p.slow_launch_jerk_mps3,
                       p.fast_launch_response_sec, p.fast_launch_acceleration_mps2, p.fast_launch_jerk_mps3,
                       p.fast_launch_command_step_sec})
    if (!std::isfinite(value)) throw std::invalid_argument("launch parameters must be finite");
  if (p.fast_launch_response_sec < 0.0 || p.slow_launch_response_sec < p.fast_launch_response_sec ||
      p.slow_launch_response_sec > 1.5 || p.slow_launch_acceleration_mps2 < 0.5 ||
      p.fast_launch_acceleration_mps2 < p.slow_launch_acceleration_mps2 ||
      p.fast_launch_acceleration_mps2 < p.max_acceleration_mps2 || p.fast_launch_acceleration_mps2 > 8.0 ||
      p.slow_launch_jerk_mps3 < 0.5 || p.fast_launch_jerk_mps3 < p.slow_launch_jerk_mps3 ||
      p.fast_launch_jerk_mps3 < p.max_jerk_mps3 || p.fast_launch_jerk_mps3 > 30.0 ||
      p.fast_launch_command_step_sec < 0.05 || p.fast_launch_command_step_sec > p.command_step_sec)
    throw std::invalid_argument("launch models must order slow before fast within limits");
  p.patience_sec = params.number("patience_sec");
  p.relaxed_launch_response_sec = params.number("relaxed_launch_response_sec");
  if (!std::isfinite(p.patience_sec) || p.patience_sec < 0.0 || p.patience_sec > 60.0 ||
      !std::isfinite(p.relaxed_launch_response_sec) || p.relaxed_launch_response_sec < 0.0 ||
      p.relaxed_launch_response_sec > 1.5)
    throw std::invalid_argument("patience_sec must be 0..60 and relaxed_launch_response_sec 0..1.5");
  p.rear_time_gap_sec = params.number("rear_time_gap_sec");
  if (!std::isfinite(p.rear_time_gap_sec) || p.rear_time_gap_sec < 0.0 || p.rear_time_gap_sec > 3.0)
    throw std::invalid_argument("rear_time_gap_sec must be between 0 and 3");
  p.commit_past_line_m = params.number("commit_past_line_m");
  if (!std::isfinite(p.commit_past_line_m) || p.commit_past_line_m < 0.0 || p.commit_past_line_m > 3.0)
    throw std::invalid_argument("commit_past_line_m must be between 0 and 3");
  p.rolling_accept_distance_m = params.number("rolling_accept_distance_m");
  if (!std::isfinite(p.rolling_accept_distance_m) || p.rolling_accept_distance_m < 0.0 ||
      p.rolling_accept_distance_m > p.reference_before_entry_m)
    throw std::invalid_argument("rolling_accept_distance_m must be between 0 and reference_before_entry_m");

  p.approach_speed_mps = params.number("approach_speed_mps");
  p.curve_speed_ratio = params.number("curve_speed_ratio");
  p.entry_stop_margin_m = params.number("entry_stop_margin_m");
  p.entry_decision_distance_m = params.number("entry_decision_distance_m");
  p.entry_check_length_m = params.number("entry_check_length_m");
  p.entry_gap_confirmation_sec = params.number("entry_gap_confirmation_sec");
  for (double value : {p.approach_speed_mps, p.curve_speed_ratio, p.entry_stop_margin_m,
                       p.entry_decision_distance_m, p.entry_check_length_m, p.entry_gap_confirmation_sec})
    if (!std::isfinite(value)) throw std::invalid_argument("entry parameters must be finite");
  if (p.approach_speed_mps < 0.5 || p.approach_speed_mps > p.desired_speed_mps ||
      p.curve_speed_ratio < 0.5 || p.curve_speed_ratio > 1.0 ||
      p.entry_stop_margin_m < 0.3 || p.entry_stop_margin_m > 3.0 ||
      p.entry_decision_distance_m < 1.0 || p.entry_decision_distance_m > 5.0 ||
      p.entry_check_length_m < p.vehicle_front_m + p.entry_stop_margin_m ||
      p.entry_check_length_m > 20.0 || p.entry_check_length_m > p.local_path_length_m ||
      p.entry_gap_confirmation_sec < 0.1 || p.entry_gap_confirmation_sec > 1.0)
    throw std::invalid_argument("entry parameter range is invalid");

  pugi::xml_document xml;const auto asset=params.asset("map_file");
  if(!xml.load_buffer(asset.data(),asset.size()))throw std::invalid_argument("invalid roundabout lanelet map");
  std::set<lanelet::Id> lanes,ways;
  for(const auto& q:xml.child("osm").children("relation"))lanes.insert(q.attribute("id").as_llong());
  for(const auto& q:xml.child("osm").children("way"))ways.insert(q.attribute("id").as_llong());
  for(double id:params.numbers("reference_lanelet_ids")) {
    if(!std::isfinite(id) || id<=0 || id!=std::floor(id) || !lanes.count(static_cast<lanelet::Id>(id)))
      throw std::invalid_argument("unknown roundabout reference lanelet");
    p.reference_lanelet_ids.push_back(static_cast<lanelet::Id>(id));
  }
  for(double id:params.numbers("conflict_lanelet_ids")) {
    if(!std::isfinite(id) || id<=0 || id!=std::floor(id) || !lanes.count(static_cast<lanelet::Id>(id)))
      throw std::invalid_argument("unknown roundabout conflict lanelet");
    p.conflict_lanelet_ids.push_back(static_cast<lanelet::Id>(id));
  }
  p.conflict_downstream_lanelet_id=params.integer("conflict_downstream_lanelet_id");
  if(p.conflict_lanelet_ids.empty() || !lanes.count(p.conflict_downstream_lanelet_id))
    throw std::invalid_argument("invalid roundabout conflict zone lanelets");
  p.conflict_downstream_m=params.number("conflict_downstream_m");
  p.conflict_zone_half_width_m=params.number("conflict_zone_half_width_m");
  p.conflict_margin_m=params.number("conflict_margin_m");
  p.conflict_speed_floor_mps=params.number("conflict_speed_floor_mps");
  p.conflict_clear_sec=params.number("conflict_clear_sec");
  p.stationary_confirmation_sec=params.number("stationary_confirmation_sec");
  p.stationary_displacement_m=params.number("stationary_displacement_m");
  if(!std::isfinite(p.conflict_downstream_m) || p.conflict_downstream_m<0 || p.conflict_downstream_m>30 ||
     !std::isfinite(p.conflict_zone_half_width_m) || p.conflict_zone_half_width_m<.5 || p.conflict_zone_half_width_m>5 ||
     !std::isfinite(p.conflict_margin_m) || p.conflict_margin_m<0 || p.conflict_margin_m>10 ||
     !std::isfinite(p.conflict_speed_floor_mps) || p.conflict_speed_floor_mps<.5 || p.conflict_speed_floor_mps>15 ||
     !std::isfinite(p.conflict_clear_sec) || p.conflict_clear_sec<0 || p.conflict_clear_sec>10 ||
     !std::isfinite(p.stationary_confirmation_sec) || p.stationary_confirmation_sec<1 || p.stationary_confirmation_sec>10 ||
     !std::isfinite(p.stationary_displacement_m) || p.stationary_displacement_m<.1 || p.stationary_displacement_m>3)
    throw std::invalid_argument("roundabout conflict zone parameter range is invalid");
  p.stop_line_id=params.integer("stop_line_id");p.departure_lanelet_id=params.integer("departure_lanelet_id");
  if(p.reference_lanelet_ids.size()<2 || !ways.count(p.stop_line_id) || !lanes.count(p.departure_lanelet_id))
    throw std::invalid_argument("invalid roundabout reference anchors");
  std::set<lanelet::Id> endpoints;
  for(const auto& way:xml.child("osm").children("way"))if(way.attribute("id").as_llong()==p.stop_line_id) {
    const auto first=way.child("nd");auto last=first;
    for(const auto& node:way.children("nd"))last=node;
    endpoints.insert(first.attribute("ref").as_llong());endpoints.insert(last.attribute("ref").as_llong());
  }
  for(const auto& node:xml.child("osm").children("node"))if(endpoints.count(node.attribute("id").as_llong())) {
    geometry_msgs::Point point;
    point.x=node.find_child_by_attribute("tag","k","local_x").attribute("v").as_double(std::numeric_limits<double>::quiet_NaN());
    point.y=node.find_child_by_attribute("tag","k","local_y").attribute("v").as_double(std::numeric_limits<double>::quiet_NaN());
    point.z=node.find_child_by_attribute("tag","k","local_z").attribute("v").as_double();
    if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))throw std::invalid_argument("invalid stop line coordinates");
    p.stop_line.push_back(point);
  }
  if(p.stop_line.size()!=2)throw std::invalid_argument("roundabout stop line needs two map endpoints");
  return p;
}
}  // namespace path_planner
