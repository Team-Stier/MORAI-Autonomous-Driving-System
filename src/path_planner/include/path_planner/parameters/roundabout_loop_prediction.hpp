#pragma once
#include <path_planner/cores/roundabout_loop_prediction.hpp>
#include <node_pipeline/extension.hpp>
#include <pugixml.hpp>
#include <cmath>
#include <set>
#include <stdexcept>

namespace path_planner {
inline RoundaboutLoopPredictionParameters loadRoundaboutLoopPredictionParameters(const node_pipeline::ParamReader& params) {
  RoundaboutLoopPredictionParameters p;
  p.history_sec=params.number("history_sec");
  p.minimum_history_sec=params.number("minimum_history_sec");
  p.minimum_partial_history_sec=params.number("minimum_partial_history_sec");
  p.pair_baseline_sec=params.number("pair_baseline_sec");
  p.maximum_observation_gap_sec=params.number("maximum_observation_gap_sec");
  p.track_expiry_sec=params.number("track_expiry_sec");
  p.ring_match_distance_m=params.number("ring_match_distance_m");
  p.tangent_window_m=params.number("tangent_window_m");
  p.maximum_speed_mps=params.number("maximum_speed_mps");
  p.maximum_lateral_speed_mps=params.number("maximum_lateral_speed_mps");
  p.maximum_longitudinal_acceleration_mps2=params.number("maximum_longitudinal_acceleration_mps2");
  p.acceleration_horizon_sec=params.number("acceleration_horizon_sec");
  for (double v:{p.history_sec,p.minimum_history_sec,p.minimum_partial_history_sec,p.pair_baseline_sec,p.maximum_observation_gap_sec,
                 p.track_expiry_sec,p.ring_match_distance_m,p.tangent_window_m,p.maximum_speed_mps,p.maximum_lateral_speed_mps,
                 p.maximum_longitudinal_acceleration_mps2,p.acceleration_horizon_sec})
    if (!std::isfinite(v) || v<=0) throw std::invalid_argument("loop prediction parameters must be finite and positive");
  if (p.history_sec>3 || p.pair_baseline_sec>p.minimum_history_sec || p.minimum_history_sec>p.history_sec ||
      p.minimum_partial_history_sec<p.minimum_history_sec || p.minimum_partial_history_sec>p.history_sec ||
      p.pair_baseline_sec<.05 || p.maximum_observation_gap_sec>1 || p.track_expiry_sec<p.maximum_observation_gap_sec ||
      p.track_expiry_sec>3 || p.ring_match_distance_m>4 || p.tangent_window_m<.5 || p.tangent_window_m>4 ||
      p.maximum_speed_mps>40 || p.maximum_lateral_speed_mps>4 ||
      p.maximum_longitudinal_acceleration_mps2>4 || p.acceleration_horizon_sec>2)
    throw std::invalid_argument("invalid loop prediction timing or geometry");
  pugi::xml_document xml;const auto asset=params.asset("map_file");
  if (!xml.load_buffer(asset.data(),asset.size())) throw std::invalid_argument("invalid loop map XML");
  std::set<lanelet::Id> relations,selected;
  for (const auto& r:xml.child("osm").children("relation")) relations.insert(r.attribute("id").as_llong());
  for (double id:params.numbers("ring_lanelet_ids")) {
    if (!std::isfinite(id) || id<=0 || id>=static_cast<double>(std::numeric_limits<lanelet::Id>::max()) ||
        id!=std::floor(id) || relations.count(static_cast<lanelet::Id>(id))==0 ||
        !selected.insert(static_cast<lanelet::Id>(id)).second)
      throw std::invalid_argument("invalid circulating lanelet ID");
    p.ring_lanelet_ids.push_back(static_cast<lanelet::Id>(id));
  }
  if (p.ring_lanelet_ids.size()<2) throw std::invalid_argument("loop prediction needs an ordered closed lanelet chain");
  return p;
}
}  // namespace path_planner
