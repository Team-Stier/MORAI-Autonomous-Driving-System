#pragma once
#include <velocity_planner/cores/integrated_stop_straight.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
namespace velocity_planner {
inline IntegratedStopStraightParameters loadIntegratedStopStraightParameters(const node_pipeline::ParamReader& reader) {
 IntegratedStopStraightParameters p;
 p.blind_detection_distance_m=reader.number("blind_detection_distance_m");
 p.blind_deceleration_mps2=reader.number("blind_deceleration_mps2");
 if(!std::isfinite(p.blind_detection_distance_m) || p.blind_detection_distance_m<0 || p.blind_detection_distance_m>50 ||
    !std::isfinite(p.blind_deceleration_mps2) || p.blind_deceleration_mps2<=0)
  throw std::invalid_argument("invalid straight stop blind-approach parameters");
 if(reader.has("blind_hold_line_ids"))
  for(const double id:reader.numbers("blind_hold_line_ids")){
   if(!std::isfinite(id) || id<=0 || id!=std::floor(id))throw std::invalid_argument("invalid blind_hold_line_ids");
   p.blind_hold_line_ids.push_back(static_cast<std::int64_t>(id));
  }
 if(reader.has("blind_hold_max_sec"))p.blind_hold_max_sec=reader.number("blind_hold_max_sec");
 if(!std::isfinite(p.blind_hold_max_sec) || p.blind_hold_max_sec<0 || p.blind_hold_max_sec>100)
  throw std::invalid_argument("invalid blind_hold_max_sec");
 if(reader.has("yellow_stop_deceleration_mps2"))p.yellow_stop_deceleration_mps2=reader.number("yellow_stop_deceleration_mps2");
 if(reader.has("front_axle_m"))p.front_axle_m=reader.number("front_axle_m");
 if(reader.has("yellow_stop_margin_m"))p.yellow_stop_margin_m=reader.number("yellow_stop_margin_m");
 if(!std::isfinite(p.yellow_stop_deceleration_mps2) || p.yellow_stop_deceleration_mps2<0 || p.yellow_stop_deceleration_mps2>10 ||
    !std::isfinite(p.front_axle_m) || p.front_axle_m<0 || p.front_axle_m>6 ||
    !std::isfinite(p.yellow_stop_margin_m) || p.yellow_stop_margin_m<0 || p.yellow_stop_margin_m>5)
  throw std::invalid_argument("invalid yellow front-axle stop parameters");
 return p;
}
}
