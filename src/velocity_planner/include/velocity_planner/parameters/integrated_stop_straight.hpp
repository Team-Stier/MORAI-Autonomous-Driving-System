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
 return p;
}
}
