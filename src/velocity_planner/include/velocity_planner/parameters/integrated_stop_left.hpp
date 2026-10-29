#pragma once
#include <velocity_planner/cores/integrated_stop_left.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
namespace velocity_planner {
inline IntegratedStopLeftParameters loadIntegratedStopLeftParameters(const node_pipeline::ParamReader& reader) {
 IntegratedStopLeftParameters p;p.extra_stop_offset_m=reader.number("extra_stop_offset_m");
 if(!std::isfinite(p.extra_stop_offset_m) || p.extra_stop_offset_m<0 || p.extra_stop_offset_m>3)
  throw std::invalid_argument("invalid left stop offset");
 return p;
}
}
