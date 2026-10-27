#pragma once
#include <path_planner/cores/final_circle_global_path.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
namespace path_planner {
inline FinalCircleGlobalPathParameters loadFinalCircleGlobalPathParameters(const node_pipeline::ParamReader& reader) {
  FinalCircleGlobalPathParameters p;p.forward_length_m=reader.number("forward_length_m");
  if(!std::isfinite(p.forward_length_m) || p.forward_length_m<50 || p.forward_length_m>300)
    throw std::invalid_argument("invalid forward route length");
  return p;
}
}
