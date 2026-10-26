#pragma once
#include <traffic_light/cores/camera7_lamp_cpu.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace traffic_light {
inline Camera7LampCpuParameters loadCamera7LampCpuParameters(
    const node_pipeline::ParamReader& params) {
  Camera7LampCpuParameters value;
  if (params.has("search_bottom_ratio"))
    value.search_bottom_ratio = params.number("search_bottom_ratio");
  if (params.has("min_area")) value.min_area = static_cast<int>(params.integer("min_area"));
  if (params.has("max_area")) value.max_area = static_cast<int>(params.integer("max_area"));
  if (params.has("max_span")) value.max_span = static_cast<int>(params.integer("max_span"));
  if (params.has("max_aspect")) value.max_aspect = params.number("max_aspect");
  if (params.has("link_distance_px")) value.link_distance_px = params.number("link_distance_px");
  if (params.has("center_sigma_px")) value.center_sigma_px = params.number("center_sigma_px");
  if (params.has("ambiguity_ratio")) value.ambiguity_ratio = params.number("ambiguity_ratio");
  if (params.has("color_margin")) value.color_margin = static_cast<int>(params.integer("color_margin"));
  if (params.has("minimum_repeat"))
    value.minimum_repeat = static_cast<int>(params.integer("minimum_repeat"));

  if (!std::isfinite(value.search_bottom_ratio) || value.search_bottom_ratio <= 0 ||
      value.search_bottom_ratio > 1)
    throw std::invalid_argument("search_bottom_ratio must be in (0,1]");
  if (value.min_area < 1 || value.max_area < value.min_area || value.max_area > 100000)
    throw std::invalid_argument("min_area/max_area are invalid");
  if (value.max_span < 1 || value.max_span > 1000)
    throw std::invalid_argument("max_span must be in [1,1000]");
  if (!std::isfinite(value.max_aspect) || value.max_aspect < 1 || value.max_aspect > 20)
    throw std::invalid_argument("max_aspect must be in [1,20]");
  if (!std::isfinite(value.link_distance_px) || value.link_distance_px <= 0)
    throw std::invalid_argument("link_distance_px must be positive");
  if (!std::isfinite(value.center_sigma_px) || value.center_sigma_px <= 0)
    throw std::invalid_argument("center_sigma_px must be positive");
  if (!std::isfinite(value.ambiguity_ratio) || value.ambiguity_ratio <= 0 ||
      value.ambiguity_ratio > 1)
    throw std::invalid_argument("ambiguity_ratio must be in (0,1]");
  if (value.color_margin < 0 || value.color_margin > 80)
    throw std::invalid_argument("color_margin must be in [0,80]");
  if (value.minimum_repeat < 1 || value.minimum_repeat > 8)
    throw std::invalid_argument("minimum_repeat must be in [1,8]");
  return value;
}
}  // namespace traffic_light
