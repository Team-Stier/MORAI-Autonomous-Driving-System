#pragma once
#include <object_detector/cores/guardrail_obb.hpp>
#include <node_pipeline/extension.hpp>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <pugixml.hpp>
#include <cmath>
#include <stdexcept>

namespace object_detector {
inline GuardrailObbParameters loadGuardrailObbParameters(const node_pipeline::ParamReader& params) {
  GuardrailObbParameters config;
  config.maximum_width_m = params.number("maximum_width_m");
  if (!std::isfinite(config.maximum_width_m) || config.maximum_width_m <= 0)
    throw std::invalid_argument("invalid guardrail_obb maximum_width_m");
  config.maximum_height_m = params.number("maximum_height_m");
  if (!std::isfinite(config.maximum_height_m) || config.maximum_height_m <= 0)
    throw std::invalid_argument("invalid guardrail_obb maximum_height_m");
  config.minimum_fragment_length_m = params.number("minimum_fragment_length_m");
  if (!std::isfinite(config.minimum_fragment_length_m) || config.minimum_fragment_length_m <= 0)
    throw std::invalid_argument("invalid guardrail_obb minimum_fragment_length_m");
  config.minimum_rail_length_m = params.number("minimum_rail_length_m");
  if (!std::isfinite(config.minimum_rail_length_m) || config.minimum_rail_length_m <= 0)
    throw std::invalid_argument("invalid guardrail_obb minimum_rail_length_m");
  config.maximum_boundary_distance_m = params.number("maximum_boundary_distance_m");
  if (!std::isfinite(config.maximum_boundary_distance_m) || config.maximum_boundary_distance_m <= 0)
    throw std::invalid_argument("invalid guardrail_obb maximum_boundary_distance_m");
  config.maximum_heading_error_rad = params.number("maximum_heading_error_rad");
  if (!std::isfinite(config.maximum_heading_error_rad) || config.maximum_heading_error_rad <= 0)
    throw std::invalid_argument("invalid guardrail_obb maximum_heading_error_rad");
  config.maximum_gap_m = params.number("maximum_gap_m");
  if (!std::isfinite(config.maximum_gap_m) || config.maximum_gap_m <= 0)
    throw std::invalid_argument("invalid guardrail_obb maximum_gap_m");
  config.maximum_offset_difference_m = params.number("maximum_offset_difference_m");
  if (!std::isfinite(config.maximum_offset_difference_m) || config.maximum_offset_difference_m <= 0)
    throw std::invalid_argument("invalid guardrail_obb maximum_offset_difference_m");
  config.memory_sec = params.number("memory_sec");
  if (!std::isfinite(config.memory_sec) || config.memory_sec <= 0)
    throw std::invalid_argument("invalid guardrail_obb memory_sec");
  config.maximum_fragment_length_m = params.number("maximum_fragment_length_m");
  config.evidence_sec = params.number("evidence_sec");
  if (!std::isfinite(config.maximum_fragment_length_m) ||
      config.maximum_fragment_length_m <= config.minimum_fragment_length_m ||
      config.maximum_fragment_length_m >= config.minimum_rail_length_m ||
      !std::isfinite(config.evidence_sec) || config.evidence_sec <= 0 || config.evidence_sec > config.memory_sec)
    throw std::invalid_argument("invalid guardrail_obb fragment evidence limits");
  if (config.maximum_heading_error_rad >= 1.5707963267948966 ||
      config.minimum_rail_length_m <= config.minimum_fragment_length_m)
    throw std::invalid_argument("invalid guardrail_obb geometry limits");
  const auto map_data = params.asset("map_file");
  pugi::xml_document document;
  if (!document.load_buffer(map_data.data(), map_data.size()))
    throw std::invalid_argument("invalid guardrail_obb Lanelet2 OSM XML");
  lanelet::osm::Errors errors;
  const auto osm = lanelet::osm::read(document, &errors);
  const lanelet::DefaultProjector projector(lanelet::Origin({0.0, 0.0}));
  const lanelet::io_handlers::OsmParser parser(projector);
  auto map = parser.fromOsmFile(osm, errors);
  if (!errors.empty()) throw std::invalid_argument(errors.front());
  if (map->laneletLayer.empty()) throw std::invalid_argument("empty guardrail_obb lanelet map");
  for (auto& point : map->pointLayer) {
    const auto x = point.attribute("local_x").asDouble().value();
    const auto y = point.attribute("local_y").asDouble().value();
    const auto z = point.attributeOr("local_z", point.z());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      throw std::invalid_argument("invalid guardrail_obb map local coordinates");
    point.x() = x;
    point.y() = y;
    point.z() = z;
  }
  const auto check_line = [](const lanelet::ConstLineString3d& line) {
    if (line.size() < 2) throw std::invalid_argument("guardrail_obb map line requires two points");
    double length = 0;
    for (std::size_t i = 0; i < line.size(); ++i) {
      const auto& point = line[i];
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()))
        throw std::invalid_argument("nonfinite guardrail_obb map geometry");
      if (i > 0) {
        const double segment_length = std::hypot(point.x() - line[i - 1].x(), point.y() - line[i - 1].y());
        if (!std::isfinite(segment_length) || segment_length <= 0)
          throw std::invalid_argument("zero length guardrail_obb map segment");
        length += segment_length;
      }
    }
    if (!std::isfinite(length) || length <= 0)
      throw std::invalid_argument("zero length guardrail_obb map geometry");
  };
  lanelet::Lanelets lanelets;
  for (auto& lanelet : map->laneletLayer) {
    lanelet.resetCache();
    check_line(lanelet.leftBound());
    check_line(lanelet.rightBound());
    check_line(lanelet.centerline());
    lanelets.push_back(lanelet);
  }
  // Restored local coordinates require rebuilt spatial indices.
  config.map = lanelet::utils::createMap(lanelets);
  return config;
}
}  // namespace object_detector
