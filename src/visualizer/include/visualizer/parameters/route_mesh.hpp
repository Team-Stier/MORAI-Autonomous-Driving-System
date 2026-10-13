#pragma once
#include <visualizer/cores/route_mesh.hpp>
#include <node_pipeline/extension.hpp>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <pugixml.hpp>
#include <cmath>
#include <stdexcept>

namespace visualizer {
inline RouteMeshParameters loadRouteMeshParameters(const node_pipeline::ParamReader& params) {
  RouteMeshParameters parameters;
  parameters.route_match_distance_m = params.number("route_match_distance_m");
  parameters.flatten_z = params.flag("flatten_z");
  if (!std::isfinite(parameters.route_match_distance_m) || parameters.route_match_distance_m <= 0)
    throw std::invalid_argument("route_match_distance_m must be positive and finite");
  const auto map_data = params.asset("map_file");
  pugi::xml_document document;
  if (!document.load_buffer(map_data.data(), map_data.size()))
    throw std::invalid_argument("invalid Lanelet2 OSM XML");
  lanelet::osm::Errors errors;
  const auto osm = lanelet::osm::read(document, &errors);
  const lanelet::DefaultProjector projector(lanelet::Origin({0.0, 0.0}));
  const lanelet::io_handlers::OsmParser parser(projector);
  auto map = parser.fromOsmFile(osm, errors);
  if (!errors.empty()) throw std::invalid_argument(errors.front());
  if (map->laneletLayer.empty()) throw std::invalid_argument("empty lanelet map");
  for (auto& point : map->pointLayer) {
    const auto x = point.attribute("local_x").asDouble().value();
    const auto y = point.attribute("local_y").asDouble().value();
    const auto z = point.attributeOr("local_z", point.z());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      throw std::invalid_argument("invalid map local_x/local_y/local_z");
    point.x() = x;
    point.y() = y;
    point.z() = z;
  }
  lanelet::Lanelets lanelets;
  for (auto& lane : map->laneletLayer) {
    lane.resetCache();
    lanelets.push_back(lane);
  }
  parameters.map = lanelet::utils::createMap(lanelets);
  return parameters;
}
}  // namespace visualizer
