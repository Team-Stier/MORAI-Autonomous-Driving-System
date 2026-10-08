#pragma once
#include <path_planner/libraries/checkpoints.hpp>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <pugixml.hpp>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace path_planner {
inline std::shared_ptr<lanelet::LaneletMap> readLaneletMap(const std::string& map_data) {
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
  for (auto& lanelet : map->laneletLayer) {
    lanelet.resetCache();
    lanelets.push_back(lanelet);
  }
  // Rebuild spatial indices after restoring the authoritative map coordinates.
  return lanelet::utils::createMap(lanelets);
}

// 지도를 해석하고 체크포인트 CSV의 모든 lanelet_id가 지도에 있는지 확인한다.
inline std::shared_ptr<lanelet::LaneletMap> readCheckedLaneletMap(const std::string& map_data,
                                                                  const std::string& checkpoints_data) {
  const auto checkpoints = readCheckpoints(checkpoints_data);
  auto map = readLaneletMap(map_data);
  for (const auto& checkpoint : checkpoints) {
    if (!map->laneletLayer.exists(checkpoint.lanelet_id))
      throw std::invalid_argument("checkpoint lanelet_id not found in map: " + std::to_string(checkpoint.lanelet_id));
  }
  return map;
}

}  // namespace path_planner
