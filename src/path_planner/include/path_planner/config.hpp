#pragma once
#include <path_planner/types.hpp>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <ros/ros.h>
#include <pugixml.hpp>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace path_planner {
inline GlobalPathParameters readGlobalPathParameters(
    const std::string& map_data, const std::string& checkpoints_data) {
  GlobalPathParameters parameters;
  std::istringstream csv(checkpoints_data);
  std::string line;
  auto read_line = [&] {
    if (!std::getline(csv, line)) throw std::invalid_argument("expected 16 checkpoints with x,y,z,yaw,lanelet_id header");
    if (!line.empty() && line.back() == '\r') line.pop_back();
  };
  read_line();
  if (line != "x,y,z,yaw,lanelet_id") throw std::invalid_argument("expected checkpoints header x,y,z,yaw,lanelet_id");
  for (auto& checkpoint : parameters.checkpoints) {
    read_line();
    std::istringstream row(line);
    std::array<double, 4> values;
    for (auto& value : values) {
      std::string field;
      if (!std::getline(row, field, ',')) throw std::invalid_argument("missing checkpoint field");
      std::size_t parsed = 0;
      value = std::stod(field, &parsed);
      if (parsed != field.size() || !std::isfinite(value))
        throw std::invalid_argument("invalid checkpoint field");
    }
    std::string id_field;
    if (!std::getline(row, id_field, ',') || id_field.empty() ||
        id_field.find_first_not_of("0123456789") != std::string::npos)
      throw std::invalid_argument("expected positive integer checkpoint lanelet_id");
    const auto lanelet_id = std::stoll(id_field);
    if (lanelet_id <= 0) throw std::invalid_argument("expected positive integer checkpoint lanelet_id");
    if (row.peek() != std::char_traits<char>::eof() || line.back() == ',')
      throw std::invalid_argument("expected five checkpoint fields");
    checkpoint = {values[0], values[1], values[2], values[3] * std::acos(-1.0) / 180.0, lanelet_id};
  }
  if (std::getline(csv, line)) throw std::invalid_argument("expected exactly 16 checkpoints");

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
  for (const auto& checkpoint : parameters.checkpoints) {
    if (!map->laneletLayer.exists(checkpoint.lanelet_id))
      throw std::invalid_argument("checkpoint lanelet_id not found in map: " + std::to_string(checkpoint.lanelet_id));
  }
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
  parameters.map = lanelet::utils::createMap(lanelets);
  return parameters;
}

inline PredictionParameters loadPredictionParameters(ros::NodeHandle& params) {
  PredictionParameters parameters;
  params.param("prediction_core/yaw_offset_rad", parameters.yaw_offset_rad, parameters.yaw_offset_rad);
  if (!std::isfinite(parameters.yaw_offset_rad))
    throw std::invalid_argument("prediction_core/yaw_offset_rad must be finite");
  return parameters;
}

inline LocalPathParameters loadLocalPathParameters(
    ros::NodeHandle& params, const std::shared_ptr<lanelet::LaneletMap>& map) {
  LocalPathParameters parameters;
  parameters.map = map;
  params.param("local_path/local_path_length_m", parameters.local_path_length_m, 20.0);
  if (!std::isfinite(parameters.local_path_length_m) || parameters.local_path_length_m <= 0)
    throw std::invalid_argument("local_path/local_path_length_m must be finite and positive");
  return parameters;
}
}  // namespace path_planner
