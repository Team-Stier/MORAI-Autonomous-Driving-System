#pragma once
#include <object_detector/cores/acquisition_bbox.hpp>
#include <node_pipeline/extension.hpp>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <pugixml.hpp>
#include <cmath>
#include <stdexcept>
namespace object_detector {
inline AcquisitionBboxParameters loadAcquisitionBboxParameters(const node_pipeline::ParamReader& params) {
  AcquisitionBboxParameters config;
  if(params.has("require_road_center_for_completion"))
    config.require_road_center_for_completion=params.flag("require_road_center_for_completion");
  if(params.has("association_gate_m"))config.association_gate_m=params.number("association_gate_m");
  if(params.has("maximum_speed_mps"))config.maximum_speed_mps=params.number("maximum_speed_mps");
  if(params.has("motion_history_sec"))config.motion_history_sec=params.number("motion_history_sec");
  if(!std::isfinite(config.association_gate_m) || config.association_gate_m<=0 ||
     !std::isfinite(config.maximum_speed_mps) || config.maximum_speed_mps<=0 ||
     !std::isfinite(config.motion_history_sec) || config.motion_history_sec<.2 || config.motion_history_sec>2)
    throw std::invalid_argument("invalid acquisition bbox motion parameters");
  config.vehicle_length_m=params.number("vehicle_length_m");config.vehicle_width_m=params.number("vehicle_width_m");
  if(params.has("vehicle_height_m"))config.vehicle_height_m=params.number("vehicle_height_m");
  if(!std::isfinite(config.vehicle_height_m) || config.vehicle_height_m<=0)throw std::invalid_argument("invalid vehicle height prior");
  config.minimum_height_m=params.number("minimum_height_m");config.road_match_distance_m=params.number("road_match_distance_m");
  for(double x:{config.vehicle_length_m,config.vehicle_width_m,config.minimum_height_m,config.road_match_distance_m})
    if(!std::isfinite(x) || x<=0)throw std::invalid_argument("invalid viewpoint bbox parameter");
  const auto position=params.numbers("lidar_position_m");
  if(position.size()!=3)throw std::invalid_argument("lidar position requires XYZ");
  for(unsigned i=0;i<3;++i){if(!std::isfinite(position[i]))throw std::invalid_argument("nonfinite lidar position");config.lidar_position[i]=position[i];}
  const auto map_data = params.asset("map_file");
  pugi::xml_document document;
  if (!document.load_buffer(map_data.data(), map_data.size()))
    throw std::invalid_argument("invalid road_roi Lanelet2 OSM XML");
  lanelet::osm::Errors errors;
  const auto osm = lanelet::osm::read(document, &errors);
  const lanelet::DefaultProjector projector(lanelet::Origin({0.0, 0.0}));
  const lanelet::io_handlers::OsmParser parser(projector);
  auto map = parser.fromOsmFile(osm, errors);
  if (!errors.empty()) throw std::invalid_argument(errors.front());
  if (map->laneletLayer.empty()) throw std::invalid_argument("empty road_roi lanelet map");
  for (auto& point : map->pointLayer) {
    const auto x = point.attribute("local_x").asDouble().value();
    const auto y = point.attribute("local_y").asDouble().value();
    const auto z = point.attributeOr("local_z", point.z());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      throw std::invalid_argument("invalid road_roi map local coordinates");
    point.x() = x;
    point.y() = y;
    point.z() = z;
  }
  const auto check_line = [](const lanelet::ConstLineString3d& line) {
    if (line.size() < 2) throw std::invalid_argument("road_roi map line requires two points");
    double length = 0;
    for (std::size_t i = 0; i < line.size(); ++i) {
      const auto& point = line[i];
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()))
        throw std::invalid_argument("nonfinite road_roi map geometry");
      if (i > 0) {
        const double segment_length = std::hypot(point.x() - line[i - 1].x(), point.y() - line[i - 1].y());
        if (!std::isfinite(segment_length) || segment_length <= 0)
          throw std::invalid_argument("zero length road_roi map segment");
        length += segment_length;
      }
    }
    if (!std::isfinite(length) || length <= 0)
      throw std::invalid_argument("zero length road_roi map geometry");
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

  for(double id:params.numbers("ring_lanelet_ids")) {
    if(!std::isfinite(id) || id<=0 || id!=std::floor(id) || !config.map->laneletLayer.exists(static_cast<lanelet::Id>(id)))
      throw std::invalid_argument("invalid bbox ring lanelet");
    config.ring_lanelet_ids.push_back(static_cast<lanelet::Id>(id));
  }
  return config;
}
}
