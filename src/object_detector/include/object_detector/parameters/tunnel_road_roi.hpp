#pragma once
#include <object_detector/cores/tunnel_road_roi.hpp>
#include <node_pipeline/extension.hpp>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <pugixml.hpp>
#include <cmath>
#include <stdexcept>

namespace object_detector {
inline TunnelRoadRoiParameters loadTunnelRoadRoiParameters(const node_pipeline::ParamReader& params) {
  TunnelRoadRoiParameters config;
  config.forward_distance_m = params.number("forward_distance_m");
  config.left_inflation_m = params.number("left_inflation_m");
  config.right_inflation_m = params.number("right_inflation_m");
  config.z_min_m = params.number("z_min_m");
  config.z_max_m = params.number("z_max_m");
  if (!std::isfinite(config.forward_distance_m) || config.forward_distance_m <= 0 ||
      !std::isfinite(config.left_inflation_m) || config.left_inflation_m < 0 ||
      !std::isfinite(config.right_inflation_m) || config.right_inflation_m < 0 ||
      !std::isfinite(config.z_min_m) || !std::isfinite(config.z_max_m) ||
      config.z_min_m > config.z_max_m)
    throw std::invalid_argument("tunnel_road_roi requires finite positive forward distance, nonnegative inflation and ordered Z bounds");
  config.tunnel_origin_x_m = params.number("tunnel_origin_x_m");
  config.tunnel_origin_y_m = params.number("tunnel_origin_y_m");
  config.tunnel_axis_yaw_rad = params.number("tunnel_axis_yaw_rad");
  config.tunnel_min_station_m = params.number("tunnel_min_station_m");
  config.tunnel_max_station_m = params.number("tunnel_max_station_m");
  config.tunnel_left_wall_m = params.number("tunnel_left_wall_m");
  config.tunnel_right_wall_m = params.number("tunnel_right_wall_m");
  config.wall_clearance_m = params.number("wall_clearance_m");
  config.walkway_min_z_m = params.number("walkway_min_z_m");
  for (double value : {config.tunnel_origin_x_m, config.tunnel_origin_y_m, config.tunnel_axis_yaw_rad,
                       config.tunnel_min_station_m, config.tunnel_max_station_m, config.tunnel_left_wall_m,
                       config.tunnel_right_wall_m, config.wall_clearance_m, config.walkway_min_z_m})
    if (!std::isfinite(value)) throw std::invalid_argument("tunnel_road_roi requires finite tunnel geometry");
  if (config.tunnel_min_station_m >= config.tunnel_max_station_m || config.wall_clearance_m < 0 ||
      config.tunnel_right_wall_m + config.wall_clearance_m >= config.tunnel_left_wall_m - config.wall_clearance_m)
    throw std::invalid_argument("tunnel_road_roi requires an ordered station range and walls wider than their clearance");

  const auto map_data = params.asset("map_file");
  pugi::xml_document document;
  if (!document.load_buffer(map_data.data(), map_data.size()))
    throw std::invalid_argument("invalid tunnel_road_roi Lanelet2 OSM XML");
  lanelet::osm::Errors errors;
  const auto osm = lanelet::osm::read(document, &errors);
  const lanelet::DefaultProjector projector(lanelet::Origin({0.0, 0.0}));
  const lanelet::io_handlers::OsmParser parser(projector);
  auto map = parser.fromOsmFile(osm, errors);
  if (!errors.empty()) throw std::invalid_argument(errors.front());
  if (map->laneletLayer.empty()) throw std::invalid_argument("empty tunnel_road_roi lanelet map");
  for (auto& point : map->pointLayer) {
    const auto x = point.attribute("local_x").asDouble().value();
    const auto y = point.attribute("local_y").asDouble().value();
    const auto z = point.attributeOr("local_z", point.z());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      throw std::invalid_argument("invalid tunnel_road_roi map local coordinates");
    point.x() = x;
    point.y() = y;
    point.z() = z;
  }
  const auto check_line = [](const lanelet::ConstLineString3d& line) {
    if (line.size() < 2) throw std::invalid_argument("tunnel_road_roi map line requires two points");
    double length = 0;
    for (std::size_t i = 0; i < line.size(); ++i) {
      const auto& point = line[i];
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()))
        throw std::invalid_argument("nonfinite tunnel_road_roi map geometry");
      if (i > 0) {
        const double segment_length = std::hypot(point.x() - line[i - 1].x(), point.y() - line[i - 1].y());
        if (!std::isfinite(segment_length) || segment_length <= 0)
          throw std::invalid_argument("zero length tunnel_road_roi map segment");
        length += segment_length;
      }
    }
    if (!std::isfinite(length) || length <= 0)
      throw std::invalid_argument("zero length tunnel_road_roi map geometry");
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
