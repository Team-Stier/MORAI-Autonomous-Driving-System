#pragma once
#include <object_detector/cores/tracking.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <pugixml.hpp>
#include <set>

namespace object_detector {
inline TrackingParameters loadTrackingParameters(const node_pipeline::ParamReader& reader) {
  TrackingParameters p;
  if(reader.has("use_observation_velocity"))p.use_observation_velocity=reader.flag("use_observation_velocity");
  p.association_gate_m = reader.number("association_gate_m");
  if(reader.has("association_max_speed_mps")) {
    p.association_max_speed_mps = reader.number("association_max_speed_mps");
    p.association_speed_margin_m = reader.number("association_speed_margin_m");
    p.association_max_gate_m = reader.number("association_max_gate_m");
    p.association_min_moving_extent_m = reader.number("association_min_moving_extent_m");
    if(!std::isfinite(p.association_max_speed_mps) || p.association_max_speed_mps < 0 ||
       !std::isfinite(p.association_min_moving_extent_m) || p.association_min_moving_extent_m < 0 ||
       !std::isfinite(p.association_speed_margin_m) || p.association_speed_margin_m < 0 ||
       !std::isfinite(p.association_max_gate_m) || p.association_max_gate_m < p.association_gate_m)
      throw std::invalid_argument("invalid tracking association speed gate");
  }
  p.shape_weight = reader.number("shape_weight");
  p.max_age_sec = reader.number("max_age_sec");
  if(reader.has("moving_vehicle_coast_sec"))p.moving_vehicle_coast_sec=reader.number("moving_vehicle_coast_sec");
  if(!std::isfinite(p.moving_vehicle_coast_sec) || p.moving_vehicle_coast_sec<0 || p.moving_vehicle_coast_sec>1.5 || p.moving_vehicle_coast_sec>p.max_age_sec)throw std::invalid_argument("invalid moving vehicle coast duration");
  p.history_size = static_cast<int>(reader.integer("history_size"));
  p.min_acceleration_observations = static_cast<int>(reader.integer("min_acceleration_observations"));
  p.min_velocity_observations = static_cast<int>(reader.integer("min_velocity_observations"));
  p.minimum_velocity_span_sec = reader.number("minimum_velocity_span_sec");
  p.pair_baseline_sec = reader.number("pair_baseline_sec");
  p.history_window_sec = reader.number("history_window_sec");
  if (reader.has("ring_lanelet_ids")) {
    const auto asset = reader.asset("map_file");
    pugi::xml_document xml;
    if (!xml.load_buffer(asset.data(), asset.size())) throw std::invalid_argument("invalid tracking map XML");
    lanelet::osm::Errors errors;
    const auto osm = lanelet::osm::read(xml, &errors);
    const lanelet::DefaultProjector projector(lanelet::Origin({0., 0.}));
    const lanelet::io_handlers::OsmParser parser(projector);
    const auto map = parser.fromOsmFile(osm, errors);
    if (!errors.empty()) throw std::invalid_argument(errors.front());
    for (auto& point : map->pointLayer) {
      point.x() = point.attribute("local_x").asDouble().value();
      point.y() = point.attribute("local_y").asDouble().value();
      point.z() = point.attributeOr("local_z", point.z());
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()))
        throw std::invalid_argument("nonfinite tracking map point");
    }
    std::set<lanelet::Id> selected;
    for (double value : reader.numbers("ring_lanelet_ids")) {
      if (!std::isfinite(value) || value <= 0 || value >= static_cast<double>(std::numeric_limits<lanelet::Id>::max()) || value != std::floor(value))
        throw std::invalid_argument("invalid tracking ring lanelet ID");
      const auto id = static_cast<lanelet::Id>(value);
      if (!selected.insert(id).second || !map->laneletLayer.exists(id)) throw std::invalid_argument("missing or duplicate tracking ring lanelet");
      auto lane = map->laneletLayer.get(id); lane.resetCache();
      for (const auto& q : lane.centerline3d()) {
        Eigen::Vector3d point(q.x(),q.y(),q.z());
        if (p.ring_centerline.empty() || (point-p.ring_centerline.back()).head<2>().norm()>1e-6)
          p.ring_centerline.push_back(point);
      }
    }
    if (selected.size()<2 || p.ring_centerline.size()<3) throw std::invalid_argument("tracking ring requires a closed lanelet chain");
    if ((p.ring_centerline.front()-p.ring_centerline.back()).head<2>().norm()>1.)
      throw std::invalid_argument("tracking ring is not closed");
    if ((p.ring_centerline.front()-p.ring_centerline.back()).head<2>().norm()>1e-6)
      p.ring_centerline.push_back(p.ring_centerline.front());
    p.ring_match_distance_m=reader.number("ring_match_distance_m");
    if (!std::isfinite(p.ring_match_distance_m) || p.ring_match_distance_m<=0 || p.ring_match_distance_m>4)
      throw std::invalid_argument("invalid tracking ring match distance");
  }
  if (!std::isfinite(p.association_gate_m) ||
      p.association_gate_m <= 0 || !std::isfinite(p.shape_weight) || p.shape_weight < 0 ||
      !std::isfinite(p.max_age_sec) || p.max_age_sec <= 0 || p.history_size < 3 ||
      p.min_acceleration_observations < 3 || p.min_acceleration_observations > p.history_size)
    throw std::invalid_argument("invalid tracker parameters");
  if (p.min_velocity_observations < 2 || p.min_velocity_observations > p.history_size ||
      !std::isfinite(p.minimum_velocity_span_sec) || p.minimum_velocity_span_sec <= 0 ||
      !std::isfinite(p.pair_baseline_sec) || p.pair_baseline_sec <= 0 ||
      p.pair_baseline_sec > p.minimum_velocity_span_sec ||
      !std::isfinite(p.history_window_sec) || p.history_window_sec < p.minimum_velocity_span_sec)
    throw std::invalid_argument("invalid tracking history parameters");
  return p;
}
}  // namespace object_detector
