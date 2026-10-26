#pragma once
#include <velocity_planner/cores/scored_stop_line_left.hpp>
#include <node_pipeline/extension.hpp>
#include <pugixml.hpp>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace velocity_planner {
inline ScoredStopLineLeftParameters loadScoredStopLineLeftParameters(const node_pipeline::ParamReader& params) {
  ScoredStopLineLeftParameters result;
  const auto map_data = params.asset("map_file");
  pugi::xml_document document;
  if (!document.load_buffer(map_data.data(), map_data.size()) ||
      std::string(document.document_element().name()) != "osm")
    throw std::invalid_argument("invalid scored stop-line OSM asset");
  const auto read_id = [](const std::string& value) -> std::int64_t {
    std::size_t parsed = 0;
    std::int64_t id = 0;
    try { id = std::stoll(value, &parsed); }
    catch (const std::exception&) { throw std::invalid_argument("invalid corridor OSM ID"); }
    if (parsed != value.size() || id == 0)
      throw std::invalid_argument("corridor OSM ID must be a nonzero signed integer");
    return id;
  };
  const auto coordinate = [](const pugi::xml_node& node, const char* key) {
    const std::string value = node.find_child_by_attribute("tag", "k", key).attribute("v").value();
    std::size_t parsed = 0;
    double result = 0;
    try { result = std::stod(value, &parsed); }
    catch (const std::exception&) { throw std::invalid_argument("invalid corridor coordinate"); }
    if (parsed != value.size() || !std::isfinite(result))
      throw std::invalid_argument("corridor coordinate must be finite");
    return result;
  };
  std::unordered_map<std::int64_t, pugi::xml_node> nodes, ways, relations;
  const auto osm = document.document_element();
  for (const auto& node : osm.children("node"))
    if (!nodes.emplace(read_id(node.attribute("id").value()), node).second)
      throw std::invalid_argument("duplicate corridor OSM node");
  for (const auto& way : osm.children("way"))
    if (!ways.emplace(read_id(way.attribute("id").value()), way).second)
      throw std::invalid_argument("duplicate corridor OSM way");
  for (const auto& relation : osm.children("relation"))
    if (!relations.emplace(read_id(relation.attribute("id").value()), relation).second)
      throw std::invalid_argument("duplicate corridor OSM relation");
  for (const auto& key : params.keys("corridor_lanelets")) {
    const auto stop_id = read_id(key);
    if (stop_id != 7294 && stop_id != 6254 && stop_id != 4188 && stop_id != 4356)
      throw std::invalid_argument("corridor is restricted to scored stop-line IDs");
    const auto stop = ways.find(stop_id);
    if (stop == ways.end() || std::string(stop->second.find_child_by_attribute(
        "tag", "k", "type").attribute("v").value()) != "stop_line")
      throw std::invalid_argument("corridor stop-line way is missing or has wrong type");
    std::size_t stop_points = 0;
    for (const auto& reference : stop->second.children("nd")) {
      const auto node = nodes.find(read_id(reference.attribute("ref").value()));
      if (node == nodes.end()) throw std::invalid_argument("missing scored stop-line node");
      coordinate(node->second, "local_x");
      coordinate(node->second, "local_y");
      ++stop_points;
    }
    if (stop_points < 2) throw std::invalid_argument("scored stop line needs two points");
    double projection = 0;
    if (params.has("corridor_projection_m/" + key))
      projection = params.number("corridor_projection_m/" + key);
    if (!std::isfinite(projection) || projection < 0)
      throw std::invalid_argument("corridor projection must be finite and nonnegative");
    std::unordered_set<std::int64_t> seen;
    for (const double value : params.numbers("corridor_lanelets/" + key)) {
      if (!std::isfinite(value) || value == 0 || std::floor(value) != value ||
          value < -std::ldexp(1.0, 63) || value >= std::ldexp(1.0, 63))
        throw std::invalid_argument("lanelet ID must be a finite nonzero signed integer");
      const auto id = static_cast<std::int64_t>(value);
      if (!seen.insert(id).second) throw std::invalid_argument("duplicate corridor lanelet ID");
      const auto relation = relations.find(id);
      if (relation == relations.end() || std::string(relation->second.find_child_by_attribute(
          "tag", "k", "type").attribute("v").value()) != "lanelet")
        throw std::invalid_argument("corridor relation must be an existing lanelet");
      ScoredStopLineLeftParameters::Corridor corridor;
      corridor.stop_line_id = stop_id;
      corridor.projection_m = projection;
      for (const auto& member : relation->second.children("member")) {
        const std::string role = member.attribute("role").value();
        if (role != "left" && role != "right") continue;
        if (std::string(member.attribute("type").value()) != "way")
          throw std::invalid_argument("lanelet left/right member must be a way");
        auto& points = role == "left" ? corridor.left : corridor.right;
        if (!points.empty()) throw std::invalid_argument("duplicate lanelet boundary role");
        const auto boundary = ways.find(read_id(member.attribute("ref").value()));
        if (boundary == ways.end()) throw std::invalid_argument("missing lanelet boundary way");
        const std::string type = boundary->second.find_child_by_attribute(
            "tag", "k", "type").attribute("v").value();
        if (type != "line_thin" && type != "line_thick" && type != "virtual" &&
            type != "curbstone" && type != "road_border" && type != "guard_rail")
          throw std::invalid_argument("invalid lanelet boundary way type");
        for (const auto& reference : boundary->second.children("nd")) {
          const auto node = nodes.find(read_id(reference.attribute("ref").value()));
          if (node == nodes.end()) throw std::invalid_argument("missing corridor boundary node");
          points.push_back({{coordinate(node->second, "local_x"),
                             coordinate(node->second, "local_y")}});
        }
        if (points.size() < 2) throw std::invalid_argument("lanelet boundary needs two points");
      }
      if (corridor.left.size() < 2 || corridor.right.size() < 2)
        throw std::invalid_argument("lanelet needs one left and one right boundary");
      result.corridors.push_back(std::move(corridor));
    }
  }
  return result;
}
}  // namespace velocity_planner
