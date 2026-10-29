#pragma once
#include <velocity_planner/banks/stop_line_state.hpp>
#include <node_pipeline/extension.hpp>
#include <pugixml.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace velocity_planner {
inline StopLineStateParameters loadStopLineStateParameters(const node_pipeline::ParamReader& params) {
  StopLineStateParameters result;
  if (params.has("intersection_speed_mps")) result.intersection_speed_mps = params.number("intersection_speed_mps");
  if (params.has("intersection_before_m")) result.intersection_before_m = params.number("intersection_before_m");
  if (params.has("intersection_after_m")) result.intersection_after_m = params.number("intersection_after_m");
  if (params.has("intersection_deceleration_mps2")) result.intersection_deceleration_mps2 = params.number("intersection_deceleration_mps2");
  if (!std::isfinite(result.intersection_speed_mps) || result.intersection_speed_mps < 0 ||
      !std::isfinite(result.intersection_before_m) || result.intersection_before_m < 0 ||
      !std::isfinite(result.intersection_after_m) || result.intersection_after_m < 0 ||
      !std::isfinite(result.intersection_deceleration_mps2) || result.intersection_deceleration_mps2 <= 0)
    throw std::invalid_argument("invalid intersection speed/distance/deceleration parameters");
  if (params.has("jump_distance_m")) result.jump_distance_m = params.number("jump_distance_m");
  if (params.has("jump_speed_mps")) result.jump_speed_mps = params.number("jump_speed_mps");
  if (!std::isfinite(result.jump_distance_m) || result.jump_distance_m <= 0 ||
      !std::isfinite(result.jump_speed_mps) || result.jump_speed_mps <= 0)
    throw std::invalid_argument("invalid stop_line_state jump_distance_m/jump_speed_mps");
  if (params.has("stop_line_extension_m")) result.stop_line_extension_m = params.number("stop_line_extension_m");
  if (!std::isfinite(result.stop_line_extension_m) || result.stop_line_extension_m < 0 || result.stop_line_extension_m > 5)
    throw std::invalid_argument("stop_line_extension_m must be between 0 and 5 m");
  result.x_m = params.number("x_m");
  result.y_m = params.number("y_m");
  result.z_mps2 = params.number("z_mps2");
  if (params.has("stop_response_time_sec")) result.stop_response_time_sec = params.number("stop_response_time_sec");
  if (params.has("stop_position_tolerance_m")) result.stop_position_tolerance_m = params.number("stop_position_tolerance_m");
  if (!std::isfinite(result.stop_response_time_sec) || result.stop_response_time_sec <= 0 ||
      !std::isfinite(result.stop_position_tolerance_m) || result.stop_position_tolerance_m < 0)
    throw std::invalid_argument("invalid stop response time or position tolerance");
  const auto n_frames = params.integer("n_frames");
  if (params.has("stop_timeout_sec")) result.stop_timeout_sec = params.number("stop_timeout_sec");
  if (!std::isfinite(result.x_m) || result.x_m < 0 || !std::isfinite(result.y_m) || result.y_m <= 0 ||
      !std::isfinite(result.z_mps2) || result.z_mps2 <= 0 || n_frames < 1 ||
      n_frames > std::numeric_limits<int>::max() || !std::isfinite(result.stop_timeout_sec) ||
      result.stop_timeout_sec <= 0)
    throw std::invalid_argument("invalid stop_line_state x_m/y_m/z_mps2/n_frames/stop_timeout_sec");
  result.n_frames = static_cast<int>(n_frames);

  const auto map_data = params.asset("map_file");
  pugi::xml_document document;
  if (!document.load_buffer(map_data.data(), map_data.size()) ||
      std::string(document.document_element().name()) != "osm")
    throw std::invalid_argument("invalid stop_line_state OSM XML");

  const auto readId = [](const pugi::xml_attribute& attribute) -> std::int64_t {
    const std::string value = attribute.value();
    std::size_t parsed = 0;
    std::int64_t id = 0;
    try { id = std::stoll(value, &parsed); }
    catch (const std::exception&) { throw std::invalid_argument("invalid OSM node/stop_line id or ref"); }
    if (parsed != value.size() || id == 0)
      throw std::invalid_argument("invalid OSM node/stop_line id or ref");
    return id;
  };
  const auto readCoordinate = [](const pugi::xml_node& node, const char* key) {
    const std::string value = node.find_child_by_attribute("tag", "k", key).attribute("v").value();
    std::size_t parsed = 0;
    double coordinate = 0;
    try { coordinate = std::stod(value, &parsed); }
    catch (const std::exception&) { throw std::invalid_argument("invalid stop_line node local_x/local_y"); }
    if (parsed != value.size() || !std::isfinite(coordinate))
      throw std::invalid_argument("invalid stop_line node local_x/local_y");
    return coordinate;
  };

  const auto osm = document.document_element();
  std::unordered_map<std::int64_t, pugi::xml_node> nodes;
  for (const auto& node : osm.children("node")) {
    if (!nodes.emplace(readId(node.attribute("id")), node).second)
      throw std::invalid_argument("duplicate OSM node id");
  }
  std::unordered_set<std::int64_t> line_ids;
  for (const auto& way : osm.children("way")) {
    if (std::string(way.find_child_by_attribute("tag", "k", "type").attribute("v").value()) != "stop_line")
      continue;
    StopLineStateParameters::StopLine line;
    line.id = readId(way.attribute("id"));
    // MORAI stop-line node order defines the approach normal (-dy, dx).
    line.directional_extension = way.find_child_by_attribute("tag", "k", "mgeo:boundary_id");
    if (!line_ids.insert(line.id).second) throw std::invalid_argument("duplicate stop_line way id");
    for (const auto& reference : way.children("nd")) {
      const auto node = nodes.find(readId(reference.attribute("ref")));
      if (node == nodes.end()) throw std::invalid_argument("stop_line references missing OSM node");
      line.points.push_back({{readCoordinate(node->second, "local_x"), readCoordinate(node->second, "local_y")}});
    }
    if (line.points.size() < 2) throw std::invalid_argument("stop_line must contain at least two nodes");
    result.stop_lines.push_back(std::move(line));
  }
  if (result.stop_lines.empty()) throw std::invalid_argument("OSM asset contains no stop_line ways");
  return result;
}
}  // namespace velocity_planner
