#pragma once
#include <visualizer/cores/lane_markings.hpp>
#include <node_pipeline/extension.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <ros/console.h>
#include <cmath>
#include <array>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace visualizer {
inline LaneMarkingsParameters loadLaneMarkingsParameters(const node_pipeline::ParamReader& params) {
  LaneMarkingsParameters parameters;
  parameters.dash_length_m = params.number("dash_length_m");
  parameters.dash_gap_m = params.number("dash_gap_m");
  parameters.double_line_gap_m = params.number("double_line_gap_m");
  parameters.line_width_m = params.number("line_width_m");
  parameters.flatten_z = params.flag("flatten_z");
  for (const auto value : {parameters.dash_length_m, parameters.dash_gap_m,
                          parameters.double_line_gap_m, parameters.line_width_m})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid lane marking spacing");
  if (params.has("checkpoints_file")) {
    std::istringstream csv(params.asset("checkpoints_file"));
    std::string row;
    if (!std::getline(csv, row) || row != "x,y,z,yaw,lanelet_id")
      throw std::invalid_argument("invalid checkpoint CSV header");
    while (std::getline(csv, row)) {
      std::istringstream fields(row);
      std::array<double, 4> values{};
      for (auto& value : values) {
        std::string field;
        if (!std::getline(fields, field, ',')) throw std::invalid_argument("missing checkpoint coordinate");
        std::size_t consumed = 0; value = std::stod(field, &consumed);
        if (consumed != field.size() || !std::isfinite(value)) throw std::invalid_argument("invalid checkpoint coordinate");
      }
      std::string id;
      if (!std::getline(fields, id) || id.empty() || id.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("invalid checkpoint lanelet id");
      geometry_msgs::Point point; point.x = values[0]; point.y = values[1]; point.z = values[2];
      parameters.checkpoints.push_back(point);
    }
    if (parameters.checkpoints.size() != 16) throw std::invalid_argument("expected 16 checkpoints");
  }
  const double period = parameters.dash_length_m + parameters.dash_gap_m;
  if (!std::isfinite(period)) throw std::invalid_argument("lane marking dash period must be finite");

  using Tree = boost::property_tree::ptree;
  const auto tags = [](const Tree& element) {
    std::map<std::string, std::string> values;
    for (const auto& child : element) {
      if (child.first != "tag") continue;
      const auto key = child.second.get<std::string>("<xmlattr>.k");
      if (key != "type" && key != "subtype" && key != "color" && key != "local_x"
          && key != "local_y" && key != "local_z" && key != "ele" && key != "mgeo:boundary_id"
          && key != "mgeo:geometry" && key != "mgeo:semantic" && key != "mgeo:lane_type"
          && key != "mgeo:lane_width") continue;
      const auto value = child.second.get<std::string>("<xmlattr>.v");
      const auto inserted = values.emplace(key, value);
      if (!inserted.second && inserted.first->second != value)
        throw std::invalid_argument("conflicting map tag: " + key);
    }
    return values;
  };
  const auto number = [](const std::string& value) {
    std::size_t consumed = 0;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(parsed))
      throw std::invalid_argument("invalid map coordinate or width: " + value);
    return parsed;
  };

  std::istringstream asset(params.asset("map_file"));
  Tree document;
  boost::property_tree::read_xml(asset, document);
  const auto& osm = document.get_child("osm");
  std::map<long long, geometry_msgs::Point> points;
  std::map<long long, const Tree*> ways;
  std::set<long long> bounds;
  for (const auto& element : osm) {
    if (element.first == "node") {
      const auto values = tags(element.second);
      if (!values.count("local_x") || !values.count("local_y")
          || (!values.count("local_z") && !values.count("ele")))
        throw std::invalid_argument("map node requires local_x/local_y and local_z or ele");
      geometry_msgs::Point point;
      point.x = number(values.at("local_x"));
      point.y = number(values.at("local_y"));
      point.z = number(values.at(values.count("local_z") ? "local_z" : "ele"));
      if (!points.emplace(element.second.get<long long>("<xmlattr>.id"), point).second)
        throw std::invalid_argument("duplicate map node id");
    } else if (element.first == "way") {
      if (!ways.emplace(element.second.get<long long>("<xmlattr>.id"), &element.second).second)
        throw std::invalid_argument("duplicate map way id");
    } else if (element.first == "relation") {
      const auto values = tags(element.second);
      const auto type = values.find("type");
      if (type == values.end() || type->second != "lanelet") continue;
      for (const auto& member : element.second) {
        if (member.first != "member") continue;
        const auto role = member.second.get<std::string>("<xmlattr>.role");
        if (role == "left" || role == "right")
          bounds.insert(member.second.get<long long>("<xmlattr>.ref"));
      }
    }
  }

  std::set<long long> selected;
  std::set<std::string> active_sources;
  for (const auto id : bounds) {
    const auto way = ways.find(id);
    if (way == ways.end()) throw std::invalid_argument("lanelet references missing boundary way");
    auto values = tags(*way->second);
    if (values["type"] != "line_thin" && values["type"] != "virtual") continue;
    selected.insert(id);
    if (values["type"] == "line_thin" && !values["mgeo:boundary_id"].empty())
      active_sources.insert(values["mgeo:boundary_id"]);
  }
  std::map<std::string, std::vector<long long>> originals;
  for (const auto& way : ways) {
    auto values = tags(*way.second);
    if (values["type"] != "line_thin" || values["mgeo:geometry"] != "original"
        || active_sources.count(values["mgeo:boundary_id"])) continue;
    const auto source = values["mgeo:boundary_id"];
    if (!source.empty()) originals[source].push_back(way.first);
  }
  // Retained source ways overlap edited lanelet bounds; use originals only for otherwise unreferenced sources.
  for (const auto& source : originals)
    if (source.second.size() == 1) selected.insert(source.second.front());

  for (const auto id : selected) {
    const auto& way = *ways.at(id);
    auto values = tags(way);
    LaneMarkingsParameters::Line line;
    line.virtual_boundary = values["type"] == "virtual";
    line.subtype = values["subtype"];
    if (!line.virtual_boundary && line.subtype != "solid" && line.subtype != "dashed"
        && line.subtype != "solid_dashed" && line.subtype != "dashed_solid")
      throw std::invalid_argument("unsupported lane marking subtype: " + line.subtype);
    line.color = values.count("color") ? values["color"] : "white";
    line.semantic = values["mgeo:semantic"];
    line.lane_type = values["mgeo:lane_type"];
    if (values.count("mgeo:lane_width")) line.width_m = number(values["mgeo:lane_width"]);
    if (line.width_m <= 0) throw std::invalid_argument("lane marking width must be positive");
    if (!std::isfinite(line.width_m + parameters.double_line_gap_m))
      throw std::invalid_argument("lane marking compound spacing must be finite");
    double station = 0;
    for (const auto& child : way) {
      if (child.first != "nd") continue;
      const auto reference = child.second.get<long long>("<xmlattr>.ref");
      const auto point = points.find(reference);
      if (point == points.end()) throw std::invalid_argument("lane marking references missing node");
      if (!line.points.empty() && point->second.x == line.points.back().x && point->second.y == line.points.back().y)
        throw std::invalid_argument("lane marking has consecutive identical xy coordinates");
      if (!line.points.empty()) {
        station += std::hypot(point->second.x - line.points.back().x, point->second.y - line.points.back().y);
        if (!std::isfinite(station) || station + period == station)
          throw std::invalid_argument("lane marking distance or dash spacing is not representable");
      }
      line.points.push_back(point->second);
    }
    if (line.points.size() < 2) throw std::invalid_argument("lane marking requires at least two points");
    parameters.lines.push_back(std::move(line));
  }
  if (parameters.lines.empty()) throw std::invalid_argument("map has no unambiguous lane markings or virtual boundaries");
  ROS_INFO("Lane markings loaded %zu canonical physical and virtual ways", parameters.lines.size());
  return parameters;
}
}  // namespace visualizer
