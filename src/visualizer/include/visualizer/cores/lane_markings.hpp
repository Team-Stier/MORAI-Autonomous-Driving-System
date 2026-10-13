#pragma once
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace visualizer {
struct LaneMarkingsParameters {
  struct Line {
    std::vector<geometry_msgs::Point> points;
    std::string subtype = "solid";
    std::string color = "white";
    std::string semantic;
    std::string lane_type;
    double width_m = 0.15;
    bool virtual_boundary = false;
  };
  std::vector<Line> lines;
  double dash_length_m = 3;
  double dash_gap_m = 3;
  double double_line_gap_m = 0.15;
  double line_width_m = 0.12;
  bool flatten_z = true;
};

class LaneMarkings {
 public:
  explicit LaneMarkings(LaneMarkingsParameters parameters) : parameters_(std::move(parameters)) {}
  visualization_msgs::Marker process() {
    visualization_msgs::Marker marker;
    marker.ns = "lane_markings";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = parameters_.line_width_m;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1;
    for (const auto& line : parameters_.lines) {
      const bool yellow = line.semantic == "center_line" || line.lane_type == "501" || line.color == "yellow";
      const bool mixed = line.subtype == "solid_dashed" || line.subtype == "dashed_solid";
      if (line.virtual_boundary) {
        appendStripe(marker, line, 0, true);
      } else if (mixed) {
        const double offset = (line.width_m + parameters_.double_line_gap_m) / 2;
        appendStripe(marker, line, offset, line.subtype == "dashed_solid");
        appendStripe(marker, line, -offset, line.subtype == "solid_dashed");
      } else {
        appendStripe(marker, line, 0, line.subtype == "dashed");
      }
      auto color = marker.color;
      if (line.virtual_boundary) {
        color.r = 0;
        color.g = 0.7;
        color.a = 0.55;
      } else {
        color.b = yellow ? 0 : 1;
      }
      marker.colors.insert(marker.colors.end(), marker.points.size() - marker.colors.size(), color);
    }
    return marker;
  }

 private:
  static geometry_msgs::Point offsetPoint(const LaneMarkingsParameters::Line& line,
                                         std::size_t index, double offset) {
    auto point = line.points[index];
    if (offset == 0) return point;
    const auto& before = line.points[index == 0 ? index : index - 1];
    const auto& after = line.points[index + 1 == line.points.size() ? index : index + 1];
    double dx = after.x - before.x;
    double dy = after.y - before.y;
    if (dx == 0 && dy == 0) {
      dx = point.x - before.x;
      dy = point.y - before.y;
    }
    const double length = std::hypot(dx, dy);
    point.x -= offset * dy / length;
    point.y += offset * dx / length;
    return point;
  }

  static geometry_msgs::Point interpolate(const geometry_msgs::Point& from,
                                          const geometry_msgs::Point& to, double fraction) {
    geometry_msgs::Point point;
    point.x = from.x + fraction * (to.x - from.x);
    point.y = from.y + fraction * (to.y - from.y);
    point.z = from.z + fraction * (to.z - from.z);
    return point;
  }

  void appendStripe(visualization_msgs::Marker& marker, const LaneMarkingsParameters::Line& line,
                    double offset, bool dashed) const {
    double station = 0;
    const double period = parameters_.dash_length_m + parameters_.dash_gap_m;
    for (std::size_t i = 1; i < line.points.size(); ++i) {
      const double length = std::hypot(line.points[i].x - line.points[i - 1].x,
                                       line.points[i].y - line.points[i - 1].y);
      auto from = offsetPoint(line, i - 1, offset);
      auto to = offsetPoint(line, i, offset);
      if (parameters_.flatten_z) from.z = to.z = 0;
      if (!dashed) {
        marker.points.push_back(from);
        marker.points.push_back(to);
      } else {
        // shortcut: OSM has no dash endpoints; replace uniform spacing when measured paint intervals are supplied.
        for (double begin = std::floor(station / period) * period; begin < station + length; begin += period) {
          const double first = std::max(station, begin);
          const double last = std::min(station + length, begin + parameters_.dash_length_m);
          if (last > first) {
            marker.points.push_back(interpolate(from, to, (first - station) / length));
            marker.points.push_back(interpolate(from, to, (last - station) / length));
          }
        }
      }
      station += length;
    }
  }

  LaneMarkingsParameters parameters_;
};
}  // namespace visualizer
