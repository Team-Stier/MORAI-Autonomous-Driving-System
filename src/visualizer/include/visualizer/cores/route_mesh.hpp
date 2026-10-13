#pragma once
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace visualizer {
struct RouteMeshParameters {
  std::shared_ptr<lanelet::LaneletMap> map;
  double route_match_distance_m = 1.0;
  bool flatten_z = true;
};

class RouteMesh {
 public:
  explicit RouteMesh(RouteMeshParameters parameters) : parameters_(std::move(parameters)) {
    mesh_.ns = "route";
    mesh_.id = 0;
    mesh_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    mesh_.action = visualization_msgs::Marker::DELETE;
    mesh_.pose.orientation.w = 1.0;
    mesh_.scale.x = mesh_.scale.y = mesh_.scale.z = 1.0;
    mesh_.color.r = 1.0;
    mesh_.color.g = 0.8;
    mesh_.color.b = 0.05;
    mesh_.color.a = 0.35;
  }

  visualization_msgs::Marker process(const nav_msgs::Path& input) {
    std::vector<geometry_msgs::Point> positions;
    positions.reserve(input.poses.size());
    for (const auto& pose : input.poses) positions.push_back(pose.pose.position);
    if (positions == positions_) return mesh_;
    positions_ = std::move(positions);
    mesh_.points.clear();
    mesh_.colors.clear();
    std::set<lanelet::Id> route;
    for (std::size_t i = 1; i < positions_.size(); ++i) {
      const lanelet::BasicPoint2d from(positions_[i - 1].x, positions_[i - 1].y);
      const lanelet::BasicPoint2d to(positions_[i].x, positions_[i].y);
      const auto direction = (to - from).eval();
      if (direction.norm() < 1e-6) continue;
      const auto middle = ((from + to) / 2.0).eval();
      double best = std::numeric_limits<double>::infinity();
      lanelet::Id matched = lanelet::InvalId;
      // shortcut: match ten nearby centerlines; supply lanelet IDs if overlapping routes become ambiguous.
      for (const auto& candidate : lanelet::geometry::findNearest(parameters_.map->laneletLayer, middle, 10)) {
        const auto line = candidate.second.centerline2d().basicLineString();
        if (line.size() < 2) continue;
        const auto segment = lanelet::geometry::closestSegment(line, middle);
        const auto tangent = (segment.second - segment.first).eval();
        if (tangent.norm() < 1e-6) continue;
        const double distance = boost::geometry::distance(line, middle);
        if (distance > parameters_.route_match_distance_m) continue;
        const double score = distance + 0.2 * (1.0 - std::abs(tangent.normalized().dot(direction.normalized())));
        if (score < best) {
          best = score;
          matched = candidate.second.id();
        }
      }
      if (matched != lanelet::InvalId) route.insert(matched);
    }
    auto boundary_color = mesh_.color;
    boundary_color.a = 1.0;
    for (const auto id : route) {
      const auto lane = parameters_.map->laneletLayer.get(id);
      const auto polygon = lane.polygon3d();
      try {
        for (const auto& triangle : lanelet::geometry::triangulate(lane.polygon2d().basicPolygon())) {
          for (const auto index : triangle) {
            mesh_.points.push_back(point(polygon[index].x(), polygon[index].y(), polygon[index].z(), 0.02));
            mesh_.colors.push_back(mesh_.color);
          }
        }
      } catch (const std::exception& error) {
        ROS_WARN("Lanelet %lld fill skipped: %s", static_cast<long long>(id), error.what());
      }
      for (const auto& bound : {lane.leftBound3d(), lane.rightBound3d()}) {
        for (std::size_t i = 1; i < bound.size(); ++i)
          appendRibbon(point(bound[i - 1].x(), bound[i - 1].y(), bound[i - 1].z(), 0.04),
                       point(bound[i].x(), bound[i].y(), bound[i].z(), 0.04), 0.3, boundary_color);
      }
    }
    mesh_.action = mesh_.points.empty() ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
    return mesh_;
  }

 private:
  geometry_msgs::Point point(double x, double y, double z, double lift) const {
    geometry_msgs::Point result;
    result.x = x;
    result.y = y;
    result.z = (parameters_.flatten_z ? 0.0 : z) + lift;
    return result;
  }

  void appendRibbon(const geometry_msgs::Point& from, const geometry_msgs::Point& to,
                    double width, const std_msgs::ColorRGBA& color) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) return;
    auto a = from;
    auto b = from;
    auto c = to;
    auto d = to;
    const double offset_x = -dy * width / (2.0 * length);
    const double offset_y = dx * width / (2.0 * length);
    a.x += offset_x;
    a.y += offset_y;
    b.x -= offset_x;
    b.y -= offset_y;
    c.x += offset_x;
    c.y += offset_y;
    d.x -= offset_x;
    d.y -= offset_y;
    for (const auto& corner : {a, b, c, b, d, c}) {
      mesh_.points.push_back(corner);
      mesh_.colors.push_back(color);
    }
  }

  RouteMeshParameters parameters_;
  std::vector<geometry_msgs::Point> positions_;
  visualization_msgs::Marker mesh_;
};
}  // namespace visualizer
