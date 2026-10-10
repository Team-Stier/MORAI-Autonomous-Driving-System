#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace path_planner {
struct FrenetPathParameters {
  double path_length_m = 20.0;
  double sample_step_m = 0.5;
  double reference_step_m = 2.0;
  std::vector<double> transition_lengths_m{8.0, 12.0, 16.0};
  std::vector<double> lateral_offsets_m{-0.5, 0.0, 0.5};
  double vehicle_front_m = 3.845008;
  double vehicle_rear_m = 0.790000;
  double vehicle_half_width_m = 1.080418;
};

class FrenetPath {
 public:
  FrenetPath(FrenetPathParameters parameters, SharedMap& shared)
      : parameters_(std::move(parameters)), map_(shared.map),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(*map_, *rules_)),
        visualizer_("frenet_candidates") {}

  interfaces::Path process(const nav_msgs::Odometry& ego, const interfaces::Objects& objects,
                           const Route& route) {
    const auto& p = ego.pose.pose.position;
    const auto& q = ego.pose.pose.orientation;
    const double heading = yaw(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    const Vec2 position(p.x, p.y);
    const auto current = match(position, heading, route->front());
    lanelet::ConstLanelets seeds{current};
    const auto left = graph_->left(current), right = graph_->right(current);
    if (left) seeds.push_back(*left);
    if (right) seeds.push_back(*right);
    std::vector<Reference> references;
    for (const auto& seed : seeds) {
      const auto chains = extend(seed, position);
      references.insert(references.end(), chains.begin(), chains.end());
    }
    updateRoute(*route);
    const auto obstacles = objectBoxes(objects);
    visualization_msgs::MarkerArray display;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    display.markers.push_back(clear);
    interfaces::Path selected;
    std::size_t best_progress = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t selected_marker = 0;
    for (const auto& reference : references) {
      for (double transition : parameters_.transition_lengths_m) {
        for (double offset : parameters_.lateral_offsets_m) {
          auto candidate = generate(reference, position, heading, transition, offset);
          if (candidate.points.empty()) continue;
          const bool accepted = !collides(candidate, obstacles);
          auto marker = visualize(candidate, ego.header, static_cast<int>(display.markers.size()), accepted);
          if (accepted) {
            const auto cost = routeCost(candidate);
            if (cost.first > best_progress || (cost.first == best_progress && cost.second < best_cost)) {
              best_progress = cost.first;
              best_cost = cost.second;
              selected = std::move(candidate);
              selected_marker = display.markers.size();
            }
          }
          display.markers.push_back(std::move(marker));
        }
      }
    }
    if (selected_marker != 0) {
      auto& marker = display.markers[selected_marker];
      marker.color.r = 0.1; marker.color.g = 1.0; marker.color.b = 0.1;
      marker.scale.x = 0.12;
    }
    visualizer_.publish(display);
    return selected;
  }

 private:
  using Vec2 = lanelet::BasicPoint2d;
  using Vec3 = lanelet::BasicPoint3d;
  using LaneKey = std::pair<lanelet::Id, bool>;
  struct Line {
    std::vector<Vec3> points;
    std::vector<double> stations;
    mutable std::vector<Vec3> controls;
  };
  struct LaneData {
    Line center;
  };
  struct Reference {
    Line line;
    lanelet::ConstLanelets lanes;
    double start_s = 0.0;
  };
  struct Projection {
    double station = 0.0;
    double distance_squared = std::numeric_limits<double>::infinity();
    Vec3 point = Vec3::Zero();
    double heading = 0.0;
    double metric = 1.0;
    double curvature = 0.0;
    Vec2 derivative = Vec2::Zero();
    Vec2 second_derivative = Vec2::Zero();
  };
  struct Box {
    Vec2 center;
    double heading, half_length, half_width;
  };
  struct RouteSegment {
    Vec2 start, end;
    std::size_t index;
  };

  static LaneKey key(const lanelet::ConstLanelet& lane) { return {lane.id(), lane.inverted()}; }
  static double wrap(double angle) { return std::remainder(angle, 6.283185307179586); }
  static Vec2 unit(double angle) { return {std::cos(angle), std::sin(angle)}; }
  static Vec2 normal(double angle) { return {-std::sin(angle), std::cos(angle)}; }
  static double yaw(const Eigen::Quaterniond& q) {
    return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
  }
  static double poseYaw(const geometry_msgs::Pose& pose) {
    const auto& q = pose.orientation;
    return yaw(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
  }
  static Vec2 xy(const geometry_msgs::Point& point) { return {point.x, point.y}; }
  static void append(Line& line, const Vec3& point) {
    const double distance = line.points.empty() ? 0.0 :
        std::hypot(point.x() - line.points.back().x(), point.y() - line.points.back().y());
    if (!line.points.empty() && distance == 0.0) return;
    line.stations.push_back(line.points.empty() ? 0.0 : line.stations.back() + distance);
    line.points.push_back(point);
    line.controls.clear();
  }
  const LaneData& data(const lanelet::ConstLanelet& lane) {
    const auto id = key(lane);
    const auto found = lanes_.find(id);
    if (found != lanes_.end()) return found->second;
    LaneData value;
    for (const auto& point : lane.centerline3d().basicLineString()) append(value.center, point);
    return lanes_.emplace(id, std::move(value)).first->second;
  }
  Projection project(const Line& line, const Vec2& position,
                     double limit = std::numeric_limits<double>::infinity()) const {
    Projection best;
    for (std::size_t i = 1; i < line.points.size(); ++i) {
      if (line.stations[i - 1] > limit) break;
      const Vec2 start = line.points[i - 1].head<2>();
      const Vec2 delta = line.points[i].head<2>() - start;
      const double ratio = std::max(0.0, std::min(1.0, (position - start).dot(delta) / delta.squaredNorm()));
      const double upper = std::min(line.stations[i], limit);
      double station = std::min(upper, line.stations[i - 1] + ratio * (line.stations[i] - line.stations[i - 1]));
      for (int iteration = 0; iteration < 8; ++iteration) {
        const auto value = sample(line, station);
        const Vec2 residual = value.point.head<2>() - position;
        const double denominator = value.derivative.squaredNorm() + residual.dot(value.second_derivative);
        if (std::abs(denominator) < 1e-12) break;
        const double next = std::max(line.stations[i - 1], std::min(upper,
            station - residual.dot(value.derivative) / denominator));
        if (std::abs(next - station) < 1e-8) { station = next; break; }
        station = next;
      }
      auto value = sample(line, station);
      value.distance_squared = (value.point.head<2>() - position).squaredNorm();
      if (value.distance_squared < best.distance_squared) best = value;
    }
    return best;
  }
  static Vec3 sampleLine(const Line& line, double station) {
    auto end = std::upper_bound(line.stations.begin(), line.stations.end(), station);
    const std::size_t i = std::min(line.points.size() - 1,
        std::max<std::size_t>(1, static_cast<std::size_t>(end - line.stations.begin())));
    const Vec3 delta = line.points[i] - line.points[i - 1];
    const double ratio = (station - line.stations[i - 1]) / (line.stations[i] - line.stations[i - 1]);
    return line.points[i - 1] + ratio * delta;
  }
  Projection sample(const Line& line, double station) const {
    const double spacing = parameters_.reference_step_m;
    if (line.controls.empty()) {
      const std::size_t count = static_cast<std::size_t>(std::ceil(line.stations.back() / spacing)) + 1;
      line.controls.reserve(count);
      for (std::size_t i = 0; i < count; ++i) line.controls.push_back(sampleLine(line, spacing * i));
    }
    const int index = static_cast<int>(std::floor(station / spacing));
    const double u = station / spacing - index, v = 1.0 - u;
    const auto control = [&](int i) -> Vec3 {
      if (i < 0) return line.controls.front() + i * (line.controls[1] - line.controls[0]);
      if (i >= static_cast<int>(line.controls.size()))
        return line.controls.back() + (i - static_cast<int>(line.controls.size()) + 1) *
            (line.controls.back() - line.controls[line.controls.size() - 2]);
      return line.controls[static_cast<std::size_t>(i)];
    };
    const Vec3 p0 = control(index - 1), p1 = control(index), p2 = control(index + 1), p3 = control(index + 2);
    // Uniform cubic B-spline controls smooth lanelet junction kinks without forcing the curve through each corner.
    const Vec3 point = (v*v*v*p0 + (3*u*u*u - 6*u*u + 4)*p1 +
        (-3*u*u*u + 3*u*u + 3*u + 1)*p2 + u*u*u*p3) / 6;
    const Vec3 first = (-v*v*p0 + (3*u*u - 4*u)*p1 +
        (-3*u*u + 2*u + 1)*p2 + u*u*p3) / (2 * spacing);
    const Vec3 second = (v*p0 + (3*u - 2)*p1 + (-3*u + 1)*p2 + u*p3) / (spacing * spacing);
    const double metric = first.head<2>().norm();
    const double curvature = metric > 1e-9 ? cross(first.head<2>(), second.head<2>()) / (metric*metric*metric) : 0.0;
    return {station, 0.0, point, std::atan2(first.y(), first.x()), metric, curvature,
            first.head<2>(), second.head<2>()};
  }
  lanelet::ConstLanelet match(const Vec2& position, double heading, lanelet::ConstLanelet selected) {
    auto score = [&](const lanelet::ConstLanelet& lane, double polygon_distance) {
      const auto projection = project(data(lane).center, position);
      const double heading_error = 1.0 - std::cos(heading - projection.heading);
      return std::make_tuple(heading_error >= 1.0, polygon_distance,
                            projection.distance_squared, heading_error, lane.id(), lane.inverted());
    };
    auto best = score(selected, std::sqrt(project(data(selected).center, position).distance_squared));
    for (const auto& candidate : lanelet::geometry::findNearest(map_->laneletLayer, position, 10)) {
      for (const auto& lane : {candidate.second, candidate.second.invert()}) {
        if (!rules_->canPass(lane)) continue;
        const auto rank = score(lane, candidate.first);
        if (rank < best) { best = rank; selected = lane; }
      }
    }
    return selected;
  }
  std::vector<Reference> extend(const lanelet::ConstLanelet& seed, const Vec2& position) {
    const Line first = data(seed).center;
    const double start = project(first, position).station;
    std::vector<Reference> pending{{first, {seed}, start}}, complete;
    while (!pending.empty()) {
      Reference reference = std::move(pending.back());
      pending.pop_back();
      reference.start_s = project(reference.line, position, first.stations.back()).station;
      if (reference.line.stations.back() - reference.start_s >= parameters_.path_length_m) {
        complete.push_back(std::move(reference));
        continue;
      }
      auto successors = graph_->following(reference.lanes.back(), false);
      std::sort(successors.begin(), successors.end(), [](const auto& a, const auto& b) { return key(a) < key(b); });
      bool extended = false;
      for (auto it = successors.rbegin(); it != successors.rend(); ++it) {
        Reference next = reference;
        for (const auto& point : data(*it).center.points) append(next.line, point);
        if (next.line.stations.back() <= reference.line.stations.back() + 1e-9) continue;
        next.lanes.push_back(*it);
        pending.push_back(std::move(next));
        extended = true;
      }
      if (!extended) {
        complete.push_back(std::move(reference));
      }
    }
    return complete;
  }
  interfaces::Path generate(const Reference& reference, const Vec2& position, double heading,
                            double transition, double offset) const {
    interfaces::Path path;
    const auto origin = sample(reference.line, reference.start_s);
    const double relative_yaw = wrap(heading - origin.heading);
    const double horizon = std::min(parameters_.path_length_m, reference.line.stations.back() - reference.start_s);
    if (horizon <= 0.0) return path;
    transition = std::min(transition, horizon);
    const double d0 = (position - origin.point.head<2>()).dot(normal(origin.heading));
    const double slope = std::tan(relative_yaw) * origin.metric * (1.0 - origin.curvature * d0) * transition;
    const double delta = offset - d0;
    const double a3 = 10 * delta - 6 * slope, a4 = -15 * delta + 8 * slope, a5 = 6 * delta - 3 * slope;
    const int steps = static_cast<int>(std::ceil(horizon / parameters_.sample_step_m));
    for (int i = 0; i <= steps; ++i) {
      const double progress = horizon * i / steps;
      const auto base = sample(reference.line, reference.start_s + progress);
      const double u = std::min(1.0, progress / transition);
      const double d = d0 + slope * u + u * u * u * (a3 + u * (a4 + u * a5));
      const Vec2 point = base.point.head<2>() + d * normal(base.heading);
      interfaces::PathPoint output;
      output.pose.pose.position.x = i == 0 ? position.x() : point.x();
      output.pose.pose.position.y = i == 0 ? position.y() : point.y();
      output.pose.pose.position.z = base.point.z();
      output.velocity = std::numeric_limits<double>::quiet_NaN();
      path.points.push_back(output);
    }
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const std::size_t before = i == 0 ? 0 : i - 1, after = std::min(i + 1, path.points.size() - 1);
      const Vec2 direction = xy(path.points[after].pose.pose.position) - xy(path.points[before].pose.pose.position);
      const double angle = i == 0 ? heading : std::atan2(direction.y(), direction.x());
      auto& orientation = path.points[i].pose.pose.orientation;
      orientation.z = std::sin(angle / 2); orientation.w = std::cos(angle / 2);
    }
    return path;
  }
  static double cross(const Vec2& a, const Vec2& b) { return a.x() * b.y() - a.y() * b.x(); }
  Box egoBox(const geometry_msgs::Pose& pose) const {
    const double heading = poseYaw(pose);
    return {xy(pose.position) + ((parameters_.vehicle_front_m - parameters_.vehicle_rear_m) / 2) * unit(heading),
            heading, (parameters_.vehicle_front_m + parameters_.vehicle_rear_m) / 2, parameters_.vehicle_half_width_m};
  }
  std::vector<Box> objectBoxes(const interfaces::Objects& objects) const {
    std::vector<Box> boxes;
    for (const auto& object : objects.objects) {
      boxes.push_back({xy(object.bbox_pose.position), poseYaw(object.bbox_pose),
                       object.bbox_size.x / 2, object.bbox_size.y / 2});
    }
    return boxes;
  }
  static double extent(const Box& box, const Vec2& axis) {
    return box.half_length * std::abs(unit(box.heading).dot(axis)) +
           box.half_width * std::abs(normal(box.heading).dot(axis));
  }
  static bool overlaps(const Box& a, const Box& b) {
    const Vec2 difference = b.center - a.center;
    for (const auto& axis : {unit(a.heading), normal(a.heading), unit(b.heading), normal(b.heading)})
      if (std::abs(difference.dot(axis)) > extent(a, axis) + extent(b, axis) + 1e-9) return false;
    return true;
  }
  bool collides(const interfaces::Path& path, const std::vector<Box>& obstacles) const {
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto first = egoBox(path.points[i == 0 ? 0 : i - 1].pose.pose), last = egoBox(path.points[i].pose.pose);
      const Box sweep = sweepBox(first, last);
      for (const auto& obstacle : obstacles) if (overlaps(sweep, obstacle)) return true;
    }
    return false;
  }
  Box sweepBox(const Box& first, const Box& last) const {
    const double angle = wrap(last.heading - first.heading), heading = first.heading + angle / 2;
    const Vec2 delta = last.center - first.center;
    // Enclose translation and rotation throughout each segment, including the rear-axle offset arc.
    const double pad = std::hypot(first.half_length, first.half_width) * std::abs(angle) / 2 +
        std::abs(parameters_.vehicle_front_m - parameters_.vehicle_rear_m) * angle * angle / 16;
    return {(first.center + last.center) / 2, heading,
            first.half_length + std::abs(delta.dot(unit(heading))) / 2 + pad,
            first.half_width + std::abs(delta.dot(normal(heading))) / 2 + pad};
  }
  void updateRoute(const lanelet::routing::LaneletPath& route) {
    std::vector<LaneKey> ids;
    for (const auto& lane : route) ids.push_back(key(lane));
    if (ids == route_ids_) return;
    route_ids_ = std::move(ids);
    route_segments_.clear();
    std::size_t index = 0;
    for (const auto& lane : route) {
      const auto& points = data(lane).center.points;
      for (std::size_t i = 1; i < points.size(); ++i)
        route_segments_.push_back({points[i - 1].head<2>(), points[i].head<2>(), index});
      ++index;
    }
  }
  std::pair<std::size_t, double> routeCost(const interfaces::Path& path) const {
    double cost = 0.0;
    std::size_t endpoint_index = 0;
    for (const auto& point : path.points) {
      const Vec2 position = xy(point.pose.pose.position);
      double nearest = std::numeric_limits<double>::infinity();
      std::size_t nearest_index = 0;
      for (const auto& segment : route_segments_) {
        const Vec2 delta = segment.end - segment.start;
        const double u = std::max(0.0, std::min(1.0, (position - segment.start).dot(delta) / delta.squaredNorm()));
        const double distance = (position - segment.start - u * delta).squaredNorm();
        if (distance < nearest || (distance == nearest && segment.index > nearest_index)) {
          nearest = distance;
          nearest_index = segment.index;
        }
      }
      cost += nearest;
      endpoint_index = nearest_index;
    }
    return {endpoint_index, cost / path.points.size()};
  }
  static visualization_msgs::Marker visualize(const interfaces::Path& path, const std_msgs::Header& header,
                                               int id, bool accepted) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "frenet_candidates";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = accepted ? 0.1 : 1.0;
    marker.color.g = accepted ? 0.6 : 0.1;
    marker.color.b = accepted ? 1.0 : 0.1;
    marker.color.a = 0.7;
    for (const auto& point : path.points) marker.points.push_back(point.pose.pose.position);
    return marker;
  }

  FrenetPathParameters parameters_;
  std::shared_ptr<lanelet::LaneletMap> map_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  std::map<LaneKey, LaneData> lanes_;
  std::vector<LaneKey> route_ids_;
  std::vector<RouteSegment> route_segments_;
  node_pipeline::Visualizer<visualization_msgs::MarkerArray> visualizer_;
};
}  // namespace path_planner
