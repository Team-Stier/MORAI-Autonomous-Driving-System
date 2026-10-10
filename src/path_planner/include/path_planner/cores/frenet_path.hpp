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
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <algorithm>
#include <array>
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
  std::vector<double> transition_lengths_m{8.0, 12.0, 16.0};
  std::vector<double> lateral_offsets_m{-0.5, 0.0, 0.5};
  double vehicle_front_m = 3.845008;
  double vehicle_rear_m = 0.790000;
  double vehicle_half_width_m = 1.080418;
  double obstacle_margin_m = 0.2;
  double max_curvature_inv_m = 0.2797;
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
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const Vec2 position(p.x, p.y);
    const double heading = yaw(rotation);
    const auto current = match(position, heading, route->front());
    lanelet::ConstLanelets seeds{current};
    const auto left = graph_->left(current), right = graph_->right(current);
    if (left) seeds.push_back(*left);
    if (right) seeds.push_back(*right);
    std::vector<Reference> references;
    lanelet::ConstLanelets corridor;
    for (const auto& seed : seeds) {
      const auto chains = extend(seed, position);
      for (const auto& reference : chains) {
        references.push_back(reference);
        for (const auto& lane : reference.lanes)
          if (std::find(corridor.begin(), corridor.end(), lane) == corridor.end()) corridor.push_back(lane);
      }
      includePreviousRoad(seed, position, corridor);
    }
    updateRoad(corridor);
    updateRoute(*route);
    const auto obstacles = objectBoxes(objects, rotation, Vec3(p.x, p.y, p.z));
    visualization_msgs::MarkerArray display;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    display.markers.push_back(clear);
    interfaces::Path selected;
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t selected_marker = 0;
    for (const auto& reference : references) {
      for (double transition : parameters_.transition_lengths_m) {
        for (double offset : parameters_.lateral_offsets_m) {
          auto candidate = generate(reference, position, heading, transition, offset);
          if (candidate.points.empty()) continue;
          const bool drivable = onRoad(candidate, current, corridor);
          const bool collision = collides(candidate, obstacles);
          const bool accepted = drivable && !collision;
          auto marker = visualize(candidate, ego.header, static_cast<int>(display.markers.size()), accepted);
          const double cost = accepted ? routeCost(candidate) : std::numeric_limits<double>::infinity();
          if (cost < best_cost) {
            best_cost = cost;
            selected = std::move(candidate);
            selected_marker = display.markers.size();
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
  using RoadPolygon = boost::geometry::model::polygon<Vec2>;
  using Road = boost::geometry::model::multi_polygon<RoadPolygon>;
  struct Line {
    std::vector<Vec3> points;
    std::vector<double> stations;
  };
  struct LaneData {
    Line center;
    std::vector<Vec2> polygon;
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
  };
  struct Box {
    Vec2 center;
    double heading, half_length, half_width;
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
  }
  const LaneData& data(const lanelet::ConstLanelet& lane) {
    const auto id = key(lane);
    const auto found = lanes_.find(id);
    if (found != lanes_.end()) return found->second;
    LaneData value;
    for (const auto& point : lane.centerline3d().basicLineString()) append(value.center, point);
    for (const auto& point : lane.leftBound3d()) value.polygon.emplace_back(point.x(), point.y());
    const auto right = lane.rightBound3d();
    for (std::size_t i = right.size(); i > 0; --i) value.polygon.emplace_back(right[i - 1].x(), right[i - 1].y());
    return lanes_.emplace(id, std::move(value)).first->second;
  }
  static Projection project(const Line& line, const Vec2& position) {
    Projection best;
    for (std::size_t i = 1; i < line.points.size(); ++i) {
      const Vec2 start = line.points[i - 1].head<2>();
      const Vec2 delta = line.points[i].head<2>() - start;
      const double ratio = std::max(0.0, std::min(1.0, (position - start).dot(delta) / delta.squaredNorm()));
      const Vec3 point = line.points[i - 1] + ratio * (line.points[i] - line.points[i - 1]);
      const double distance = (point.head<2>() - position).squaredNorm();
      if (distance < best.distance_squared)
        best = {line.stations[i - 1] + ratio * (line.stations[i] - line.stations[i - 1]),
                distance, point, std::atan2(delta.y(), delta.x())};
    }
    return best;
  }
  static Projection sample(const Line& line, double station) {
    auto end = std::upper_bound(line.stations.begin(), line.stations.end(), station);
    const std::size_t i = std::min(line.points.size() - 1,
        std::max<std::size_t>(1, static_cast<std::size_t>(end - line.stations.begin())));
    const Vec3 delta = line.points[i] - line.points[i - 1];
    const double ratio = (station - line.stations[i - 1]) / (line.stations[i] - line.stations[i - 1]);
    return {station, 0.0, line.points[i - 1] + ratio * delta, std::atan2(delta.y(), delta.x())};
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
    const double coverage = start + parameters_.path_length_m +
                            std::hypot(parameters_.vehicle_front_m, parameters_.vehicle_half_width_m);
    std::vector<Reference> pending{{first, {seed}, start}}, complete;
    while (!pending.empty()) {
      Reference reference = std::move(pending.back());
      pending.pop_back();
      if (reference.line.stations.back() >= coverage) {
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
      if (!extended) complete.push_back(std::move(reference));
    }
    return complete;
  }
  void includePreviousRoad(const lanelet::ConstLanelet& seed, const Vec2& position,
                           lanelet::ConstLanelets& corridor) {
    const double remaining = std::hypot(parameters_.vehicle_rear_m, parameters_.vehicle_half_width_m) -
                             project(data(seed).center, position).station;
    std::vector<std::pair<lanelet::ConstLanelet, double>> pending{{seed, remaining}};
    std::map<LaneKey, double> explored;
    while (!pending.empty()) {
      const auto item = pending.back();
      pending.pop_back();
      if (item.second <= 0.0) continue;
      const auto visited = explored.find(key(item.first));
      if (visited != explored.end() && visited->second >= item.second) continue;
      explored[key(item.first)] = item.second;
      // A reference starts at the current lane, but the rear bumper can still occupy earlier lanes.
      for (const auto& lane : graph_->previous(item.first, false)) {
        if (std::find(corridor.begin(), corridor.end(), lane) == corridor.end()) corridor.push_back(lane);
        pending.emplace_back(lane, item.second - data(lane).center.stations.back());
      }
    }
  }
  interfaces::Path generate(const Reference& reference, const Vec2& position, double heading,
                            double transition, double offset) const {
    interfaces::Path path;
    const auto origin = sample(reference.line, reference.start_s);
    const double relative_yaw = wrap(heading - origin.heading);
    if (std::cos(relative_yaw) <= 0.05) return path;
    const double horizon = std::min(parameters_.path_length_m, reference.line.stations.back() - reference.start_s -
        std::hypot(parameters_.vehicle_front_m, parameters_.vehicle_half_width_m));
    if (horizon < parameters_.sample_step_m) return path;
    transition = std::min(transition, horizon);
    const double d0 = (position - origin.point.head<2>()).dot(normal(origin.heading));
    const double slope = std::tan(relative_yaw) * transition;
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
      if (i == 0) continue;
      const Vec2 step = xy(path.points[i].pose.pose.position) - xy(path.points[i - 1].pose.pose.position);
      if (step.dot(unit(poseYaw(path.points[i - 1].pose.pose))) <= 0.0 ||
          std::abs(wrap(angle - poseYaw(path.points[i - 1].pose.pose))) > parameters_.max_curvature_inv_m * step.norm())
        return {};
    }
    return path;
  }
  static bool contains(const std::vector<Vec2>& polygon, const Vec2& point) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const Vec2 a = polygon[j], b = polygon[i], delta = b - a;
      if (point.x() >= std::min(a.x(), b.x()) - 1e-9 && point.x() <= std::max(a.x(), b.x()) + 1e-9 &&
          point.y() >= std::min(a.y(), b.y()) - 1e-9 && point.y() <= std::max(a.y(), b.y()) + 1e-9 &&
          std::abs(cross(point - a, delta)) <= 1e-9 * delta.norm()) return true;
      if ((a.y() > point.y()) != (b.y() > point.y()) &&
          point.x() < a.x() + (point.y() - a.y()) * delta.x() / delta.y()) inside = !inside;
    }
    return inside;
  }
  static double cross(const Vec2& a, const Vec2& b) { return a.x() * b.y() - a.y() * b.x(); }
  bool connected(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to) const {
    if (from == to) return true;
    const auto following = graph_->following(from, false);
    if (std::find(following.begin(), following.end(), to) != following.end()) return true;
    const auto left = graph_->left(from), right = graph_->right(from);
    return (left && *left == to) || (right && *right == to);
  }
  bool onRoad(const interfaces::Path& path, const lanelet::ConstLanelet& current,
              const lanelet::ConstLanelets& corridor) {
    lanelet::ConstLanelets active{current};
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto& pose = path.points[i].pose.pose;
      const auto first = egoBox(path.points[i == 0 ? 0 : i - 1].pose.pose), last = egoBox(pose);
      if (!boost::geometry::covered_by(boxPolygon(sweepBox(first, last)), road_)) return false;
      if (i == 0) continue;
      const Vec2 begin = xy(path.points[i - 1].pose.pose.position), end = xy(pose.position), delta = end - begin;
      std::vector<double> cuts{0.0, 1.0};
      for (const auto& lane : corridor) {
        const auto& polygon = data(lane).polygon;
        for (std::size_t k = 0, j = polygon.size() - 1; k < polygon.size(); j = k++) {
          const Vec2 edge = polygon[k] - polygon[j];
          const double denominator = cross(delta, edge);
          if (std::abs(denominator) < 1e-12) continue;
          const double u = cross(polygon[j] - begin, edge) / denominator;
          const double v = cross(polygon[j] - begin, delta) / denominator;
          if (u > 0.0 && u < 1.0 && v >= 0.0 && v <= 1.0) cuts.push_back(u);
        }
      }
      std::sort(cuts.begin(), cuts.end());
      for (std::size_t k = 1; k < cuts.size(); ++k) {
        if (cuts[k] - cuts[k - 1] < 1e-9) continue;
        const Vec2 middle = begin + ((cuts[k] + cuts[k - 1]) / 2) * delta;
        lanelet::ConstLanelets next;
        for (const auto& lane : corridor) {
          if (contains(data(lane).polygon, middle) &&
              std::any_of(active.begin(), active.end(), [&](const auto& from) { return connected(from, lane); }))
            next.push_back(lane);
        }
        if (next.empty()) return false;
        active = std::move(next);
      }
    }
    return true;
  }
  Box egoBox(const geometry_msgs::Pose& pose) const {
    const double heading = poseYaw(pose);
    return {xy(pose.position) + ((parameters_.vehicle_front_m - parameters_.vehicle_rear_m) / 2) * unit(heading),
            heading, (parameters_.vehicle_front_m + parameters_.vehicle_rear_m) / 2, parameters_.vehicle_half_width_m};
  }
  static RoadPolygon boxPolygon(const Box& box) {
    RoadPolygon polygon;
    const Vec2 x = box.half_length * unit(box.heading), y = box.half_width * normal(box.heading);
    polygon.outer() = {box.center + x + y, box.center + x - y,
                       box.center - x - y, box.center - x + y, box.center + x + y};
    return polygon;
  }
  void updateRoad(const lanelet::ConstLanelets& corridor) {
    std::vector<LaneKey> ids;
    for (const auto& lane : corridor) ids.push_back(key(lane));
    std::sort(ids.begin(), ids.end());
    if (ids == road_ids_) return;
    road_ids_ = std::move(ids);
    road_.clear();
    for (const auto& lane : corridor) {
      RoadPolygon polygon;
      for (const auto& point : data(lane).polygon) polygon.outer().push_back(point);
      boost::geometry::correct(polygon);
      if (road_.empty()) road_.push_back(std::move(polygon));
      else {
        Road united;
        boost::geometry::union_(road_, polygon, united);
        road_ = std::move(united);
      }
    }
  }
  std::vector<Box> objectBoxes(const interfaces::Objects& objects, const Eigen::Quaterniond& rotation,
                               const Vec3& origin) const {
    std::vector<Box> boxes;
    for (const auto& object : objects.objects) {
      const auto& p = object.bbox_pose.position;
      const auto& q = object.bbox_pose.orientation;
      const Vec3 center = origin + rotation * Vec3(p.x, p.y, p.z);
      const Eigen::Matrix3d axes = (rotation * Eigen::Quaterniond(q.w, q.x, q.y, q.z)).toRotationMatrix();
      const double heading = std::atan2(axes(1, 0), axes(0, 0));
      double length = 0.0, width = 0.0;
      // Project all three box axes so roll/pitch cannot shrink the collision footprint.
      for (int i = 0; i < 3; ++i) {
        const double half_size = (i == 0 ? object.bbox_size.x : i == 1 ? object.bbox_size.y : object.bbox_size.z) / 2;
        length += half_size * std::abs(axes.col(i).head<2>().dot(unit(heading)));
        width += half_size * std::abs(axes.col(i).head<2>().dot(normal(heading)));
      }
      boxes.push_back({center.head<2>(), heading, length + parameters_.obstacle_margin_m,
                       width + parameters_.obstacle_margin_m});
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
    for (const auto& lane : route) {
      const auto& points = data(lane).center.points;
      for (std::size_t i = 1; i < points.size(); ++i)
        route_segments_.emplace_back(points[i - 1].head<2>(), points[i].head<2>());
    }
  }
  double routeCost(const interfaces::Path& path) const {
    double cost = 0.0;
    for (const auto& point : path.points) {
      const Vec2 position = xy(point.pose.pose.position);
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto& segment : route_segments_) {
        const Vec2 delta = segment.second - segment.first;
        const double u = std::max(0.0, std::min(1.0, (position - segment.first).dot(delta) / delta.squaredNorm()));
        nearest = std::min(nearest, (position - segment.first - u * delta).squaredNorm());
      }
      cost += nearest;
    }
    return cost / path.points.size();
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
  std::vector<LaneKey> road_ids_;
  Road road_;
  std::vector<LaneKey> route_ids_;
  std::vector<std::pair<Vec2, Vec2>> route_segments_;
  node_pipeline::Visualizer<visualization_msgs::MarkerArray> visualizer_;
};
}  // namespace path_planner
