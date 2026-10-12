#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
namespace path_planner {
struct HybridAstarParameters {
  double path_length_m = 50.0, sample_step_m = 0.25, primitive_length_m = 1.0;
  double grid_resolution_m = 0.5;
  int yaw_bins = 72, max_expansions = 800;
  double max_steer_rad = 0.698132, wheelbase_m = 3.0;
  double vehicle_front_m = 3.845008, vehicle_rear_m = 0.79, vehicle_half_width_m = 1.080418;
  double collision_margin_m = 0.3, road_margin_m = 0.05;
  std::vector<double> transition_lengths_m{8.0, 12.0, 16.0};
  std::vector<double> steering_candidates_deg{-40, -35, -30, -25, -20, -15, -10, -5,
                                               0, 5, 10, 15, 20, 25, 30, 35, 40};
};
class HybridAstar {
 public:
  HybridAstar(HybridAstarParameters parameters, SharedMap& shared)
      : parameters_(std::move(parameters)), map_(shared.map),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(*map_, *rules_)) {}

  interfaces::Path process(const nav_msgs::Odometry& ego, const interfaces::Objects& objects,
                           const Route& route, const PredictedObjects& predictions) {
    const auto& p = ego.pose.pose.position;
    const State start{{p.x, p.y, p.z}, poseYaw(ego.pose.pose), 0.0};
    const auto current = match(start, *route);
    lanelet::ConstLanelets seeds{current};
    const auto left = graph_->left(current), right = graph_->right(current);
    if (left) seeds.push_back(*left);
    if (right) seeds.push_back(*right);
    std::vector<Reference> references;
    for (const auto& seed : seeds) extend(seed, start, *route, references);
    const auto road = corridor(references, current);
    const auto obstacles = occupancy(objects, predictions);
    if (!clear(start, start, road, obstacles)) return {};
    updateRoute(*route);
    interfaces::Path best;
    double best_progress = -1.0, best_cost = std::numeric_limits<double>::infinity();
    std::size_t selected_reference = 0;
    bool full = false;
    for (std::size_t i = 0; i < references.size(); ++i) {
      auto& ref = references[i];
      fit(ref.line);
      ref.start_s = project(ref.line, start.position.head<2>()).station;
      for (double transition : parameters_.transition_lengths_m) {
        const auto candidate = connect(start, ref, transition, parameters_.path_length_m, road, obstacles);
        if (candidate.points.empty()) continue;
        const double length = pathLength(candidate);
        const double progress = project(ref.line, xy(candidate.points.back())).station - ref.start_s;
        const double cost = routeCost(candidate) + 0.001 * transition;
        const bool complete = length >= parameters_.path_length_m - 0.01;
        if ((complete && !full) || (complete == full &&
            (complete ? cost < best_cost : progress > best_progress + 1e-6 ||
                (std::abs(progress - best_progress) <= 1e-6 && cost < best_cost)))) {
          best = candidate; best_progress = progress; best_cost = cost;
          selected_reference = i; full = complete;
        }
      }
    }
    if (full) return best;
    // shortcut: use the preferred reference for bounded Hybrid A*; search other branches if mission routing needs it.
    const auto& reference = references[selected_reference];
    return search(start, reference, road, obstacles, std::move(best), best_progress);
  }

 private:
  using Vec2 = Eigen::Vector2d;
  using Vec3 = Eigen::Vector3d;
  using LaneKey = std::pair<lanelet::Id, bool>;
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Polygon = boost::geometry::model::polygon<Point>;
  using Polygons = boost::geometry::model::multi_polygon<Polygon>;
  struct Road {
    Polygons polygons;
    std::vector<std::pair<Vec2, Vec2>> boundary;
  };
  struct Line {
    std::vector<Vec3> points, second;
    std::vector<double> stations;
  };
  struct Reference { Line line; lanelet::ConstLanelets lanes; double start_s = 0.0; };
  struct Sample { Vec3 position, first, second; };
  struct Projection { double station = 0.0, distance_squared = std::numeric_limits<double>::infinity(); };
  struct RouteLine { Line line; Vec2 lower, upper; };
  struct State { Vec3 position; double heading, steer; };
  struct Box { Vec2 center; double heading, half_length, half_width; };
  struct Node {
    State state;
    double length = 0.0, cost = 0.0, progress = 0.0;
    int parent = -1, steering = 0;
    std::vector<State> edge;
  };
  using Cell = std::tuple<int, int, int, int>;

  static double wrap(double angle) { return std::remainder(angle, 6.283185307179586); }
  static Vec2 unit(double angle) { return {std::cos(angle), std::sin(angle)}; }
  static Vec2 normal(double angle) { return {-std::sin(angle), std::cos(angle)}; }
  static LaneKey key(const lanelet::ConstLanelet& lane) { return {lane.id(), lane.inverted()}; }
  static Vec2 xy(const interfaces::PathPoint& point) {
    return {point.pose.pose.position.x, point.pose.pose.position.y};
  }
  static double poseYaw(const geometry_msgs::Pose& pose) {
    const auto& q = pose.orientation;
    return std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
  }
  static double cross(const Vec2& a, const Vec2& b) { return a.x()*b.y() - a.y()*b.x(); }
  static void append(Line& line, const Vec3& point) {
    const double distance = line.points.empty() ? 0.0 : (point.head<2>() - line.points.back().head<2>()).norm();
    if (!line.points.empty() && distance < 1e-8) return;
    line.stations.push_back(line.points.empty() ? 0.0 : line.stations.back() + distance);
    line.points.push_back(point);
  }
  static Line center(const lanelet::ConstLanelet& lane) {
    Line line;
    for (const auto& point : lane.centerline3d()) append(line, {point.x(), point.y(), point.z()});
    return line;
  }
  static void fit(Line& line) {
    const auto count = line.points.size();
    line.second.assign(count, Vec3::Zero());
    std::vector<double> upper(count, 0.0);
    for (std::size_t i = 1; i + 1 < count; ++i) {
      const double a = line.stations[i] - line.stations[i - 1];
      const double b = line.stations[i + 1] - line.stations[i];
      const double denominator = 2*(a + b) - a*upper[i - 1];
      upper[i] = b / denominator;
      const Vec3 rhs = 6*((line.points[i + 1] - line.points[i])/b -
                         (line.points[i] - line.points[i - 1])/a);
      line.second[i] = (rhs - a*line.second[i - 1]) / denominator;
    }
    for (std::size_t i = count - 2; i > 0; --i) line.second[i] -= upper[i]*line.second[i + 1];
    if (std::all_of(line.second.begin(), line.second.end(), [](const Vec3& v) { return v.isZero(0.0); }))
      line.second.clear();
  }
  static Sample sample(const Line& line, double station) {
    station = std::max(0.0, std::min(station, line.stations.back()));
    const auto end = std::upper_bound(line.stations.begin(), line.stations.end(), station);
    const std::size_t i = std::min(line.points.size() - 1,
        std::max<std::size_t>(1, static_cast<std::size_t>(end - line.stations.begin())));
    const double h = line.stations[i] - line.stations[i - 1];
    const double b = (station - line.stations[i - 1]) / h, a = 1 - b;
    const Vec3 m0 = line.second.empty() ? Vec3::Zero() : line.second[i - 1];
    const Vec3 m1 = line.second.empty() ? Vec3::Zero() : line.second[i];
    return {a*line.points[i - 1] + b*line.points[i] +
                ((a*a*a - a)*m0 + (b*b*b - b)*m1)*h*h/6,
            (line.points[i] - line.points[i - 1])/h +
                ((1 - 3*a*a)*m0 + (3*b*b - 1)*m1)*h/6,
            a*m0 + b*m1};
  }
  static Projection project(const Line& line, const Vec2& position) {
    Projection best;
    std::size_t closest = 1;
    for (std::size_t i = 1; i < line.points.size(); ++i) {
      const Vec2 delta = line.points[i].head<2>() - line.points[i - 1].head<2>();
      const double ratio = std::max(0.0, std::min(1.0,
          (position - line.points[i - 1].head<2>()).dot(delta) / delta.squaredNorm()));
      const double distance = (line.points[i - 1].head<2>() + ratio*delta - position).squaredNorm();
      if (distance < best.distance_squared) {
        best = {line.stations[i - 1] + ratio*(line.stations[i] - line.stations[i - 1]), distance};
        closest = i;
      }
    }
    if (line.second.empty()) return best;
    const auto refine = [&](std::size_t i, double station) {
      for (int iteration = 0; iteration < 4; ++iteration) {
        const auto value = sample(line, station);
        const Vec2 residual = value.position.head<2>() - position;
        const double denominator = value.first.head<2>().squaredNorm() + residual.dot(value.second.head<2>());
        if (std::abs(denominator) < 1e-9) break;
        station = std::max(line.stations[i - 1], std::min(line.stations[i],
            station - residual.dot(value.first.head<2>())/denominator));
      }
      const double distance = (sample(line, station).position.head<2>() - position).squaredNorm();
      if (distance < best.distance_squared) best = {station, distance};
    };
    const double seed = best.station;
    best.distance_squared = std::numeric_limits<double>::infinity();
    refine(closest, seed);
    for (std::size_t i = 1; i < line.points.size(); ++i) {
      if (i == closest) continue;
      const Vec2 delta = line.points[i].head<2>() - line.points[i - 1].head<2>();
      const double ratio = std::max(0.0, std::min(1.0,
          (position - line.points[i - 1].head<2>()).dot(delta)/delta.squaredNorm()));
      const double h = line.stations[i] - line.stations[i - 1];
      const double distance = (line.points[i - 1].head<2>() + ratio*delta - position).norm();
      // Bound the cubic's displacement from its chord before discarding a possible closer segment.
      const double deviation = h*h*(line.second[i - 1].head<2>().norm() + line.second[i].head<2>().norm())/6;
      if (distance > std::sqrt(best.distance_squared) + deviation) continue;
      refine(i, line.stations[i - 1] + ratio*h);
    }
    return best;
  }
  lanelet::ConstLanelet match(const State& start, const lanelet::routing::LaneletPath& route) const {
    const auto score = [&](const lanelet::ConstLanelet& lane) {
      const auto line = center(lane);
      const auto projection = project(line, start.position.head<2>());
      const auto tangent = sample(line, projection.station).first;
      const double alignment = 1 - std::cos(std::atan2(tangent.y(), tangent.x()) - start.heading);
      return std::make_tuple(alignment >= 1.0, projection.distance_squared, alignment,
                            projection.station >= line.stations.back() - 1e-6);
    };
    auto selected = route.front();
    auto best = score(selected);
    for (const auto& lane : route) {
      const auto rank = score(lane);
      if (rank < best) { selected = lane; best = rank; }
    }
    return selected;
  }
  void extend(const lanelet::ConstLanelet& seed, const State& start,
              const lanelet::routing::LaneletPath& route, std::vector<Reference>& result) const {
    auto line = center(seed);
    const double origin = project(line, start.position.head<2>()).station;
    std::vector<Reference> pending{{std::move(line), {seed}, origin}};
    // shortcut: enumerate at most 32 lane chains; increase this bound for dense branching maps.
    const std::size_t branch_limit = 32;
    while (!pending.empty() && result.size() < branch_limit) {
      auto ref = std::move(pending.back()); pending.pop_back();
      if (ref.line.stations.back() - origin >= parameters_.path_length_m + parameters_.vehicle_front_m) {
        result.push_back(std::move(ref)); continue;
      }
      auto successors = graph_->following(ref.lanes.back(), false);
      std::sort(successors.begin(), successors.end(), [&](const auto& a, const auto& b) {
        const bool ar = std::find(route.begin(), route.end(), a) != route.end();
        const bool br = std::find(route.begin(), route.end(), b) != route.end();
        return std::make_pair(!ar, key(a)) < std::make_pair(!br, key(b));
      });
      bool added = false;
      for (auto it = successors.rbegin(); it != successors.rend(); ++it) {
        if (std::find(ref.lanes.begin(), ref.lanes.end(), *it) != ref.lanes.end()) continue;
        if (pending.size() + result.size() >= branch_limit) break;
        Reference next = ref;
        for (const auto& point : it->centerline3d()) append(next.line, {point.x(), point.y(), point.z()});
        if (next.line.stations.back() <= ref.line.stations.back() + 1e-8) continue;
        next.lanes.push_back(*it); pending.push_back(std::move(next)); added = true;
      }
      if (!added) result.push_back(std::move(ref));
    }
  }
  Road corridor(const std::vector<Reference>& references, const lanelet::ConstLanelet& current) const {
    std::map<LaneKey, lanelet::ConstLanelet> lanes;
    for (const auto& ref : references) for (const auto& lane : ref.lanes) lanes.emplace(key(lane), lane);
    for (const auto& lane : graph_->previous(current, false)) lanes.emplace(key(lane), lane);
    Road road;
    for (const auto& entry : lanes) {
      Polygon polygon;
      for (const auto& p : entry.second.polygon2d()) polygon.outer().emplace_back(p.x(), p.y());
      boost::geometry::correct(polygon);
      if (road.polygons.empty()) { road.polygons.push_back(std::move(polygon)); continue; }
      Polygons joined;
      boost::geometry::union_(road.polygons, polygon, joined);
      road.polygons = std::move(joined);
    }
    const auto edges = [&](const auto& ring) {
      for (std::size_t i = 1; i < ring.size(); ++i)
        road.boundary.push_back({{ring[i - 1].x(), ring[i - 1].y()}, {ring[i].x(), ring[i].y()}});
    };
    for (const auto& polygon : road.polygons) {
      edges(polygon.outer());
      for (const auto& hole : polygon.inners()) edges(hole);
    }
    return road;
  }
  static Box objectBox(const interfaces::Object& object) {
    return {{object.bbox_pose.position.x, object.bbox_pose.position.y}, poseYaw(object.bbox_pose),
            object.bbox_size.x/2, object.bbox_size.y/2};
  }
  static Box sweptBox(const Box& a, const Box& b, double offset = 0.0) {
    const double angle = wrap(b.heading - a.heading), heading = a.heading + angle/2;
    const Vec2 forward = unit(heading), side(-forward.y(), forward.x());
    const Vec2 delta = b.center - a.center;
    const double radius = std::max(std::hypot(a.half_length, a.half_width),
                                   std::hypot(b.half_length, b.half_width));
    const double pad = radius*std::abs(angle)/2 + offset*angle*angle/8 + delta.norm()*std::abs(angle)/8;
    return {(a.center + b.center)/2, heading,
        std::max(a.half_length, b.half_length) + std::abs(delta.dot(forward))/2 + pad,
        std::max(a.half_width, b.half_width) + std::abs(delta.dot(side))/2 + pad};
  }
  static std::vector<Box> occupancy(const interfaces::Objects& objects, const PredictedObjects& predictions) {
    std::vector<Box> result;
    std::set<std::tuple<double, double, double, double, double>> seen;
    const auto add = [&](const Box& box) {
      if (seen.emplace(box.center.x(), box.center.y(), box.heading, box.half_length, box.half_width).second)
        result.push_back(box);
    };
    for (const auto& object : objects.objects) add(objectBox(object));
    const interfaces::Objects* previous = &objects;
    // Geometry-only planning reserves the entire predicted sweep; no arrival speed is assumed.
    for (const auto& prediction : predictions) {
      for (const auto& object : prediction.objects) {
        const auto box = objectBox(object);
        add(box);
        const auto from = std::find_if(previous->objects.begin(), previous->objects.end(),
            [&](const auto& candidate) { return candidate.id == object.id; });
        if (from != previous->objects.end()) add(sweptBox(objectBox(*from), box));
      }
      previous = &prediction;
    }
    return result;
  }
  Box egoBox(const State& state, double margin) const {
    const double offset = (parameters_.vehicle_front_m - parameters_.vehicle_rear_m)/2;
    return {state.position.head<2>() + offset*unit(state.heading), state.heading,
            (parameters_.vehicle_front_m + parameters_.vehicle_rear_m)/2 + margin,
            parameters_.vehicle_half_width_m + margin};
  }
  static bool boundaryInside(const Box& box, const std::pair<Vec2, Vec2>& edge,
                             const Vec2& direction, const Vec2& side) {
    const Vec2 delta = edge.second - edge.first, from = edge.first - box.center;
    double low = 0.0, high = 1.0;
    for (const auto& axis : {direction, side}) {
      const double extent = (axis == direction ? box.half_length : box.half_width) - 1e-8;
      const double origin = from.dot(axis), step = delta.dot(axis);
      if (std::abs(step) < 1e-12) {
        if (std::abs(origin) >= extent) return false;
        continue;
      }
      const double a = (-extent - origin)/step, b = (extent - origin)/step;
      low = std::max(low, std::min(a, b)); high = std::min(high, std::max(a, b));
      if (low > high) return false;
    }
    return high > low + 1e-12;
  }
  static bool overlaps(const Box& a, const Box& b) {
    const Vec2 delta = b.center - a.center;
    const Vec2 ax = unit(a.heading), ay(-ax.y(), ax.x());
    const Vec2 bx = unit(b.heading), by(-bx.y(), bx.x());
    for (const auto& axis : {ax, ay, bx, by}) {
      if (std::abs(delta.dot(axis)) > a.half_length*std::abs(ax.dot(axis)) +
          a.half_width*std::abs(ay.dot(axis)) + b.half_length*std::abs(bx.dot(axis)) +
          b.half_width*std::abs(by.dot(axis)) + 1e-8) return false;
    }
    return true;
  }
  bool clear(const State& from, const State& to, const Road& road, const std::vector<Box>& obstacles) const {
    const double offset = std::abs(parameters_.vehicle_front_m - parameters_.vehicle_rear_m)/2;
    const auto sweep = sweptBox(egoBox(from, parameters_.collision_margin_m),
                                egoBox(to, parameters_.collision_margin_m), offset);
    for (const auto& obstacle : obstacles) if (overlaps(sweep, obstacle)) return false;
    const auto road_sweep = sweptBox(egoBox(from, parameters_.road_margin_m),
                                    egoBox(to, parameters_.road_margin_m), offset);
    if (!boost::geometry::covered_by(Point(road_sweep.center.x(), road_sweep.center.y()), road.polygons)) return false;
    // A connected rectangle is inside the road iff its center is inside and no outer/hole boundary enters it.
    const Vec2 forward = unit(road_sweep.heading), side(-forward.y(), forward.x());
    for (const auto& edge : road.boundary) if (boundaryInside(road_sweep, edge, forward, side)) return false;
    return true;
  }
  static interfaces::PathPoint output(const State& state) {
    interfaces::PathPoint point;
    point.pose.pose.position.x = state.position.x(); point.pose.pose.position.y = state.position.y();
    point.pose.pose.position.z = state.position.z();
    point.pose.pose.orientation.z = std::sin(state.heading/2);
    point.pose.pose.orientation.w = std::cos(state.heading/2);
    point.velocity = point.acceleration = std::numeric_limits<double>::quiet_NaN();
    return point;
  }
  static double pathLength(const interfaces::Path& path) {
    double length = 0.0;
    for (std::size_t i = 1; i < path.points.size(); ++i) length += (xy(path.points[i]) - xy(path.points[i - 1])).norm();
    return length;
  }
  void updateRoute(const lanelet::routing::LaneletPath& route) {
    std::vector<LaneKey> ids;
    for (const auto& lane : route) ids.push_back(key(lane));
    if (ids == route_ids_) return;
    route_ids_ = std::move(ids);
    route_lines_.clear();
    for (const auto& lane : route) {
      auto line = center(lane);
      Vec2 lower = line.points.front().head<2>(), upper = lower;
      for (const auto& point : line.points) {
        lower = lower.cwiseMin(point.head<2>());
        upper = upper.cwiseMax(point.head<2>());
      }
      route_lines_.push_back({std::move(line), lower, upper});
    }
  }
  double routeCost(const interfaces::Path& path) const {
    double cost = 0.0;
    for (const auto& point : path.points) {
      double distance = std::numeric_limits<double>::infinity();
      const Vec2 position = xy(point);
      for (const auto& entry : route_lines_) {
        const Vec2 outside = (entry.lower - position).cwiseMax(Vec2::Zero()) +
                             (position - entry.upper).cwiseMax(Vec2::Zero());
        if (outside.squaredNorm() >= distance) continue;
        distance = std::min(distance, project(entry.line, position).distance_squared);
      }
      cost += distance;
    }
    return cost / path.points.size();
  }
  interfaces::Path connect(const State& start, const Reference& ref, double transition, double limit,
                           const Road& road, const std::vector<Box>& obstacles) const {
    interfaces::Path path;
    path.points.push_back(output(start));
    const double origin = project(ref.line, start.position.head<2>()).station;
    const double horizon = std::min(ref.line.stations.back() - origin, limit);
    transition = std::min(transition, horizon);
    if (transition < 1e-6) return path;
    const auto end = sample(ref.line, origin + transition);
    std::array<Vec3, 6> c;
    c[0] = start.position;
    c[1] = {transition*std::cos(start.heading), transition*std::sin(start.heading), end.first.z()*transition};
    c[2] = Vec3::Zero();
    c[2].head<2>() = 0.5*transition*transition*std::tan(start.steer)/parameters_.wheelbase_m*normal(start.heading);
    const Vec3 r0 = end.position - c[0] - c[1] - c[2];
    const Vec3 r1 = transition*end.first - c[1] - 2*c[2];
    const Vec3 r2 = transition*transition*end.second - 2*c[2];
    c[3] = 10*r0 - 4*r1 + 0.5*r2; c[4] = -15*r0 + 7*r1 - r2; c[5] = 6*r0 - 3*r1 + 0.5*r2;
    const auto evaluate = [&](double progress) -> Sample {
      if (progress >= transition) return sample(ref.line, origin + progress);
      const double u = progress/transition;
      return {c[0] + u*(c[1] + u*(c[2] + u*(c[3] + u*(c[4] + u*c[5])))),
              (c[1] + u*(2*c[2] + u*(3*c[3] + u*(4*c[4] + u*5*c[5]))))/transition,
              (2*c[2] + u*(6*c[3] + u*(12*c[4] + u*20*c[5])))/(transition*transition)};
    };
    State previous = start;
    double progress = 0.0, length = 0.0;
    const double max_curvature = std::tan(parameters_.max_steer_rad)/parameters_.wheelbase_m;
    while (progress < horizon - 1e-8 && length < limit - 1e-8) {
      const double metric = evaluate(progress).first.head<2>().norm();
      if (metric < 1e-6) break;
      double step = std::min({std::min(parameters_.sample_step_m, 0.25)/metric,
                              horizon - progress, limit - length});
      if (progress < transition) step = std::min({step, transition - progress, transition/32});
      Sample value = evaluate(progress + step);
      for (int halve = 0; halve < 12 && (value.position.head<2>() - previous.position.head<2>()).norm() >
           std::min({parameters_.sample_step_m, 0.25, limit - length}) + 1e-8; ++halve) {
        step *= 0.5; value = evaluate(progress + step);
      }
      const double ds = (value.position.head<2>() - previous.position.head<2>()).norm();
      const double norm = value.first.head<2>().norm();
      if (ds < 1e-8 || norm < 1e-6) break;
      const double curvature = cross(value.first.head<2>(), value.second.head<2>())/(norm*norm*norm);
      if (std::abs(curvature) > max_curvature + 1e-8) break;
      const State next{value.position, std::atan2(value.first.y(), value.first.x()),
                       std::atan(curvature*parameters_.wheelbase_m)};
      const auto tangent = sample(ref.line, origin + progress + step).first;
      if ((next.position.head<2>() - previous.position.head<2>()).dot(unit(previous.heading)) <= 0 ||
          value.first.head<2>().dot(tangent.head<2>()) <= 0 ||
          std::abs(wrap(next.heading - previous.heading))/ds > max_curvature + 1e-4 ||
          !clear(previous, next, road, obstacles)) break;
      path.points.push_back(output(next)); length += ds; progress += step; previous = next;
    }
    return path;
  }
  Cell cell(const State& state, const State& start, int steering) const {
    const Vec2 delta = state.position.head<2>() - start.position.head<2>();
    const double yaw = wrap(state.heading);
    return {static_cast<int>(std::floor(delta.x()/parameters_.grid_resolution_m)),
            static_cast<int>(std::floor(delta.y()/parameters_.grid_resolution_m)),
            static_cast<int>(std::floor((yaw + 3.141592653589793)*parameters_.yaw_bins/6.283185307179586)), steering};
  }
  static interfaces::Path reconstruct(const std::vector<Node>& nodes, int index, const State& start) {
    std::vector<int> chain;
    for (int i = index; nodes[i].parent >= 0; i = nodes[i].parent) chain.push_back(i);
    interfaces::Path path;
    path.points.push_back(output(start));
    for (auto i = chain.rbegin(); i != chain.rend(); ++i)
      for (const auto& state : nodes[*i].edge) path.points.push_back(output(state));
    return path;
  }
  interfaces::Path search(const State& start, const Reference& ref, const Road& road,
                          const std::vector<Box>& obstacles, interfaces::Path best, double best_progress) const {
    const int straight = static_cast<int>(std::find(parameters_.steering_candidates_deg.begin(),
        parameters_.steering_candidates_deg.end(), 0.0) - parameters_.steering_candidates_deg.begin());
    std::vector<Node> nodes{{start, 0.0, 0.0, 0.0, -1, straight, {}}};
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>> open;
    std::map<Cell, double> costs;
    open.emplace(parameters_.path_length_m, 0);
    costs.emplace(cell(start, start, straight), 0.0);
    double best_cost = std::numeric_limits<double>::infinity();
    for (int expanded = 0; expanded < parameters_.max_expansions && !open.empty(); ++expanded) {
      const int index = open.top().second; open.pop();
      const Node node = nodes[index];
      if (node.cost > costs[cell(node.state, start, node.steering)] + 1e-8) continue;
      const double remaining = parameters_.path_length_m - node.length;
      const auto projection = project(ref.line, node.state.position.head<2>());
      if (expanded % 8 == 0) {
        for (double transition : parameters_.transition_lengths_m) {
          const auto tail = connect(node.state, ref, transition, remaining, road, obstacles);
          const double progress = project(ref.line, xy(tail.points.back())).station - ref.start_s;
          if (progress > best_progress + 1e-6 || (std::abs(progress - best_progress) <= 1e-6 && node.cost < best_cost)) {
            best = reconstruct(nodes, index, start);
            best.points.insert(best.points.end(), tail.points.begin() + 1, tail.points.end());
            best_progress = progress; best_cost = node.cost;
          }
        }
      }
      if (remaining <= 1e-6) continue;
      for (int steering = 0; steering < static_cast<int>(parameters_.steering_candidates_deg.size()); ++steering) {
        const double target = parameters_.steering_candidates_deg[steering]*3.141592653589793/180;
        const double length = std::min(remaining, parameters_.primitive_length_m);
        Node next{node.state, node.length, node.cost, node.progress, index, steering, {}};
        Projection end_projection = projection;
        double advanced = 0.0;
        while (advanced < length - 1e-8) {
          const double ds = std::min({parameters_.sample_step_m, 0.25, length - advanced});
          const double mid_steer = node.state.steer + (target - node.state.steer)*(advanced + ds/2)/length;
          const double curvature = std::tan(mid_steer)/parameters_.wheelbase_m;
          const double angle = curvature*ds;
          State state = next.state;
          const double chord = std::abs(angle) < 1e-8 ? ds : 2*std::sin(angle/2)/curvature;
          state.position.head<2>() += chord*unit(state.heading + angle/2);
          state.heading = wrap(state.heading + angle);
          state.steer = node.state.steer + (target - node.state.steer)*(advanced + ds)/length;
          const auto projection = project(ref.line, state.position.head<2>());
          const auto reference = sample(ref.line, projection.station);
          state.position.z() = reference.position.z();
          if (projection.station - ref.start_s < next.progress - 1e-6 ||
              unit(state.heading).dot(reference.first.head<2>()) <= 0 ||
              !clear(next.state, state, road, obstacles)) break;
          next.edge.push_back(state); next.state = state; next.length += chord;
          end_projection = projection;
          next.progress = projection.station - ref.start_s;
          next.cost += ds*(1 + 0.25*projection.distance_squared + 0.2*target*target) +
                       0.1*std::abs(state.steer - node.state.steer)*ds/length;
          advanced += ds;
        }
        if (next.edge.empty()) continue;
        const bool exhausted = next.length >= parameters_.path_length_m - 0.01;
        if (next.progress > best_progress + 1e-6 || (std::abs(next.progress - best_progress) <= 1e-6 && next.cost < best_cost)) {
          best = reconstruct(nodes, index, start);
          for (const auto& state : next.edge) best.points.push_back(output(state));
          best_progress = next.progress; best_cost = next.cost;
        }
        if (exhausted) continue;
        if (advanced < length - 1e-8) continue;
        const Cell id = cell(next.state, start, steering);
        const auto found = costs.find(id);
        if (found != costs.end() && found->second <= next.cost) continue;
        costs[id] = next.cost;
        // Favor forward progress within the finite search budget; collision checks remain exact.
        const double heuristic = 2*std::max(0.0, parameters_.path_length_m - next.progress) +
            std::sqrt(end_projection.distance_squared);
        nodes.push_back(std::move(next));
        open.emplace(nodes.back().cost + heuristic, static_cast<int>(nodes.size() - 1));
      }
    }
    return best;
  }
  HybridAstarParameters parameters_;
  std::shared_ptr<lanelet::LaneletMap> map_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  std::vector<LaneKey> route_ids_;
  std::vector<RouteLine> route_lines_;
};
}  // namespace path_planner
