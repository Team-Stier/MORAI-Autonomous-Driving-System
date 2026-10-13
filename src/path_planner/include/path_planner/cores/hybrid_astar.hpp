#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <boost/geometry/algorithms/distance.hpp>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <utility>
namespace path_planner {
struct HybridAstarParameters {
  double path_length_m = 50.0, sample_step_m = 0.25, primitive_length_m = 1.0;
  double grid_resolution_m = 0.5;
  int yaw_bins = 72, max_expansions = 800;
  double max_steer_rad = 0.698132, wheelbase_m = 3.0;
  double vehicle_front_m = 3.845008, vehicle_rear_m = 0.79, vehicle_half_width_m = 1.080418;
  double collision_margin_m = 1.2, road_margin_m = 0.05;
  double prediction_acceleration_mps2 = 6.0;
  double emergency_deceleration_mps2 = 6.0;
  double occupancy_history_sec = 0.8;
  std::vector<double> transition_lengths_m{8.0, 12.0, 16.0};
  std::vector<double> lateral_offsets_m{0.0, -1.5, 1.5, -3.0, 3.0, -4.5, 4.5, -6.0, 6.0, -7.5};
  std::vector<double> steering_candidates_deg{-40, -35, -30, -25, -20, -15, -10, -5,
                                               0, 5, 10, 15, 20, 25, 30, 35, 40};
};
class HybridAstar {
 public:
  HybridAstar(HybridAstarParameters parameters, SharedMap& shared)
      : parameters_(std::move(parameters)),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(*shared.map, *rules_)) {}

  interfaces::Path process(const nav_msgs::Odometry& ego, const interfaces::Objects& objects,
                           const Route& route, const PredictedObjects& predictions) {
    const auto& p = ego.pose.pose.position;
    const double heading = poseYaw(ego.pose.pose), speed = ego.twist.twist.linear.x;
    const double motion_dt = std::max(0.0, (ego.header.stamp - previous_motion_stamp_).toSec());
    const double traveled = has_previous_position_ ?
        motion_dt*(previous_speed_mps_ + std::max(0.0, speed))/2 : 0.0;
    previous_motion_stamp_ = ego.header.stamp; previous_speed_mps_ = std::max(0.0, speed);
    const double steer = speed > 0.25 ? std::max(-parameters_.max_steer_rad,
        std::min(parameters_.max_steer_rad,
            std::atan(parameters_.wheelbase_m*ego.twist.twist.angular.z/speed))) : 0.0;
    const State start{{p.x, p.y, p.z}, heading, steer};
    if (has_previous_position_)
      avoidance_remaining_m_ = std::max(0.0, avoidance_remaining_m_ - traveled);
    has_previous_position_ = true;
    if (avoidance_remaining_m_ == 0.0) previous_offset_m_ = 0.0;
    const auto current = match(start, *route);
    std::vector<Reference> references;
    extend(current, start, *route, references);
    const auto obstacles = occupancy(objects, predictions, ego.header.stamp);
    prediction_speed_mps_ = std::max(0.0, speed);
    observation_age_sec_ = std::max(0.0, (ego.header.stamp - objects.header.stamp).toSec());
    for (const auto& object : objects.objects)
      if (overlaps(egoBox(start, 0.0), objectBox(object))) return {};
    fit(references.front().line);
    const auto center_reference = references.front();
    const bool left_lane_change = static_cast<bool>(graph_->left(center_reference.lanes.front()));
    lanelet::ConstLanelets road_lanes = center_reference.lanes;
    for (const auto& lane : center_reference.lanes)
      for (const auto& neighbor : {graph_->left(lane), graph_->right(lane)})
        if (neighbor && std::find(road_lanes.begin(), road_lanes.end(), *neighbor) == road_lanes.end())
          road_lanes.push_back(*neighbor);
    std::vector<lanelet::BasicPolygon2d> roads;
    for (const auto& lane : road_lanes) roads.push_back(lane.polygon2d().basicPolygon());
    const double origin = project(center_reference.line, start.position.head<2>()).station;
    if (previous_offset_m_ != 0.0 && has_bypass_anchor_) {
      const auto projection = project(center_reference.line, bypass_anchor_);
      const auto value = sample(center_reference.line, projection.station);
      const Vec2 tangent = value.first.head<2>().normalized();
      const double physical_offset = (bypass_anchor_ - value.position.head<2>()).dot(Vec2(-tangent.y(), tangent.x()));
      previous_offset_m_ = *std::min_element(parameters_.lateral_offsets_m.begin(), parameters_.lateral_offsets_m.end(),
          [&](double a, double b) { return std::abs(a - physical_offset) < std::abs(b - physical_offset); });
    }
    double obstacle_distance = 0.0;
    for (const auto& object : objects.objects) {
      const auto box = objectBox(object);
      const double progress = signedStation(center_reference.line, box.center) - origin;
      if (progress > -parameters_.vehicle_rear_m && progress < parameters_.path_length_m)
        obstacle_distance = std::max(obstacle_distance, progress + parameters_.vehicle_rear_m + 10.0);
    }
    double ramp_end = parameters_.path_length_m;
    for (const auto& obstacle : obstacles) {
      const double station = signedStation(center_reference.line, obstacle.box.center);
      const auto value = sample(center_reference.line, station);
      const State on_center{value.position, std::atan2(value.first.y(), value.first.x()), 0.0};
      if (station > origin - parameters_.vehicle_rear_m && station < origin + parameters_.path_length_m &&
          overlaps(egoBox(on_center, parameters_.collision_margin_m), obstacle.box))
        ramp_end = std::min(ramp_end, station - origin - parameters_.vehicle_front_m -
            std::hypot(obstacle.box.half_length, obstacle.box.half_width) - 2.0);
    }
    if (previous_offset_m_ != 0.0 && has_bypass_anchor_)
      ramp_end = std::min(ramp_end, signedStation(center_reference.line, bypass_anchor_) - origin);
    const double ramp_begin = ramp_end - std::max(16.0, 2.0*prediction_speed_mps_);
    const auto anchorFor = [&](const Reference& ref) -> Vec2 {
      const auto value = sample(center_reference.line, origin + std::max(0.0, ramp_end));
      const Vec2 tangent = value.first.head<2>().normalized();
      return value.position.head<2>() + ref.offset_m*Vec2(-tangent.y(), tangent.x());
    };
    auto offsets = parameters_.lateral_offsets_m;
    std::stable_sort(offsets.begin(), offsets.end(), [&](double a, double b) {
      return std::abs(a - previous_offset_m_) < std::abs(b - previous_offset_m_);
    });
    for (double offset : offsets) {
      if (offset == 0.0) continue;
      Reference ref;
      ref.offset_m = offset;
      ref.lanes = center_reference.lanes;
      const auto& line = center_reference.line;
      const auto add = [&](double station) {
        const auto value = sample(line, station);
        const Vec2 tangent = value.first.head<2>().normalized();
        const double u = ramp_end <= ramp_begin ? 1.0 : std::max(0.0, std::min(1.0,
            (station - origin - ramp_begin)/(ramp_end - ramp_begin)));
        const double weight = u*u*u*(10.0 + u*(-15.0 + 6.0*u));
        Vec3 point = value.position;
        point.head<2>() += offset*weight*Vec2(-tangent.y(), tangent.x());
        append(ref.line, point);
      };
      for (double station = 0.0; station < line.stations.back(); station += 2.0) add(station);
      add(line.stations.back());
      references.push_back(std::move(ref));
    }
    for (auto& ref : references) {
      fit(ref.line);
      ref.start_s = project(ref.line, start.position.head<2>()).station;
    }
    interfaces::Path best;
    double best_progress = -1.0, best_cost = std::numeric_limits<double>::infinity();
    std::size_t selected_reference = 0;
    double selected_transition = 0.0;
    bool selected_anchor = false;
    bool full = false;
    bool retained_side = false;
    for (std::size_t i = 0; i < references.size(); ++i) {
      auto& ref = references[i];
      auto transitions = parameters_.transition_lengths_m;
      double anchored_transition = -1.0;
      if (has_merge_anchor_ && ref.offset_m == previous_offset_m_) {
        const auto projection = project(ref.line, merge_anchor_);
        const double remaining = projection.station - ref.start_s;
        if (projection.distance_squared < 0.25 && remaining > 0.5 && remaining <= parameters_.path_length_m) {
          anchored_transition = remaining;
          transitions.insert(transitions.begin(), remaining);
        }
      }
      for (double transition : transitions) {
        const auto candidate = connect(start, ref, transition, parameters_.path_length_m, obstacles);
        if (candidate.points.empty()) continue;
        const double length = pathLength(candidate);
        const double progress = project(center_reference.line, xy(candidate.points.back())).station - origin;
        const double side_cost = !left_lane_change && ref.offset_m > 0.0 ? 4*ref.offset_m*ref.offset_m : 0.0;
        const double cost = routeCost(candidate, center_reference.line) + 64*roadCost(candidate, roads) + transition + side_cost +
            4*std::pow(ref.offset_m - previous_offset_m_, 2);
        const bool complete = length >= parameters_.path_length_m - 0.01;
        const bool stable_side = previous_offset_m_ != 0.0 && ref.offset_m == previous_offset_m_ &&
            progress >= std::max(10.0, 2.0*prediction_speed_mps_ + parameters_.vehicle_front_m);
        const bool better = (complete && !full) || (complete == full &&
            (complete ? cost < best_cost : progress > best_progress + 1e-6 ||
                (std::abs(progress - best_progress) <= 1e-6 && cost < best_cost)));
        if ((stable_side && !retained_side) || ((!retained_side || stable_side) && better)) {
          best = candidate; best_progress = progress; best_cost = cost;
          selected_reference = i; selected_transition = transition; full = complete;
          selected_anchor = transition == anchored_transition;
        }
        retained_side = retained_side || stable_side;
      }
      // Keep a usable bypass prefix instead of asking the controller to switch sides abruptly.
      if (retained_side) break;
      if (full && i == 0) break;
    }
    if (full) {
      const bool recovering = previous_offset_m_ != 0.0;
      const auto& ref = references[selected_reference];
      remember(ref, obstacle_distance, anchorFor(ref), center_reference.line);
      // Finish the merge at a fixed world point instead of moving its end ahead on every replan.
      merge_anchor_ = sample(ref.line, ref.start_s + selected_transition).position.head<2>();
      has_merge_anchor_ = selected_anchor || (ref.offset_m == 0.0 && recovering) ||
          (ref.offset_m != 0.0 && selected_transition >= ramp_end);
      emergency_stopping_ = false;
      return best;
    }
    const auto& reference = references[selected_reference];
    if (retained_side) {
      remember(reference, obstacle_distance, anchorFor(reference), center_reference.line);
      has_merge_anchor_ = false;
      return emergencyStop(std::move(best), start, obstacles);
    }
    best = search(start, reference, center_reference.line, obstacles, std::move(best), best_progress);
    has_merge_anchor_ = false;
    return emergencyStop(std::move(best), start, obstacles);
  }

 private:
  using Vec2 = Eigen::Vector2d;
  using Vec3 = Eigen::Vector3d;
  struct Line {
    std::vector<Vec3> points, second;
    std::vector<double> stations;
  };
  struct Reference { Line line; lanelet::ConstLanelets lanes; double start_s = 0.0, offset_m = 0.0; };
  struct Sample { Vec3 position, first, second; };
  struct Projection { double station = 0.0, distance_squared = std::numeric_limits<double>::infinity(); };
  struct State { Vec3 position; double heading, steer; double distance = 0.0; };
  struct Box { Vec2 center; double heading, half_length, half_width; };
  struct ObservedBox { std::uint32_t id; ros::Time stamp; Box box; };
  struct Occupancy { Box box; double begin_sec, end_sec; };
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
  static double signedStation(const Line& line, const Vec2& position) {
    const double station = project(line, position).station;
    if (station <= 1e-6) {
      const auto value = sample(line, 0.0);
      return station + std::min(0.0, (position - value.position.head<2>()).dot(value.first.head<2>().normalized()));
    }
    if (station >= line.stations.back() - 1e-6) {
      const auto value = sample(line, line.stations.back());
      return station + std::max(0.0, (position - value.position.head<2>()).dot(value.first.head<2>().normalized()));
    }
    return station;
  }
  lanelet::ConstLanelet match(const State& start, const lanelet::routing::LaneletPath& route) const {
    const double alignment_weight = 2*std::pow(*std::max_element(parameters_.transition_lengths_m.begin(),
        parameters_.transition_lengths_m.end()), 2);
    const auto score = [&](const lanelet::ConstLanelet& lane) {
      const auto line = center(lane);
      const auto projection = project(line, start.position.head<2>());
      const auto tangent = sample(line, projection.station).first;
      const double alignment = 1 - std::cos(std::atan2(tangent.y(), tangent.x()) - start.heading);
      return std::make_tuple(alignment >= 1.0, projection.distance_squared + alignment_weight*alignment, alignment,
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
    Reference ref{center(seed), {seed}, 0.0};
    ref.start_s = project(ref.line, start.position.head<2>()).station;
    auto next = std::find(route.begin(), route.end(), seed);
    for (++next; next != route.end() && ref.line.stations.back() - ref.start_s <
         parameters_.path_length_m + parameters_.vehicle_front_m; ++next) {
      const auto following = graph_->following(ref.lanes.back(), false);
      if (std::find(following.begin(), following.end(), *next) == following.end()) {
        if (ref.lanes.size() > 1) break;
        // A lateral edge overlaps the old lane; join its center with C2 instead of driving back to its start.
        ref = {center(*next), {*next}, 0.0};
        ref.start_s = project(ref.line, start.position.head<2>()).station;
        continue;
      }
      for (const auto& point : next->centerline3d()) append(ref.line, {point.x(), point.y(), point.z()});
      ref.lanes.push_back(*next);
    }
    result.push_back(std::move(ref));
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
  static Box enclose(const Box& a, const Box& b) {
    const Vec2 forward = unit(a.heading), side = normal(a.heading);
    const Vec2 delta = b.center - a.center;
    const double x = delta.dot(forward), y = delta.dot(side);
    const double length = b.half_length*std::abs(unit(b.heading).dot(forward)) +
        b.half_width*std::abs(normal(b.heading).dot(forward));
    const double width = b.half_length*std::abs(unit(b.heading).dot(side)) +
        b.half_width*std::abs(normal(b.heading).dot(side));
    const double x_min = std::min(-a.half_length, x - length), x_max = std::max(a.half_length, x + length);
    const double y_min = std::min(-a.half_width, y - width), y_max = std::max(a.half_width, y + width);
    return {a.center + 0.5*(x_min + x_max)*forward + 0.5*(y_min + y_max)*side,
            a.heading, 0.5*(x_max - x_min), 0.5*(y_max - y_min)};
  }
  std::vector<Occupancy> occupancy(const interfaces::Objects& objects, const PredictedObjects& predictions,
                                   const ros::Time& stamp) {
    observation_history_.erase(std::remove_if(observation_history_.begin(), observation_history_.end(),
        [&](const ObservedBox& observed) {
          return (stamp - observed.stamp).toSec() > parameters_.occupancy_history_sec;
        }), observation_history_.end());
    if (objects.header.stamp != previous_objects_stamp_) {
      for (const auto& object : objects.objects)
        observation_history_.push_back({object.id, objects.header.stamp, objectBox(object)});
      previous_objects_stamp_ = objects.header.stamp;
    }
    std::map<std::uint32_t, Box> observed_boxes;
    for (const auto& object : objects.objects) observed_boxes.emplace(object.id, objectBox(object));
    // Recent world footprints survive a narrow view or a brief detection dropout.
    for (const auto& observed : observation_history_) {
      const auto found = observed_boxes.find(observed.id);
      if (found == observed_boxes.end()) observed_boxes.emplace(observed.id, observed.box);
      else found->second = enclose(found->second, observed.box);
    }
    std::vector<Occupancy> result;
    for (const auto& observed : observed_boxes)
      result.push_back({observed.second, -std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity()});
    const interfaces::Objects* previous = &objects;
    double begin = 0.0;
    for (const auto& prediction : predictions) {
      for (const auto& object : prediction.objects) {
        const auto box = objectBox(object);
        const auto from = std::find_if(previous->objects.begin(), previous->objects.end(),
            [&](const auto& candidate) { return candidate.id == object.id; });
        result.push_back({from == previous->objects.end() ? box : sweptBox(objectBox(*from), box),
                          begin, &prediction == &predictions.back() ? std::numeric_limits<double>::infinity() : begin + 0.5});
      }
      previous = &prediction; begin += 0.5;
    }
    return result;
  }
  Box egoBox(const State& state, double margin) const {
    const double offset = (parameters_.vehicle_front_m - parameters_.vehicle_rear_m)/2;
    return {state.position.head<2>() + offset*unit(state.heading), state.heading,
            (parameters_.vehicle_front_m + parameters_.vehicle_rear_m)/2 + margin,
            parameters_.vehicle_half_width_m + margin};
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
  bool clear(const State& from, const State& to, const std::vector<Occupancy>& obstacles) const {
    const double offset = std::abs(parameters_.vehicle_front_m - parameters_.vehicle_rear_m)/2;
    const auto margin = [&](const State& state) {
      return parameters_.collision_margin_m*std::min(1.0, state.distance/3.0);
    };
    const auto sweep = sweptBox(egoBox(from, margin(from)), egoBox(to, margin(to)), offset);
    const double root = std::sqrt(prediction_speed_mps_*prediction_speed_mps_ +
                                  2*parameters_.prediction_acceleration_mps2*from.distance);
    const double begin = observation_age_sec_ + (from.distance <= 1e-8 ? 0.0 :
        2*from.distance/(prediction_speed_mps_ + root));
    // shortcut: allow acceleration and half-speed arrival; stopped execution needs a velocity-planning contract.
    const double end = observation_age_sec_ + to.distance/std::max(0.25, prediction_speed_mps_/2);
    for (const auto& obstacle : obstacles) {
      if (end < obstacle.begin_sec - 0.25 || begin > obstacle.end_sec + 0.25) continue;
      if (overlaps(sweep, obstacle.box)) return false;
    }
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
  interfaces::Path emergencyStop(interfaces::Path path, const State& start,
                                  const std::vector<Occupancy>& obstacles) {
    const auto advance = [&](const State& from, double distance) {
      const double curvature = std::tan(from.steer)/parameters_.wheelbase_m;
      const double angle = curvature*distance;
      const double chord = std::abs(angle) < 1e-8 ? distance : 2*std::sin(angle/2)/curvature;
      State next = from;
      next.position.head<2>() += chord*unit(from.heading + angle/2);
      next.heading = wrap(from.heading + angle); next.distance += chord;
      return next;
    };
    const double step = std::min(parameters_.sample_step_m, 0.25);
    if (path.points.size() == 1 && !clear(start, advance(start, step), obstacles)) {
      double distance = step;
      for (int halve = 0; halve < 16; ++halve) {
        distance *= 0.5;
        const auto next = advance(start, distance);
        if (clear(start, next, obstacles)) {
          path.points.push_back(output(next));
          emergency_stopping_ = true;
          break;
        }
      }
    }
    const double length = pathLength(path);
    const double stopping_distance = prediction_speed_mps_*prediction_speed_mps_/
        (2*parameters_.emergency_deceleration_mps2) + 0.05*prediction_speed_mps_;
    if (path.points.size() < 2) return path;
    if (length >= parameters_.path_length_m - 0.01) {
      emergency_stopping_ = false;
      return path;
    }
    if (!emergency_stopping_ &&
        length > std::max(parameters_.primitive_length_m, stopping_distance)) return path;
    const auto& pose = path.points.back().pose.pose;
    const auto& previous = path.points[path.points.size() - 2].pose.pose;
    const double ds = (xy(path.points.back()) - xy(path.points[path.points.size() - 2])).norm();
    const double curvature = wrap(poseYaw(pose) - poseYaw(previous))/ds;
    State end{{pose.position.x, pose.position.y, pose.position.z}, poseYaw(pose),
              std::atan(curvature*parameters_.wheelbase_m), length};
    const auto next = advance(end, step);
    emergency_stopping_ = !clear(end, next, obstacles) || (emergency_stopping_ &&
        length < std::max(parameters_.primitive_length_m, stopping_distance));
    if (emergency_stopping_) {
      // Finite values are reserved for emergency braking; the downstream controller requires both.
      for (auto& point : path.points) point.velocity = point.acceleration = 0.0;
    }
    return path;
  }
  void remember(const Reference& ref, double obstacle_distance, const Vec2& anchor, const Line& center_line) {
    const bool continuing = has_bypass_anchor_ && previous_offset_m_ == ref.offset_m;
    previous_offset_m_ = ref.offset_m;
    has_bypass_anchor_ = previous_offset_m_ != 0.0;
    if (has_bypass_anchor_) {
      avoidance_remaining_m_ = std::max(avoidance_remaining_m_, obstacle_distance);
      if (!continuing || signedStation(center_line, anchor) < signedStation(center_line, bypass_anchor_))
        bypass_anchor_ = anchor;
    }
  }
  static double routeCost(const interfaces::Path& path, const Line& reference) {
    double cost = 0.0;
    std::size_t count = 0;
    for (std::size_t i = 0; i < path.points.size(); i += 4, ++count)
      cost += project(reference, xy(path.points[i])).distance_squared;
    return cost / count;
  }
  double roadCost(const interfaces::Path& path, const std::vector<lanelet::BasicPolygon2d>& roads) const {
    double cost = 0.0;
    std::size_t count = 0;
    for (std::size_t i = 0; i < path.points.size(); i += 12) {
      const auto& pose = path.points[i].pose.pose;
      const auto box = egoBox({{pose.position.x, pose.position.y, pose.position.z}, poseYaw(pose), 0.0}, 0.0);
      const Vec2 forward = unit(box.heading), side(-forward.y(), forward.x());
      for (double x : {-box.half_length, box.half_length})
        for (double y : {-box.half_width, box.half_width}) {
          const Vec2 point = box.center + x*forward + y*side;
          double distance = std::numeric_limits<double>::infinity();
          for (const auto& road : roads) distance = std::min(distance, boost::geometry::distance(point, road));
          cost += std::pow(std::max(0.0, distance - parameters_.road_margin_m), 2);
          ++count;
        }
    }
    return cost/count;
  }
  interfaces::Path connect(const State& start, const Reference& ref, double transition, double limit,
                           const std::vector<Occupancy>& obstacles) const {
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
    const double start_curvature = std::tan(start.steer)/parameters_.wheelbase_m;
    c[2].head<2>() = 0.5*transition*transition*start_curvature*normal(start.heading);
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
                       std::atan(curvature*parameters_.wheelbase_m), previous.distance + ds};
      if ((next.position.head<2>() - previous.position.head<2>()).dot(unit(previous.heading)) <= 0 ||
          std::abs(wrap(next.heading - previous.heading))/ds > max_curvature + 1e-4 ||
          !clear(previous, next, obstacles)) break;
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
  interfaces::Path search(const State& start, const Reference& ref, const Line& progress_line,
                          const std::vector<Occupancy>& obstacles, interfaces::Path best, double best_progress) const {
    const double progress_origin = project(progress_line, start.position.head<2>()).station;
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
          const auto tail = connect(node.state, ref, transition, remaining, obstacles);
          const double progress = project(progress_line, xy(tail.points.back())).station - progress_origin;
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
          state.distance += chord;
          state.heading = wrap(state.heading + angle);
          state.steer = node.state.steer + (target - node.state.steer)*(advanced + ds)/length;
          const auto projection = project(ref.line, state.position.head<2>());
          const auto reference = sample(ref.line, projection.station);
          state.position.z() = reference.position.z();
          if (!clear(next.state, state, obstacles)) break;
          next.edge.push_back(state); next.state = state; next.length += chord;
          end_projection = projection;
          next.progress = project(progress_line, state.position.head<2>()).station - progress_origin;
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
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  double prediction_speed_mps_ = 0.0, observation_age_sec_ = 0.0;
  std::vector<ObservedBox> observation_history_;
  ros::Time previous_objects_stamp_;
  bool emergency_stopping_ = false;
  bool has_previous_position_ = false;
  double previous_offset_m_ = 0.0, avoidance_remaining_m_ = 0.0;
  ros::Time previous_motion_stamp_;
  double previous_speed_mps_ = 0.0;
  Vec2 bypass_anchor_ = Vec2::Zero();
  bool has_bypass_anchor_ = false;
  Vec2 merge_anchor_ = Vec2::Zero();
  bool has_merge_anchor_ = false;
};
}  // namespace path_planner
