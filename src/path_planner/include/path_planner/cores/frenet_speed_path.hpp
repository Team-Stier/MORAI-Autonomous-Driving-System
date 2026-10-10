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
#include <ros/console.h>
#include <array>
#include <deque>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace path_planner {
struct FrenetSpeedPathParameters {
  double path_length_m = 20.0;
  double sample_step_m = 0.5;
  double reference_step_m = 2.0;
  std::vector<double> transition_lengths_m{8.0, 12.0, 16.0};
  std::vector<double> lateral_offsets_m{-0.5, 0.0, 0.5};
  double desired_speed_mps = 8.0;
  double max_acceleration_mps2 = 2.0;
  double max_deceleration_mps2 = 4.0;
  double max_jerk_mps3 = 4.0;
  double max_lateral_acceleration_mps2 = 2.0;
  double time_step_sec = 0.25;
  double command_step_sec = 0.2;
  int beam_width = 64;
  double collision_margin_m = 0.5;
  double prediction_uncertainty_mps = 0.2;
  double vehicle_front_m = 3.845008;
  double vehicle_rear_m = 0.790000;
  double vehicle_half_width_m = 1.080418;
};

class FrenetSpeedPath {
 public:
  FrenetSpeedPath(FrenetSpeedPathParameters parameters, SharedMap& shared)
      : parameters_(std::move(parameters)), map_(shared.map),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(*map_, *rules_)),
        visualizer_("frenet_speed_candidates") {}

  interfaces::Path process(const nav_msgs::Odometry& ego,
                           const interfaces::CompetitionVehicleStatus& competition,
                           const interfaces::Objects& objects, const Route& route,
                           const PredictedObjects& predictions, const ros::Time& stamp) {
    const double observation_time = predictions.front().header.stamp.toSec();
    const double plan_time = stamp.toSec();
    rememberPose(ego, competition);
    const auto frame = observationFrame(plan_time);
    const Eigen::Quaterniond rotation(frame.rotation());
    const Vec2 position(frame.translation().x(), frame.translation().y());
    const double heading = yaw(rotation);
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
    auto obstacles = tubes(predictions);
    for (const auto& object : objects.objects) {
      Tube occupied;
      occupied.boxes.fill(box(object));
      occupied.start = objects.header.stamp.toSec() - observation_time;
      occupied.end = occupied.start + 0.5;
      obstacles.push_back(occupied);
    }
    const double age = plan_time - observation_time, horizon = 5.0 - age;
    const double measured_time = competition.header.stamp.toSec();
    if (measured_time != last_speed_time_) {
      last_acceleration_ = 0.0;
      if (measured_time > last_speed_time_ && measured_time - last_speed_time_ < 0.5)
        last_acceleration_ = clamp((competition.velocity_x_mps - last_speed_) / (measured_time - last_speed_time_),
                                   -parameters_.max_deceleration_mps2, parameters_.max_acceleration_mps2);
    }
    last_speed_ = competition.velocity_x_mps;
    last_speed_time_ = measured_time;
    const double initial_velocity = std::max(0.0,
        last_speed_ + last_acceleration_*(plan_time - measured_time));
    const double cap = std::max(parameters_.desired_speed_mps,
        initial_velocity + std::pow(std::max(0.0, last_acceleration_), 2)/(2*parameters_.max_jerk_mps3));
    visualization_msgs::MarkerArray display;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    display.markers.push_back(clear);
    auto header = ego.header;
    header.stamp = stamp;
    interfaces::Path selected, braking_geometry;
    std::size_t best_progress = 0, braking_progress = 0, selected_marker = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    double braking_cost = std::numeric_limits<double>::infinity();
    for (const auto& reference : references) {
      for (double transition : parameters_.transition_lengths_m) {
        for (double offset : parameters_.lateral_offsets_m) {
          auto candidate = generate(reference, position, heading, transition, offset);
          if (candidate.points.size() < 2) {
            if (braking_geometry.points.empty()) {
              // Retain the lane's last segment so Control accepts the terminal stop target.
              const auto& points = reference.line.points;
              const auto direction = points.back() - points[points.size() - 2];
              const double heading = std::atan2(direction.y(), direction.x());
              for (const auto& point : {points[points.size() - 2], points.back()}) {
                interfaces::PathPoint output;
                output.pose.pose.position.x = point.x();
                output.pose.pose.position.y = point.y();
                output.pose.pose.position.z = point.z();
                output.pose.pose.orientation.z = std::sin(heading / 2);
                output.pose.pose.orientation.w = std::cos(heading / 2);
                braking_geometry.points.push_back(output);
              }
            }
            continue;
          }
          const auto route_cost = routeCost(candidate);
          if (route_cost.first > braking_progress ||
              (route_cost.first == braking_progress && route_cost.second < braking_cost)) {
            braking_geometry = candidate;
            braking_progress = route_cost.first;
            braking_cost = route_cost.second;
          }
          std::vector<double> station(candidate.points.size(), 0.0);
          for (std::size_t i = 1; i < station.size(); ++i) {
            const auto& a = candidate.points[i - 1].pose.pose.position;
            const auto& b = candidate.points[i].pose.pose.position;
            station[i] = station[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
          }
          const auto schedule = search(candidate, station, obstacles, age, horizon,
                                       initial_velocity, last_acceleration_);
          bool accepted = false;
          if (!schedule.empty()) {
            injectProfile(candidate, station, schedule, cap, age);
            accepted = collisionFree(candidate, station, schedule, obstacles, age);
            if (accepted) {
              const double cost = route_cost.second + schedule.back().cost/horizon;
              if (route_cost.first > best_progress ||
                  (route_cost.first == best_progress && cost < best_cost)) {
                const auto command = stateAt(schedule, std::min(parameters_.command_step_sec, horizon));
                candidate.points.front().velocity = command.v;
                candidate.points.front().acceleration = command.a;
                best_progress = route_cost.first;
                best_cost = cost;
                selected = candidate;
                selected_marker = display.markers.size();
              }
            }
          }
          display.markers.push_back(visualize(candidate, header,
              static_cast<int>(display.markers.size()), accepted));
        }
      }
    }
    if (selected_marker != 0) {
      auto& marker = display.markers[selected_marker];
      marker.color.r = 0.1; marker.color.g = 1.0; marker.color.b = 0.1;
      marker.scale.x = 0.12;
    }
    visualizer_.publish(display);
    if (selected.points.empty()) {
      ROS_WARN_THROTTLE(1.0, "frenet_speed_path: no feasible timed candidate; requesting stop");
      for (auto& point : braking_geometry.points) {
        point.velocity = 0.0;
        point.acceleration = -parameters_.max_deceleration_mps2;
      }
      return braking_geometry;
    }
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
  struct Pose {
    double x, y, yaw;
  };
  struct EgoPose {
    double time;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d velocity, angular_velocity;
  };
  struct Box {
    Pose pose;
    double half_length, half_width;
  };
  struct Tube {
    std::array<Box, 11> boxes;
    double start = 0.0, end = 5.0;
  };
  struct State {
    double t, s, v, a, cost;
    int parent;
  };
  struct Edge {
    State start;
    double h, v1, a1;
    double c2() const { return 3 * (v1 - start.v) / (h*h) - (2*start.a + a1) / h; }
    double c3() const { return 2 * (start.v - v1) / (h*h*h) + (start.a + a1) / (h*h); }
    double velocity(double t) const { return start.v + t * (start.a + t * (c2() + t*c3())); }
    double acceleration(double t) const { return start.a + t * (2*c2() + 3*t*c3()); }
    double jerk(double t) const { return 2*c2() + 6*t*c3(); }
    double station(double t) const {
      return start.s + t * (start.v + t * (start.a/2 + t * (c2()/3 + t*c3()/4)));
    }
  };

  static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
  static double angle(double x) { return std::atan2(std::sin(x), std::cos(x)); }
  static double quaternionYaw(const geometry_msgs::Quaternion& q) {
    return std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
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
    marker.ns = "frenet_speed_candidates";
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

  void rememberPose(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& status) {
    const double time = ego.header.stamp.toSec();
    if (!ego_history_.empty() && time < ego_history_.back().time) ego_history_.clear();
    if (ego_history_.empty() || time >= ego_history_.back().time) {
      if (!ego_history_.empty() && time == ego_history_.back().time) ego_history_.pop_back();
      const auto& p = ego.pose.pose.position;
      const auto& q = ego.pose.pose.orientation;
      ego_history_.push_back({time, {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z},
          {status.velocity_x_mps, ego.twist.twist.linear.y, ego.twist.twist.linear.z},
          {status.angular_velocity_x_radps, status.angular_velocity_y_radps, status.angular_velocity_z_radps}});
      while (ego_history_.size() > 2 && time - ego_history_[1].time > 1.0) ego_history_.pop_front();
    }
  }
  Eigen::Isometry3d observationFrame(double time) const {
    const auto after = std::lower_bound(ego_history_.begin(), ego_history_.end(), time,
        [](const EgoPose& pose, double t) { return pose.time < t; });
    Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
    if (after != ego_history_.begin() && after != ego_history_.end() &&
        after->time - (after - 1)->time <= 0.2) {
      const auto& before = *(after - 1);
      const double u = (time - before.time) / (after->time - before.time);
      frame.linear() = before.rotation.slerp(u, after->rotation).toRotationMatrix();
      frame.translation() = (1 - u)*before.position + u*after->position;
    } else {
      const auto& pose = after == ego_history_.end() ? ego_history_.back() : *after;
      const double dt = time - pose.time;
      const double omega = pose.angular_velocity.norm();
      const double turn = omega*dt;
      const double a = std::abs(turn) < 1e-5 ? dt*dt*(0.5 - turn*turn/24) :
          (1 - std::cos(turn))/(omega*omega);
      const double b = std::abs(turn) < 1e-5 ? dt*dt*dt*(1.0/6 - turn*turn/120) :
          (turn - std::sin(turn))/(omega*omega*omega);
      const auto cross = pose.angular_velocity.cross(pose.velocity);
      frame.translation() = pose.position + pose.rotation *
          (dt*pose.velocity + a*cross + b*pose.angular_velocity.cross(cross));
      const Eigen::Quaterniond delta = omega == 0 ? Eigen::Quaterniond::Identity() :
          Eigen::Quaterniond(Eigen::AngleAxisd(turn, pose.angular_velocity / omega));
      // shortcut: constant body twist outside pose history; use timestamp-matched localization for tighter gaps.
      frame.linear() = (pose.rotation * delta).toRotationMatrix();
    }
    return frame;
  }
  static Box box(const interfaces::Object& object) {
    const auto& p = object.bbox_pose.position;
    const auto& q = object.bbox_pose.orientation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const double heading = std::atan2(2*(rotation.w()*rotation.z() + rotation.x()*rotation.y()),
                                      1 - 2*(rotation.y()*rotation.y() + rotation.z()*rotation.z()));
    const Eigen::Vector3d forward(std::cos(heading), std::sin(heading), 0);
    const Eigen::Vector3d lateral(-std::sin(heading), std::cos(heading), 0);
    const auto axes = rotation.toRotationMatrix();
    const auto& size = object.bbox_size;
    return {{p.x, p.y, heading},
        (std::abs(forward.dot(axes.col(0)))*size.x + std::abs(forward.dot(axes.col(1)))*size.y +
         std::abs(forward.dot(axes.col(2)))*size.z) / 2,
        (std::abs(lateral.dot(axes.col(0)))*size.x + std::abs(lateral.dot(axes.col(1)))*size.y +
         std::abs(lateral.dot(axes.col(2)))*size.z) / 2};
  }
  static std::vector<Tube> tubes(const PredictedObjects& predictions) {
    std::vector<Tube> result;
    for (const auto& object : predictions.front().objects) {
      Tube tube;
      for (std::size_t i = 0; i < predictions.size(); ++i) {
        const auto& list = predictions[i].objects;
        const auto predicted = std::find_if(list.begin(), list.end(),
            [&](const interfaces::Object& item) { return item.id == object.id; });
        tube.boxes[i + 1] = box(*predicted);
      }
      tube.boxes[0] = tube.boxes[1];
      tube.boxes[0].pose.x -= tube.boxes[2].pose.x - tube.boxes[1].pose.x;
      tube.boxes[0].pose.y -= tube.boxes[2].pose.y - tube.boxes[1].pose.y;
      tube.boxes[0].pose.yaw -= angle(tube.boxes[2].pose.yaw - tube.boxes[1].pose.yaw);
      // shortcut: back-project the first half-second linearly; include t=0 in upstream predictions for tighter occupancy.
      result.push_back(tube);
    }
    return result;
  }
  static Box boxAt(const Tube& tube, double time) {
    const double index = clamp(time / 0.5, 0, 10);
    const std::size_t before = std::min<std::size_t>(9, static_cast<std::size_t>(index));
    const double u = index - before;
    const auto& a = tube.boxes[before];
    const auto& b = tube.boxes[before + 1];
    return {{a.pose.x + u*(b.pose.x - a.pose.x), a.pose.y + u*(b.pose.y - a.pose.y),
             a.pose.yaw + u*angle(b.pose.yaw - a.pose.yaw)},
            std::max(a.half_length, b.half_length), std::max(a.half_width, b.half_width)};
  }
  static Pose poseAt(const Message& path, const std::vector<double>& station, double s) {
    const auto after = std::upper_bound(station.begin(), station.end(), s);
    const std::size_t i = std::min<std::size_t>(station.size() - 2, after - station.begin() - 1);
    const double u = clamp((s - station[i]) / (station[i + 1] - station[i]), 0, 1);
    const auto& a = path.points[i].pose.pose;
    const auto& b = path.points[i + 1].pose.pose;
    return {a.position.x + u*(b.position.x - a.position.x),
            a.position.y + u*(b.position.y - a.position.y),
            quaternionYaw(a.orientation) + u*angle(quaternionYaw(b.orientation) - quaternionYaw(a.orientation))};
  }
  Box egoBox(const Pose& pose) const {
    const double offset = (parameters_.vehicle_front_m - parameters_.vehicle_rear_m) / 2;
    return {{pose.x + offset*std::cos(pose.yaw), pose.y + offset*std::sin(pose.yaw), pose.yaw},
            (parameters_.vehicle_front_m + parameters_.vehicle_rear_m) / 2, parameters_.vehicle_half_width_m};
  }
  static double support(const Box& box, double axis) {
    return box.half_length*std::abs(std::cos(box.pose.yaw - axis)) +
           box.half_width*std::abs(std::sin(box.pose.yaw - axis));
  }
  static bool sweptOverlap(const Box& a0, const Box& a1, const Box& b0, const Box& b1, double pad) {
    pad += std::hypot(a0.half_length, a0.half_width)*std::abs(angle(a1.pose.yaw - a0.pose.yaw)) +
           std::hypot(b0.half_length, b0.half_width)*std::abs(angle(b1.pose.yaw - b0.pose.yaw));
    double lo = 0, hi = 1;
    for (double axis : {a0.pose.yaw, a0.pose.yaw + M_PI/2, b0.pose.yaw, b0.pose.yaw + M_PI/2}) {
      const double x = std::cos(axis), y = std::sin(axis);
      const double d0 = (b0.pose.x - a0.pose.x)*x + (b0.pose.y - a0.pose.y)*y;
      const double d1 = (b1.pose.x - a1.pose.x)*x + (b1.pose.y - a1.pose.y)*y;
      const double extent = support(a0, axis) + support(b0, axis) + pad;
      const double delta = d1 - d0;
      if (std::abs(delta) < 1e-12) {
        if (std::abs(d0) > extent) return false;
      } else {
        const double enter = (-extent - d0) / delta, leave = (extent - d0) / delta;
        lo = std::max(lo, std::min(enter, leave));
        hi = std::min(hi, std::max(enter, leave));
        if (lo > hi) return false;
      }
    }
    return true;
  }
  static double curvature(const Message& path, const std::vector<double>& station, double a, double b) {
    double maximum = 0;
    for (std::size_t i = 0; i + 1 < station.size(); ++i) {
      if (station[i + 1] < a || station[i] > b) continue;
      const double turn = angle(quaternionYaw(path.points[i + 1].pose.pose.orientation) -
                                quaternionYaw(path.points[i].pose.pose.orientation));
      maximum = std::max(maximum, std::abs(turn) / (station[i + 1] - station[i]));
    }
    return maximum;
  }
  bool admissible(const Edge& edge, double cap, double k) const {
    std::vector<double> times{0, edge.h};
    if (std::abs(edge.c3()) > 1e-12) {
      const double t = -edge.c2() / (3*edge.c3());
      if (t > 0 && t < edge.h) times.push_back(t);
    }
    const double a = 3*edge.c3(), b = 2*edge.c2(), c = edge.start.a;
    const double discriminant = b*b - 4*a*c;
    if (std::abs(a) > 1e-12 && discriminant >= 0) {
      for (double t : {(-b - std::sqrt(discriminant))/(2*a), (-b + std::sqrt(discriminant))/(2*a)})
        if (t > 0 && t < edge.h) times.push_back(t);
    } else if (std::abs(b) > 1e-12) {
      const double t = -c / b;
      if (t > 0 && t < edge.h) times.push_back(t);
    }
    for (double t : times) {
      const double v = edge.velocity(t), acceleration = edge.acceleration(t);
      if (v < -1e-9 || v > cap + 1e-9 || acceleration < -parameters_.max_deceleration_mps2 - 1e-9 ||
          acceleration > parameters_.max_acceleration_mps2 + 1e-9 || std::abs(edge.jerk(t)) > parameters_.max_jerk_mps3 + 1e-9 ||
          v*v*k > parameters_.max_lateral_acceleration_mps2 + 1e-9) return false;
    }
    return true;
  }
  bool clear(const Edge& edge, const Message& path, const std::vector<double>& station,
             const std::vector<Tube>& obstacles, double age) const {
    if (obstacles.empty()) return true;
    for (double t = 0; t < edge.h - 1e-10;) {
      double end = std::min(edge.h, t + 0.05);
      const double next_slice = (std::floor((age + edge.start.t + t) / 0.5 + 1e-9) + 1)*0.5;
      end = std::min(end, next_slice - age - edge.start.t);
      const auto knot = std::upper_bound(station.begin(), station.end(), edge.station(t) + 1e-9);
      if (knot != station.end() && edge.station(end) > *knot) {
        double lo = t, hi = end;
        for (int i = 0; i < 40; ++i) {
          const double middle = (lo + hi) / 2;
          if (edge.station(middle) < *knot) lo = middle;
          else hi = middle;
        }
        end = hi;
      }
      const double s0 = edge.station(t), s1 = edge.station(end);
      const auto a0 = egoBox(poseAt(path, station, s0));
      const auto a1 = egoBox(poseAt(path, station, s1));
      const double k = curvature(path, station, s0, s1);
      const double dt = end - t;
      const double vmax = std::max(edge.velocity(t), edge.velocity(end)) + parameters_.max_acceleration_mps2*dt;
      const double deviation = (std::max(parameters_.max_acceleration_mps2, parameters_.max_deceleration_mps2) + vmax*vmax*k)*dt*dt/8;
      const double offset = std::abs(parameters_.vehicle_front_m - parameters_.vehicle_rear_m) / 2;
      const double pad = parameters_.collision_margin_m +
                         parameters_.prediction_uncertainty_mps*(age + edge.start.t + end) +
                         deviation + 2*offset*std::abs(angle(a1.pose.yaw - a0.pose.yaw));
      for (const auto& tube : obstacles) {
        const double begin_time = std::max(t, tube.start - age - edge.start.t);
        const double end_time = std::min(end, tube.end - age - edge.start.t);
        if (begin_time > end_time) continue;
        const auto from = egoBox(poseAt(path, station, edge.station(begin_time)));
        const auto to = egoBox(poseAt(path, station, edge.station(end_time)));
        if (sweptOverlap(from, to, boxAt(tube, age + edge.start.t + begin_time),
                         boxAt(tube, age + edge.start.t + end_time), pad)) return false;
      }
      t = end;
    }
    return true;
  }
  double stoppingTime(double v, double a) const {
    const double jerk = parameters_.max_jerk_mps3, deceleration = parameters_.max_deceleration_mps2;
    const double peak = std::sqrt(jerk*v + a*a/2);
    if (a < 0 && peak < -a - 1e-9) return std::numeric_limits<double>::infinity();
    if (peak <= deceleration) return (a + 2*peak) / jerk;
    return (a + 2*deceleration) / jerk + (v + (a*a - 2*deceleration*deceleration)/(2*jerk))/deceleration;
  }
  double stoppingDistance(double v, double a) const {
    if (!std::isfinite(stoppingTime(v, a))) return std::numeric_limits<double>::infinity();
    const double jerk = parameters_.max_jerk_mps3, d = parameters_.max_deceleration_mps2;
    const double peak = std::min(d, std::sqrt(jerk*v + a*a/2));
    const double first = (a + peak)/jerk;
    double distance = v*first + a*first*first/2 - jerk*first*first*first/6;
    v += a*first - jerk*first*first/2;
    const double middle = peak > 0 ? std::max(0.0, (v - peak*peak/(2*jerk))/peak) : 0;
    if (peak > 0) {
      distance += v*middle - peak*middle*middle/2;
      v -= peak*middle;
    }
    const double last = peak/jerk;
    return distance + v*last - peak*last*last/2 + jerk*last*last*last/6;
  }
  std::vector<State> stoppingTail(const State& start, double horizon, double cap,
                                  const Message& path, const std::vector<double>& station,
                                  const std::vector<Tube>& obstacles, double age) const {
    if (stoppingTime(start.v, start.a) > horizon - start.t + 1e-9) return {};
    const double jerk = parameters_.max_jerk_mps3, deceleration = parameters_.max_deceleration_mps2;
    const double peak = std::sqrt(jerk*start.v + start.a*start.a/2);
    std::vector<std::pair<double, double>> phases;
    if (peak <= deceleration) {
      phases = {{(start.a + peak)/jerk, -jerk}, {peak/jerk, jerk}};
    } else {
      phases = {{(start.a + deceleration)/jerk, -jerk},
                {(start.v + (start.a*start.a - 2*deceleration*deceleration)/(2*jerk))/deceleration, 0},
                {deceleration/jerk, jerk}};
    }
    State q = start;
    std::vector<State> result{q};
    const auto advance = [&](double duration, double rate) {
      for (double remaining = duration; remaining > 1e-10;) {
        const double h = std::min(parameters_.time_step_sec, remaining);
        const double v = q.v + q.a*h + rate*h*h/2;
        const double a = q.a + rate*h;
        const Edge edge{q, h, std::abs(v) < 1e-9 ? 0 : v, std::abs(a) < 1e-9 ? 0 : a};
        const double s = edge.station(h);
        if (s > station.back() + 1e-9 || !admissible(edge, cap, curvature(path, station, q.s, s)) ||
            !clear(edge, path, station, obstacles, age)) return false;
        const double cost = q.cost + h*(std::pow(parameters_.desired_speed_mps - edge.v1, 2) +
            0.1*edge.a1*edge.a1 + 0.02*(std::pow(edge.jerk(0), 2) + std::pow(edge.jerk(h), 2))) - 2*(s - q.s);
        q = {q.t + h, s, edge.v1, edge.a1, cost, -1};
        result.push_back(q);
        remaining -= h;
      }
      return true;
    };
    for (const auto& phase : phases)
      if (!advance(phase.first, phase.second)) return {};
    if (!advance(std::max(0.0, horizon - q.t), 0)) return {};
    return result;
  }
  static std::vector<State> trace(const std::vector<std::vector<State>>& layers, int index) {
    std::vector<State> result;
    for (std::size_t layer = layers.size(); layer-- > 0;) {
      const auto q = layers[layer][static_cast<std::size_t>(index)];
      result.push_back(q);
      index = q.parent;
    }
    std::reverse(result.begin(), result.end());
    return result;
  }
  std::vector<State> search(const Message& path, const std::vector<double>& station,
                            const std::vector<Tube>& obstacles, double age, double horizon,
                            double v0, double a0) const {
    if (station.size() < 2 || horizon <= 0) return {};
    const double cap = std::max(parameters_.desired_speed_mps,
        v0 + std::pow(std::max(0.0, a0), 2)/(2*parameters_.max_jerk_mps3));
    std::vector<std::vector<State>> layers{{{0, 0, v0, a0, 0, -1}}};
    std::vector<State> best;
    for (double time = 0; time < horizon - 1e-9;) {
      const double h = std::min(parameters_.time_step_sec, horizon - time);
      using Key = std::tuple<long long, long long, long long>;
      std::map<Key, State> cells;
      const auto& previous = layers.back();
      const auto braking = std::min_element(previous.begin(), previous.end(),
          [&](const State& a, const State& b) { return stoppingTime(a.v, a.a) < stoppingTime(b.v, b.a); });
      // Analytic braking phases retain a feasible terminal stop between lattice timestamps.
      for (int index : {0, static_cast<int>(braking - previous.begin())}) {
        auto tail = stoppingTail(previous[index], horizon, cap, path, station, obstacles, age);
        if (!tail.empty() && (best.empty() || tail.back().cost < best.back().cost)) {
          best = trace(layers, index);
          best.insert(best.end(), tail.begin() + 1, tail.end());
        }
      }
      for (std::size_t index = 0; index < previous.size(); ++index) {
        const auto& q = previous[index];
        const double lo_a = std::max(-parameters_.max_deceleration_mps2, q.a - parameters_.max_jerk_mps3*h);
        const double hi_a = std::min(parameters_.max_acceleration_mps2, q.a + parameters_.max_jerk_mps3*h);
        for (double a : {lo_a, hi_a, clamp(0, lo_a, hi_a), clamp(q.a, lo_a, hi_a)}) {
          const double lo_v = std::max(0.0, q.v + std::max((4*q.a + 2*a)*h - parameters_.max_jerk_mps3*h*h,
                                                        (2*q.a + 4*a)*h - parameters_.max_jerk_mps3*h*h)/6);
          const double hi_v = std::min(cap, q.v + std::min((4*q.a + 2*a)*h + parameters_.max_jerk_mps3*h*h,
                                                         (2*q.a + 4*a)*h + parameters_.max_jerk_mps3*h*h)/6);
          if (lo_v > hi_v + 1e-10) continue;
          for (double v : {lo_v, std::max(lo_v, hi_v), clamp(0, lo_v, std::max(lo_v, hi_v)),
                           clamp(q.v + (q.a + a)*h/2, lo_v, std::max(lo_v, hi_v)),
                           clamp(parameters_.desired_speed_mps, lo_v, std::max(lo_v, hi_v))}) {
            const Edge edge{q, h, v, a};
            const double s = edge.station(h);
            if (s + stoppingDistance(v, a) > station.back() + 1e-9 ||
                !admissible(edge, cap, curvature(path, station, q.s, s)) ||
                !clear(edge, path, station, obstacles, age)) continue;
            if (time + h >= horizon - 1e-9) {
              const State terminal{time + h, s, v, a, 0, -1};
              if (stoppingTail(terminal, terminal.t + stoppingTime(v, a), cap,
                               path, station, {}, age).empty()) continue;
            }
            const double cost = q.cost + h*(std::pow(parameters_.desired_speed_mps - v, 2) +
                0.1*a*a + 0.02*(std::pow(edge.jerk(0), 2) + std::pow(edge.jerk(h), 2))) - 2*(s - q.s);
            const State next{time + h, s, v, a, cost, static_cast<int>(index)};
            const auto velocity_cell = v == 0 ? 0LL : 1 + std::llround(v / 0.125);
            const auto acceleration_cell = a == 0 ? 0LL : (a > 0 ? 1LL : -1LL) + std::llround(a / 0.25);
            const Key key{std::llround(s / 0.25), velocity_cell, acceleration_cell};
            const auto found = cells.find(key);
            if (found == cells.end() || cost < found->second.cost) cells[key] = next;
          }
        }
      }
      std::vector<State> next;
      for (const auto& cell : cells) next.push_back(cell.second);
      std::sort(next.begin(), next.end(), [](const State& a, const State& b) { return a.cost < b.cost; });
      if (next.empty()) break;
      if (next.size() > static_cast<std::size_t>(parameters_.beam_width)) {
        // shortcut: finite beam can miss a feasible gap; widen it or use an exhaustive lattice when required.
        const auto braking = std::min_element(next.begin(), next.end(),
            [&](const State& a, const State& b) { return stoppingTime(a.v, a.a) < stoppingTime(b.v, b.a); });
        const State keep_braking = *braking;
        const auto stopped = std::find_if(next.begin(), next.end(),
            [](const State& q) { return std::abs(q.v) < 1e-9 && std::abs(q.a) < 1e-9; });
        const bool have_stopped = stopped != next.end();
        const State keep = have_stopped ? *stopped : next.front();
        next.resize(parameters_.beam_width);
        next.back() = keep_braking;
        if (have_stopped && next.size() > 1) next[next.size() - 2] = keep;
      }
      layers.push_back(std::move(next));
      time += h;
    }
    if (layers.back().front().t >= horizon - 1e-9 &&
        (best.empty() || layers.back().front().cost < best.back().cost)) return trace(layers, 0);
    return best;
  }
  static State stateAt(const std::vector<State>& schedule, double time) {
    const auto after = std::upper_bound(schedule.begin(), schedule.end(), time,
        [](double t, const State& state) { return t < state.t; });
    if (after == schedule.end()) return schedule.back();
    const auto& before = *(after - 1);
    const Edge edge{before, after->t - before.t, after->v, after->a};
    const double t = time - before.t;
    return {time, edge.station(t), std::max(0.0, edge.velocity(t)), edge.acceleration(t), 0, -1};
  }

  void injectProfile(Message& path, const std::vector<double>& station,
                     const std::vector<State>& nominal, double cap, double age) const {
    auto schedule = nominal;
    const auto terminal = schedule.back();
    // shortcut: braking beyond prediction coverage is kinematic; replan before executing it.
    const auto tail = stoppingTail(terminal, terminal.t + stoppingTime(terminal.v, terminal.a),
                                   cap, path, station, {}, age);
    schedule.insert(schedule.end(), tail.begin() + 1, tail.end());
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto after = std::lower_bound(schedule.begin(), schedule.end(), station[i],
          [](const State& state, double s) { return state.s < s; });
      if (after == schedule.end()) {
        path.points[i].velocity = 0.0;
        path.points[i].acceleration = 0.0;
      } else if (after == schedule.begin()) {
        path.points[i].velocity = after->v;
        path.points[i].acceleration = after->a;
      } else {
        const auto& before = *(after - 1);
        const Edge edge{before, after->t - before.t, after->v, after->a};
        double lo = 0, hi = edge.h;
        for (int iteration = 0; iteration < 40; ++iteration) {
          const double t = (lo + hi)/2;
          if (edge.station(t) < station[i]) lo = t;
          else hi = t;
        }
        path.points[i].velocity = std::max(0.0, edge.velocity(hi));
        path.points[i].acceleration = edge.acceleration(hi);
      }
    }
  }
  bool collisionFree(const Message& path, const std::vector<double>& station,
                     const std::vector<State>& schedule, const std::vector<Tube>& obstacles,
                     double age) const {
    for (std::size_t i = 1; i < schedule.size(); ++i) {
      const auto& a = schedule[i - 1];
      const auto& b = schedule[i];
      if (!clear({a, b.t - a.t, b.v, b.a}, path, station, obstacles, age)) return false;
    }
    return true;
  }

  FrenetSpeedPathParameters parameters_;
  std::shared_ptr<lanelet::LaneletMap> map_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  std::map<LaneKey, LaneData> lanes_;
  std::vector<LaneKey> route_ids_;
  std::vector<RouteSegment> route_segments_;
  node_pipeline::Visualizer<visualization_msgs::MarkerArray> visualizer_;
  std::deque<EgoPose> ego_history_;
  double last_speed_ = 0;
  double last_speed_time_ = -std::numeric_limits<double>::infinity();
  double last_acceleration_ = 0;
};
}  // namespace path_planner
