#pragma once
#include <path_planner/types.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace path_planner {
class LocalPath {
 public:
  explicit LocalPath(LocalPathParameters parameters)
      : parameters_(std::move(parameters)),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(*parameters_.map, *rules_)) {}

  Message process(const nav_msgs::Odometry& ego, const interfaces::SectionAnnotation&,
                  const Route& global_path, const PredictedObjects&) {
    const auto& route = global_path.value();
    const auto& position = ego.pose.pose.position;
    const lanelet::BasicPoint2d ego_xy(position.x, position.y);
    Message message;
    const auto first_line = route.front().centerline3d().basicLineString();
    const auto projection = project(first_line, ego_xy);
    appendPoint(message, projection.point);
    double length_m = 0;
    appendLine(message, first_line, projection.segment + 1, length_m);
    for (std::size_t i = 1; i < route.size() && length_m < parameters_.local_path_length_m; ++i)
      appendLine(message, route[i].centerline3d().basicLineString(), 0, length_m);

    auto last = route.back();
    while (length_m < parameters_.local_path_length_m) {
      const auto successors = graph_->following(last, false);
      if (successors.empty()) break;
      const auto last_line = last.centerline3d().basicLineString();
      const auto incoming = last_line.back() - last_line[last_line.size() - 2];
      const double incoming_yaw = std::atan2(incoming.y(), incoming.x());
      auto score = [&](const lanelet::ConstLanelet& next) {
        const auto line = next.centerline3d().basicLineString();
        const auto direction = line[1] - line[0];
        return std::make_pair(1 - std::cos(std::atan2(direction.y(), direction.x()) - incoming_yaw), next.id());
      };
      const auto next = *std::min_element(successors.begin(), successors.end(),
          [&](const lanelet::ConstLanelet& a, const lanelet::ConstLanelet& b) { return score(a) < score(b); });
      const double previous_length_m = length_m;
      appendLine(message, next.centerline3d().basicLineString(), 0, length_m);
      last = next;
      if (length_m == previous_length_m) break;
    }
    for (std::size_t i = 0; i + 1 < message.points.size(); ++i) {
      const auto& from = message.points[i].pose.pose.position;
      const auto& to = message.points[i + 1].pose.pose.position;
      const double heading = std::atan2(to.y - from.y, to.x - from.x);
      auto& orientation = message.points[i].pose.pose.orientation;
      orientation.z = std::sin(heading / 2);
      orientation.w = std::cos(heading / 2);
    }
    if (message.points.size() > 1)
      message.points.back().pose.pose.orientation = message.points[message.points.size() - 2].pose.pose.orientation;
    return message;
  }

 private:
  struct Projection {
    lanelet::BasicPoint3d point;
    std::size_t segment = 0;
    double squared_distance = std::numeric_limits<double>::infinity();
  };

  static Projection project(const lanelet::BasicLineString3d& line, const lanelet::BasicPoint2d& position) {
    Projection closest;
    for (std::size_t i = 0; i + 1 < line.size(); ++i) {
      const lanelet::BasicPoint2d from = line[i].head<2>();
      const lanelet::BasicPoint2d delta = line[i + 1].head<2>() - from;
      const double squared_length = delta.squaredNorm();
      if (squared_length == 0) continue;
      const double ratio = std::max(0.0, std::min(1.0, (position - from).dot(delta) / squared_length));
      const lanelet::BasicPoint3d point = line[i] + ratio * (line[i + 1] - line[i]);
      const double distance = (point.head<2>() - position).squaredNorm();
      if (distance < closest.squared_distance) closest = {point, i, distance};
    }
    return closest;
  }

  static void appendPoint(Message& message, const lanelet::BasicPoint3d& position) {
    interfaces::PathPoint point;
    point.pose.pose.position.x = position.x();
    point.pose.pose.position.y = position.y();
    point.pose.pose.position.z = position.z();
    point.pose.pose.orientation.w = 1;
    message.points.push_back(point);
  }

  void appendLine(Message& message, const lanelet::BasicLineString3d& line,
                  std::size_t begin, double& length_m) const {
    for (std::size_t i = begin; i < line.size() && length_m < parameters_.local_path_length_m; ++i) {
      const auto& previous = message.points.back().pose.pose.position;
      const lanelet::BasicPoint3d from(previous.x, previous.y, previous.z);
      const lanelet::BasicPoint3d delta = line[i] - from;
      const double segment_m = delta.head<2>().norm();
      if (segment_m == 0) continue;
      const double take_m = std::min(segment_m, parameters_.local_path_length_m - length_m);
      appendPoint(message, from + (take_m / segment_m) * delta);
      length_m += take_m;
    }
  }

  LocalPathParameters parameters_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
};
}  // namespace path_planner
