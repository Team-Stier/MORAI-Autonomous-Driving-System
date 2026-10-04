#pragma once
#include <path_planner/types.hpp>
#include <nav_msgs/Path.h>
#include <ros/publisher.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace path_planner {
class GlobalPath {
 public:
  using Result = Route;
  explicit GlobalPath(GlobalPathParameters parameters, ros::Publisher visualizer = {})
      : parameters_(std::move(parameters)),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(*parameters_.map, *rules_)),
        visualizer_(std::move(visualizer)) {
    for (const auto& checkpoint : parameters_.checkpoints)
      checkpoints_.push_back(parameters_.map->laneletLayer.get(checkpoint.lanelet_id));
  }

  Result process(const nav_msgs::Odometry& ego, const interfaces::SectionAnnotation& section) {
    const auto& position = ego.pose.pose.position;
    const auto& q = ego.pose.pose.orientation;
    const double yaw_rad = std::atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z));
    const lanelet::BasicPoint2d ego_xy(position.x, position.y);
    const auto start = match(ego_xy, yaw_rad);
    const std::size_t completed = std::max(completed_checkpoints_, static_cast<std::size_t>(section.state));
    if (!route_ || route_->front() != start || completed != completed_checkpoints_) {
      completed_checkpoints_ = completed;
      if (completed == checkpoints_.size()) {
        route_ = lanelet::routing::LaneletPath({start});
      } else {
        const lanelet::ConstLanelets via(checkpoints_.begin() + completed, checkpoints_.end() - 1);
        route_ = graph_->shortestPathVia(start, via, checkpoints_.back());
      }
    }

    if (visualizer_) {
      nav_msgs::Path path;
      path.header = ego.header;
      if (route_) {
        for (const auto& lane : *route_) {
          for (const auto& point : lane.centerline3d()) {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            pose.pose.orientation.w = 1;
            path.poses.push_back(pose);
          }
        }
      }
      visualizer_.publish(path);
    }
    return route_;
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

  lanelet::ConstLanelet match(const lanelet::BasicPoint2d& position, double yaw_rad) const {
    // ponytail: ten spatial candidates; increase the query count if junction density requires it.
    const auto candidates = lanelet::geometry::findNearest(parameters_.map->laneletLayer, position, 10);
    std::vector<std::pair<std::tuple<bool, double, double, double, lanelet::Id, bool>, lanelet::ConstLanelet>> matches;
    for (const auto& candidate : candidates) {
      for (const auto& lanelet : {candidate.second, candidate.second.invert()}) {
        if (!rules_->canPass(lanelet)) continue;
        const auto line = lanelet.centerline3d().basicLineString();
        const auto projection = project(line, position);
        const auto direction = line[projection.segment + 1] - line[projection.segment];
        const double heading_error = 1 - std::cos(std::atan2(direction.y(), direction.x()) - yaw_rad);
        matches.emplace_back(std::make_tuple(heading_error >= 1, candidate.first,
            projection.squared_distance, heading_error, lanelet.id(), lanelet.inverted()), lanelet);
      }
    }
    std::sort(matches.begin(), matches.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    return matches.at(0).second;
  }

  GlobalPathParameters parameters_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  lanelet::ConstLanelets checkpoints_;
  std::size_t completed_checkpoints_ = 0;
  lanelet::Optional<lanelet::routing::LaneletPath> route_;
  ros::Publisher visualizer_;
};
}  // namespace path_planner
