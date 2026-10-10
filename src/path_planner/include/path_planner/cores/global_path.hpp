#pragma once
#include <sensor_msgs/Imu.h>
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/types.hpp>
#include <nav_msgs/Path.h>
#include <node_pipeline/extension.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>

namespace path_planner {
// 전달받은 체크포인트를 순서대로 경유하는 경로를 탐색한다. 마지막 체크포인트가 목적지다.
class GlobalPath {
 public:
  explicit GlobalPath(SharedMap& shared)
      : map_(shared.map),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(*map_, *rules_)),
        visualizer_("global_path") {}

  std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::Objects,
             tf2_msgs::TFMessage, sensor_msgs::Imu, Route> process(
      const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
      const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
      const sensor_msgs::Imu& imu, const Checkpoints& checkpoints) {
    const auto& position = ego.pose.pose.position;
    const auto& q = ego.pose.pose.orientation;
    const double yaw_rad = std::atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z));
    const lanelet::BasicPoint2d ego_xy(position.x, position.y);
    const auto start = match(ego_xy, yaw_rad);
    lanelet::Ids checkpoint_ids;
    for (const auto& checkpoint : checkpoints) checkpoint_ids.push_back(checkpoint.lanelet_id);
    if (!route_ || route_->front() != start || checkpoint_ids != checkpoint_ids_) {
      checkpoint_ids_ = checkpoint_ids;
      if (checkpoints.empty()) {
        route_ = lanelet::routing::LaneletPath({start});
      } else {
        lanelet::ConstLanelets via;
        for (std::size_t i = 0; i + 1 < checkpoints.size(); ++i)
          via.push_back(map_->laneletLayer.get(checkpoints[i].lanelet_id));
        route_ = graph_->shortestPathVia(start, via, map_->laneletLayer.get(checkpoints.back().lanelet_id));
      }
    }

    auto route = route_;
    nav_msgs::Path path;
    path.header = ego.header;
    if (route) {
      const auto predecessors = graph_->previous(start, false);
      if (!predecessors.empty()) {
        // shortcut: use the first predecessor at merges; track the incoming lane when ambiguity matters.
        lanelet::ConstLanelets lanes{predecessors.front()};
        lanes.insert(lanes.end(), route->begin(), route->end());
        route = lanelet::routing::LaneletPath(std::move(lanes));
      }
      for (const auto& lane : *route) {
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
    return std::make_tuple(ego, competition, objects, tf, imu, std::move(route));
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
    const auto candidates = lanelet::geometry::findNearest(map_->laneletLayer, position, 10);
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

  std::shared_ptr<lanelet::LaneletMap> map_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  lanelet::Ids checkpoint_ids_;
  Route route_;
  node_pipeline::Visualizer<nav_msgs::Path> visualizer_;
};
}  // namespace path_planner
