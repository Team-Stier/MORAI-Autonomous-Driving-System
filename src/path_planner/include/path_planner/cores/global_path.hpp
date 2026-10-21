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
#include <iterator>
#include <tuple>
#include <utility>

namespace path_planner {
struct GlobalPathParameters {
  lanelet::Ids adjacent_checkpoint_lanelet_ids;
  double checkpoint_radius_m = 10.0;
};
// 전달받은 체크포인트를 순서대로 경유하는 경로를 탐색한다. 마지막 체크포인트가 목적지다.
class GlobalPath {
 public:
  explicit GlobalPath(SharedMap& shared)
      : GlobalPath(GlobalPathParameters{}, shared) {}

  GlobalPath(GlobalPathParameters parameters, SharedMap& shared)
      : parameters_(std::move(parameters)), map_(shared.map),
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
    lanelet::Ids checkpoint_ids;
    for (const auto& checkpoint : checkpoints) checkpoint_ids.push_back(checkpoint.lanelet_id);
    const auto start = match(ego_xy, yaw_rad, checkpoint_ids == checkpoint_ids_);
    if (!route_ || route_->front() != start || checkpoint_ids != checkpoint_ids_) {
      checkpoint_ids_ = checkpoint_ids;
      if (checkpoints.empty()) {
        route_ = lanelet::routing::LaneletPath({start});
      } else {
        lanelet::ConstLanelets via;
        for (std::size_t i = 0; i + 1 < checkpoints.size(); ++i)
          via.push_back(map_->laneletLayer.get(checkpoints[i].lanelet_id));
        route_ = graph_->shortestPathVia(start, via, map_->laneletLayer.get(checkpoints.back().lanelet_id));
        if (route_) {
          route_ = earliestFirstLaneChange(*route_, checkpoint_ids);
          auto routing_ids = checkpoint_ids;
          // #ADJACENT_CHECKPOINT_PASSAGE
          // 승인한 체크포인트만 기존 경로에서 바로 다음으로 선택한 합법적
          // 옆 차선으로 통과할 수 있다. 원본 체크포인트 위치와 섹션 판정은
          // 그대로 두고, 이 차선 중심선이 같은 통과 반경 안을 지나는지 확인한다.
          // 최종 목적지·다른 필수 경유 차선은 바꾸지 않는다. 앞뒤 차량 간격과
          // 실제 변경 시각·차체 경계는 LocalPath의 기존 시간별 검사로 결정한다.
          // 지도 ID는 임무 설정으로 주입하며 GT 객체나 특정 차량 ID를 쓰지 않는다.
          for (std::size_t i = 0; i + 1 < checkpoints.size(); ++i) {
            if (std::find(parameters_.adjacent_checkpoint_lanelet_ids.begin(),
                          parameters_.adjacent_checkpoint_lanelet_ids.end(), checkpoint_ids[i]) ==
                parameters_.adjacent_checkpoint_lanelet_ids.end()) continue;
            const auto current = std::find_if(route_->begin(), route_->end(),
                [&](const lanelet::ConstLanelet& lane) { return lane.id() == checkpoint_ids[i]; });
            if (current == route_->end() || std::next(current) == route_->end()) continue;
            // 이미 옆 차선에 있는 경우 원본 경유 조건이 그 앞에서 복귀 변경을
            // 만들 수 있다. 체크포인트 이후의 기존 변경 방향만 따로 조회한다.
            const auto continuation = earliestFirstLaneChange(lanelet::routing::LaneletPath(
                lanelet::ConstLanelets(current, route_->end())), checkpoint_ids);
            const auto& target = continuation[1];
            const auto left = graph_->left(*current), right = graph_->right(*current);
            if (!(left && *left == target) && !(right && *right == target)) continue;
            const auto nearby = project(target.centerline3d().basicLineString(),
                lanelet::BasicPoint2d(checkpoints[i].x_m, checkpoints[i].y_m));
            if (nearby.squared_distance <= parameters_.checkpoint_radius_m * parameters_.checkpoint_radius_m)
              routing_ids[i] = target.id();
          }
          if (routing_ids != checkpoint_ids) {
            lanelet::ConstLanelets adjusted_via;
            for (std::size_t i = 0; i + 1 < routing_ids.size(); ++i)
              adjusted_via.push_back(map_->laneletLayer.get(routing_ids[i]));
            const auto adjusted = graph_->shortestPathVia(start, adjusted_via,
                map_->laneletLayer.get(routing_ids.back()));
            if (adjusted) route_ = earliestFirstLaneChange(*adjusted, routing_ids);
          }
        }
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
  GlobalPathParameters parameters_;
  // #EARLY_ADJACENT_LANE_CHANGE
  // 최단 경로가 같은 방향의 병렬 차선 변경을 먼 짧은 lanelet까지 미루면
  // 주도로 진입 후 변경 가능한 긴 구간을 지나친다. 첫 변경만 바로 앞의
  // 종방향 source/target 쌍으로 당긴다. 같은 방향의 공유 경계와 RoutingGraph의
  // 합법 left/right 및 양쪽 following 관계가 모두 확인될 때만 이동한다.
  // 필수 경유 lanelet을 제거하지 않으며 다른 차선 변경은 재배열하지 않는다.
  // 실제 곡선과 속도·앞뒤 객체 간격은 LocalPath가 기존 차체·시간별 충돌
  // 검사로 결정한다. 전역 참조가 바뀌어도 변경을 강제하거나 제동을 해제하지 않는다.
  lanelet::routing::LaneletPath earliestFirstLaneChange(
      const lanelet::routing::LaneletPath& path, const lanelet::Ids& checkpoints) const {
    lanelet::ConstLanelets lanes(path.begin(), path.end());
    const auto follows = [&](const lanelet::ConstLanelet& a, const lanelet::ConstLanelet& b) {
      const auto next = graph_->following(a, false);
      return std::find(next.begin(), next.end(), b) != next.end();
    };
    for (std::size_t i = 0; i + 1 < lanes.size(); ++i) {
      const auto left = graph_->left(lanes[i]), right = graph_->right(lanes[i]);
      const bool move_left = left && *left == lanes[i + 1];
      if (!move_left && !(right && *right == lanes[i + 1])) continue;
      while (i > 0 && std::find(checkpoints.begin(), checkpoints.end(), lanes[i].id()) == checkpoints.end()) {
        const auto prior_target = move_left ? graph_->left(lanes[i - 1]) : graph_->right(lanes[i - 1]);
        if (!prior_target || !follows(lanes[i - 1], lanes[i]) || !follows(*prior_target, lanes[i + 1])) break;
        if (move_left ? lanes[i - 1].leftBound().id() != prior_target->rightBound().id()
                      : lanes[i - 1].rightBound().id() != prior_target->leftBound().id()) break;
        lanes[i] = *prior_target;
        --i;
      }
      break;
    }
    return lanelet::routing::LaneletPath(std::move(lanes));
  }

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

  lanelet::ConstLanelet match(const lanelet::BasicPoint2d& position, double yaw_rad, bool same_checkpoints) const {
    // ponytail: ten spatial candidates; increase the query count if junction density requires it.
    const auto candidates = lanelet::geometry::findNearest(map_->laneletLayer, position, 10);
    std::vector<std::pair<std::tuple<bool, double, bool, double, double, lanelet::Id, bool>, lanelet::ConstLanelet>> matches;
    bool keep_planned_branch = false;
    for (const auto& candidate : candidates) {
      for (const auto& lanelet : {candidate.second, candidate.second.invert()}) {
        if (!rules_->canPass(lanelet)) continue;
        const auto line = lanelet.centerline3d().basicLineString();
        const auto projection = project(line, position);
        const auto direction = line[projection.segment + 1] - line[projection.segment];
        const double heading_error = 1 - std::cos(std::atan2(direction.y(), direction.x()) - yaw_rad);
        // #JUNCTION_ROUTE_CONTINUITY
        // 분기 polygon이 겹치면 자세의 작은 변화만으로 출구와 순환로가 번갈아
        // 선택될 수 있다. 같은 체크포인트 임무에서 실제 점을 포함하는 후보끼리는
        // 직전에 계획한 route의 중심선이 경계보다 가까울 때만 출구를 유지한다.
        // 경계에 더 가까워지면 다른 가지에 진입하는 것으로 보고 기존 방향 매칭을
        // 적용한다. 차선 폭에서 계산하므로 별도 거리·각도 보정 계수를 두지 않는다.
        // 전후 방향과 polygon 거리의 우선순위는 그대로다. 원래 polygon을
        // 떠나거나 방향이 반대가 되면 다른 차선으로 재매칭하며, 임무 변경과
        // 첫 호출에는 기존 방향/중심선 판정을 사용한다. 지도/차선 변경 권한,
        // 경유지, 도로 여유·객체·MPC 제약을 변경하지 않는다.
        const bool leaves_planned_branch = same_checkpoints && route_ && heading_error < 1 && candidate.first == 0 &&
            std::find(route_->begin(), route_->end(), lanelet) == route_->end();
        if (same_checkpoints && route_ && heading_error < 1 && candidate.first == 0 &&
            !leaves_planned_branch) {
          const double boundary_distance = std::min(
              lanelet::geometry::distance2d(lanelet.leftBound2d(), position),
              lanelet::geometry::distance2d(lanelet.rightBound2d(), position));
          keep_planned_branch = keep_planned_branch ||
              projection.squared_distance <= boundary_distance * boundary_distance;
        }
        matches.emplace_back(std::make_tuple(heading_error >= 1, candidate.first,
            leaves_planned_branch, heading_error, projection.squared_distance,
            lanelet.id(), lanelet.inverted()), lanelet);
      }
    }
    if (!keep_planned_branch) {
      for (auto& match : matches) std::get<2>(match.first) = false;
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
