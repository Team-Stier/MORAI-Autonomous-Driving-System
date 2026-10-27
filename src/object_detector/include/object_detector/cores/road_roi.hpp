#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <node_pipeline/extension.hpp>
#include <visualization_msgs/Marker.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace object_detector {
struct RoadRoiParameters {
  lanelet::LaneletMapPtr map;
  double forward_distance_m = 50;
  double left_inflation_m = 0;
  double right_inflation_m = 0;
  double z_min_m = -2.0;
  double z_max_m = 3.0;
};
class RoadRoi {
 public:
  explicit RoadRoi(RoadRoiParameters parameters) : parameters_(std::move(parameters)) {
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    const auto graph = lanelet::routing::RoutingGraph::build(*parameters_.map, *rules);
    for (const auto& lane : parameters_.map->laneletLayer) {
      for (const auto& oriented : {lanelet::ConstLanelet(lane), lanelet::ConstLanelet(lane.invert())}) {
        if (!rules->canPass(oriented)) continue;
        Geometry geometry;
        geometry.center = oriented.centerline2d().basicLineString();
        for (std::size_t i = 1; i < geometry.center.size(); ++i)
          geometry.length += (geometry.center[i] - geometry.center[i - 1]).norm();
        const auto left = oriented.leftBound2d().basicLineString();
        const auto right = oriented.rightBound2d().basicLineString();
        geometry.original = polygon(left, right);
        geometry.expanded = polygon(offset(left, parameters_.left_inflation_m),
                                    offset(right, -parameters_.right_inflation_m));
        geometry.minimum = geometry.maximum = geometry.expanded.front();
        for (const auto& point : geometry.expanded) {
          geometry.minimum = geometry.minimum.cwiseMin(point);
          geometry.maximum = geometry.maximum.cwiseMax(point);
        }
        for (const auto& next : graph->following(oriented, false)) geometry.following.push_back(key(next));
        const auto left_lane = graph->left(oriented);
        const auto right_lane = graph->right(oriented);
        if (left_lane) geometry.neighbors.push_back(key(*left_lane));
        if (right_lane) geometry.neighbors.push_back(key(*right_lane));
        lanes_.emplace(key(oriented), std::move(geometry));
      }
    }
  }

  std::tuple<sensor_msgs::PointCloud2, interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const sensor_msgs::PointCloud2& cloud, const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& tf) {
    const auto& transform = std::find_if(tf.transforms.begin(), tf.transforms.end(),
        [](const geometry_msgs::TransformStamped& value) { return value.child_frame_id == "base_link_level"; }
      )->transform;
    const auto& q = transform.rotation;
    const double yaw = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const Point ego(transform.translation.x, transform.translation.y);
    const auto regions = road(ego, Point(cy, sy));
    publishRoi(regions, cloud.header.stamp, transform.translation.z);
    const double cr = std::cos(competition.roll_rad), sr = std::sin(competition.roll_rad);
    const double cp = std::cos(competition.pitch_rad), sp = std::sin(competition.pitch_rad);
    const auto field_offset = [&](const char* name) {
      return std::find_if(cloud.fields.begin(), cloud.fields.end(),
          [&](const sensor_msgs::PointField& field) { return field.name == name; })->offset;
    };
    const std::uint32_t offsets[] = {field_offset("x"), field_offset("y"), field_offset("z")};
    auto output = cloud;
    std::uint32_t retained = 0;
    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        const auto* record = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
                                               static_cast<std::size_t>(col) * cloud.point_step;
        float xyz[3];
        for (std::size_t axis = 0; axis < 3; ++axis)
          std::memcpy(&xyz[axis], record + offsets[axis], sizeof(float));
        if (xyz[2] < parameters_.z_min_m || xyz[2] > parameters_.z_max_m) continue;
        // Undo roll/pitch to test the base_link collider in a leveled cloud.
        const double bx = cp*xyz[0] - sp*xyz[2];
        const double bz = sp*xyz[0] + cp*xyz[2];
        const double by = cr*xyz[1] + sr*bz;
        const double vehicle_z = -sr*xyz[1] + cr*bz;
        const bool inside_vehicle =
            bx >= -0.7899996042251587 && bx <= 3.845008373260498 &&
            by >= -1.0804176330566406 && by <= 1.080417275428772 &&
            vehicle_z >= -0.3684999644756317 && vehicle_z <= 1.2755463123321533;
        if (inside_vehicle) continue;
        const Point point = ego + Point(cy*xyz[0] - sy*xyz[1], sy*xyz[0] + cy*xyz[1]);
        for (const auto& region : regions) {
          const auto& geometry = *region.geometry;
          if ((point.array() < geometry.minimum.array()).any() ||
              (point.array() > geometry.maximum.array()).any() || !contains(geometry.expanded, point)) continue;
          const double station = project(geometry.center, point).station;
          if (station < region.begin - 1e-8 || station > region.end + 1e-8) continue;
          std::memcpy(output.data.data() + static_cast<std::size_t>(retained) * cloud.point_step,
                      record, cloud.point_step);
          ++retained;
          break;
        }
      }
    }
    output.height = 1;
    output.width = retained;
    output.row_step = retained * output.point_step;
    output.data.resize(output.row_step);
    return std::make_tuple(std::move(output), competition, tf);
  }

 private:
  using Point = lanelet::BasicPoint2d;
  using Line = lanelet::BasicLineString2d;
  using Key = std::pair<lanelet::Id, bool>;
  struct Projection {
    Point point = Point::Zero();
    Point direction = Point::Zero();
    double station = 0;
    double squared_distance = std::numeric_limits<double>::infinity();
  };
  struct Geometry {
    Line center, original, expanded;
    Point minimum, maximum;
    double length = 0;
    std::vector<Key> following, neighbors;
  };
  struct Region {
    const Geometry* geometry;
    double begin, end;
  };
  struct Visit {
    double traveled;
    Key lane;
    double begin;
    Point cross_section;
    bool operator>(const Visit& other) const {
      return std::tie(traveled, lane, begin) > std::tie(other.traveled, other.lane, other.begin);
    }
  };

  static Key key(const lanelet::ConstLanelet& lane) { return {lane.id(), lane.inverted()}; }

  static Line polygon(const Line& left, const Line& right) {
    Line result = left;
    result.insert(result.end(), right.rbegin(), right.rend());
    return result;
  }

  static Line offset(const Line& line, double distance) {
    Line result;
    for (std::size_t i = 0; i < line.size(); ++i) {
      const Point before = (i == 0 ? line[1] - line[0] : line[i] - line[i - 1]).normalized();
      const Point after = (i + 1 == line.size() ? before : Point((line[i + 1] - line[i]).normalized()));
      const Point normal_before(-before.y(), before.x()), normal_after(-after.y(), after.x());
      const Point bisector = normal_before + normal_after;
      const double denominator = bisector.dot(normal_before);
      if (denominator > 1e-8) result.push_back(line[i] + distance * bisector / denominator);
      else {
        result.push_back(line[i] + distance * normal_before);
        result.push_back(line[i] + distance * normal_after);
      }
    }
    return result;
  }

  static bool contains(const Line& polygon, const Point& point) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const auto& a = polygon[j];
      const auto& b = polygon[i];
      const Point delta = b - a;
      const double length_squared = delta.squaredNorm();
      const double ratio = length_squared == 0 ? 0 : std::max(0.0, std::min(1.0, (point - a).dot(delta) / length_squared));
      if ((point - a - ratio * delta).squaredNorm() <= 1e-16) return true;
      if ((a.y() > point.y()) != (b.y() > point.y()) &&
          point.x() < a.x() + (point.y() - a.y()) * delta.x() / delta.y()) inside = !inside;
    }
    return inside;
  }

  static Projection project(const Line& line, const Point& point) {
    Projection result;
    double station = 0;
    for (std::size_t i = 1; i < line.size(); ++i) {
      const Point delta = line[i] - line[i - 1];
      const double length = delta.norm();
      const double ratio = (point - line[i - 1]).dot(delta) / (length * length);
      const double bounded = std::max(0.0, std::min(1.0, ratio));
      const Point projected = line[i - 1] + bounded * delta;
      const double squared_distance = (point - projected).squaredNorm();
      if (squared_distance < result.squared_distance) {
        // Endpoint extrapolation prevents retaining points behind the first cross-section.
        const double longitudinal = (i == 1 && ratio < 0) || (i + 1 == line.size() && ratio > 1) ? ratio : bounded;
        result = {projected, delta / length, station + longitudinal * length, squared_distance};
      }
      station += length;
    }
    return result;
  }

  static Point atStation(const Line& line, double station) {
    for (std::size_t i = 1; i < line.size(); ++i) {
      const Point delta = line[i] - line[i - 1];
      const double length = delta.norm();
      if (station <= length) return line[i - 1] + (station / length) * delta;
      station -= length;
    }
    return line.back();
  }

  std::vector<Region> road(const Point& ego, const Point& heading) const {
    auto current = lanes_.end();
    Projection ego_projection;
    auto best = std::make_tuple(true, std::numeric_limits<double>::infinity(), 2.0, Key{});
    for (auto lane = lanes_.begin(); lane != lanes_.end(); ++lane) {
      const auto projection = project(lane->second.center, ego);
      const double alignment = projection.direction.dot(heading);
      if (alignment <= 0) continue;
      const auto score = std::make_tuple(!contains(lane->second.original, ego), projection.squared_distance,
                                        1 - alignment, lane->first);
      if (score < best) { best = score; current = lane; ego_projection = projection; }
    }
    std::vector<Region> result;
    if (current == lanes_.end()) return result;
    std::priority_queue<Visit, std::vector<Visit>, std::greater<Visit>> pending;
    const double initial_begin = std::max(0.0, std::min(current->second.length, ego_projection.station));
    pending.push({0, current->first, initial_begin, atStation(current->second.center, initial_begin)});
    std::set<std::pair<Key, double>> visited;
    while (!pending.empty()) {
      const auto visit = pending.top();
      pending.pop();
      const double traveled = visit.traveled, begin = visit.begin;
      const auto lane = lanes_.find(visit.lane);
      if (lane == lanes_.end() || traveled > parameters_.forward_distance_m ||
          !visited.insert({lane->first, begin}).second) continue;
      const auto& geometry = lane->second;
      const double end = std::min(geometry.length, begin + parameters_.forward_distance_m - traveled);
      result.push_back({&geometry, begin, end});
      for (const auto& neighbor : geometry.neighbors) {
        const auto adjacent = lanes_.find(neighbor);
        if (adjacent == lanes_.end()) continue;
        const double adjacent_begin = std::max(0.0, std::min(adjacent->second.length,
            project(adjacent->second.center, visit.cross_section).station));
        // Keep one cross-section through lateral traversal to avoid projection drift.
        pending.push({traveled, neighbor, adjacent_begin, visit.cross_section});
      }
      if (end < geometry.length) continue;
      for (const auto& next : geometry.following) {
        const auto successor = lanes_.find(next);
        if (successor != lanes_.end())
          pending.push({traveled + geometry.length - begin, next, 0, successor->second.center.front()});
      }
    }
    return result;
  }

  void publishRoi(const std::vector<Region>& regions, const ros::Time& stamp, double height) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; marker.header.stamp = stamp;
    marker.ns = "lidar_roi"; marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = marker.scale.y = .45;
    marker.color.g = 1; marker.color.b = .25; marker.color.a = .28;
    marker.lifetime = ros::Duration(.5);
    std::set<std::pair<int,int>> cells;
    for (const auto& region : regions) {
      const auto& geometry = *region.geometry;
      for (int x = static_cast<int>(std::ceil(geometry.minimum.x()/.75)); x*.75 <= geometry.maximum.x(); ++x)
        for (int y = static_cast<int>(std::ceil(geometry.minimum.y()/.75)); y*.75 <= geometry.maximum.y(); ++y) {
          const Point p(x*.75, y*.75);
          if (!contains(geometry.expanded, p)) continue;
          const double station = project(geometry.center, p).station;
          if (station < region.begin - 1e-8 || station > region.end + 1e-8 || !cells.emplace(x,y).second) continue;
          geometry_msgs::Point q; q.x = p.x(); q.y = p.y(); q.z = height + .1;
          marker.points.push_back(q);
        }
    }
    roi_view_.publish(marker);
  }
  node_pipeline::Visualizer<visualization_msgs::Marker> roi_view_{"object_motion"};
  RoadRoiParameters parameters_;
  std::map<Key, Geometry> lanes_;
};
}  // namespace object_detector
