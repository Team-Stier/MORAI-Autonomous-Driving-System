#pragma once
#include <geometry_msgs/Point.h>
#include <interfaces/Path.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace path_planner {
struct RddfCoreParameters {
  double local_path_length_m = 50.0;
  std::vector<geometry_msgs::Point> global_path;
};

class RddfCore {
 public:
  explicit RddfCore(RddfCoreParameters parameters) : parameters_(std::move(parameters)) {}

  interfaces::Path process(const nav_msgs::Odometry& ego) {
    const auto& position = ego.pose.pose.position;
    const auto& points = parameters_.global_path;
    std::size_t first_segment = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    geometry_msgs::Point projection;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const auto& from = points[i];
      const auto& to = points[i + 1];
      const double dx = to.x - from.x;
      const double dy = to.y - from.y;
      const double squared_length = dx * dx + dy * dy;
      if (squared_length == 0.0) continue;
      const double ratio = std::max(0.0, std::min(1.0,
          ((position.x - from.x) * dx + (position.y - from.y) * dy) / squared_length));
      const auto candidate = interpolate(from, to, ratio);
      const double distance = std::pow(candidate.x - position.x, 2) +
                              std::pow(candidate.y - position.y, 2);
      if (distance < closest_distance) {
        closest_distance = distance;
        first_segment = i;
        projection = candidate;
      }
    }

    interfaces::Path path;
    appendPoint(path, projection);
    double length_m = 0.0;
    for (std::size_t i = first_segment + 1;
         i < points.size() && length_m < parameters_.local_path_length_m; ++i) {
      const auto from = path.points.back().pose.pose.position;
      const auto& to = points[i];
      const double segment_m = std::hypot(to.x - from.x, to.y - from.y);
      if (segment_m == 0.0) continue;
      const double take_m = std::min(segment_m, parameters_.local_path_length_m - length_m);
      appendPoint(path, interpolate(from, to, take_m / segment_m));
      length_m += take_m;
    }
    for (std::size_t i = 0; i + 1 < path.points.size(); ++i) {
      const auto& from = path.points[i].pose.pose.position;
      const auto& to = path.points[i + 1].pose.pose.position;
      const double yaw = std::atan2(to.y - from.y, to.x - from.x);
      path.points[i].pose.pose.orientation.z = std::sin(yaw / 2.0);
      path.points[i].pose.pose.orientation.w = std::cos(yaw / 2.0);
    }
    if (path.points.size() > 1)
      path.points.back().pose.pose.orientation = path.points[path.points.size() - 2].pose.pose.orientation;
    return path;
  }

 private:
  static geometry_msgs::Point interpolate(const geometry_msgs::Point& from,
                                         const geometry_msgs::Point& to, double ratio) {
    geometry_msgs::Point point;
    point.x = from.x + ratio * (to.x - from.x);
    point.y = from.y + ratio * (to.y - from.y);
    point.z = from.z + ratio * (to.z - from.z);
    return point;
  }

  static void appendPoint(interfaces::Path& path, const geometry_msgs::Point& position) {
    interfaces::PathPoint point;
    point.pose.pose.position = position;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    path.points.push_back(point);
  }

  RddfCoreParameters parameters_;
};
}  // namespace path_planner
