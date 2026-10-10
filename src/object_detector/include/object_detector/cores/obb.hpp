#pragma once
#include <interfaces/Objects.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <object_detector/libraries/types.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace object_detector {
class Obb {
 public:
  std::tuple<interfaces::Objects, interfaces::CompetitionVehicleStatus> process(
      const Clusters& clusters,
      const interfaces::CompetitionVehicleStatus& competition) {
    interfaces::Objects output;
    for (const auto& cluster : clusters) output.objects.push_back(makeObb(cluster));
    return std::make_tuple(std::move(output), competition);
  }

 private:
  using XY = std::pair<double, double>;
  static constexpr double kPi = 3.14159265358979323846;

  static interfaces::Object makeAabb(const Cloud& cloud) {
    auto minimum = cloud.front();
    auto maximum = minimum;
    for (const auto& point : cloud) {
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      minimum.z = std::min(minimum.z, point.z);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
      maximum.z = std::max(maximum.z, point.z);
    }
    interfaces::Object object;
    // AABB uses the extrema midpoint, not the point centroid; sizes are full lengths.
    object.position.x = (static_cast<double>(minimum.x) + maximum.x) / 2.0;
    object.position.y = (static_cast<double>(minimum.y) + maximum.y) / 2.0;
    object.position.z = (static_cast<double>(minimum.z) + maximum.z) / 2.0;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.x = 0.0;
    object.bbox_pose.orientation.y = 0.0;
    object.bbox_pose.orientation.z = 0.0;
    object.bbox_pose.orientation.w = 1.0;
    object.bbox_size.x = static_cast<double>(maximum.x) - minimum.x;
    object.bbox_size.y = static_cast<double>(maximum.y) - minimum.y;
    object.bbox_size.z = static_cast<double>(maximum.z) - minimum.z;
    object.label = "unknown";
    object.velocity_available = false;
    object.acceleration_available = false;
    return object;
  }

  static double canonicalYaw(double yaw_rad) {
    yaw_rad = std::fmod(yaw_rad + kPi / 2.0, kPi);
    if (yaw_rad < 0.0) yaw_rad += kPi;
    return yaw_rad - kPi / 2.0;
  }

  static double cross(const XY& a, const XY& b, const XY& c) {
    return (b.first-a.first)*(c.second-a.second) -
           (b.second-a.second)*(c.first-a.first);
  }

  static std::vector<XY> convexHull(std::vector<XY> points) {
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() <= 2) return points;
    std::vector<XY> hull;
    hull.reserve(2 * points.size());
    for (const auto& point : points) {
      while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), point) <= 0.0)
        hull.pop_back();
      hull.push_back(point);
    }
    const auto lower_size = hull.size();
    for (auto it = points.rbegin()+1; it != points.rend(); ++it) {
      while (hull.size() > lower_size && cross(hull[hull.size()-2], hull.back(), *it) <= 0.0)
        hull.pop_back();
      hull.push_back(*it);
    }
    hull.pop_back();  // The first point occurs at both ends.
    return hull;
  }

  struct Rectangle {
    double center_x, center_y, length, width, yaw_rad;
  };

  static Rectangle rectangleAt(const std::vector<XY>& hull, const XY& origin, double yaw_rad) {
    const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
    double min_u = std::numeric_limits<double>::infinity(), min_v = min_u;
    double max_u = -min_u, max_v = -min_u;
    for (const auto& point : hull) {
      // Translate before projecting to reduce cancellation under translation.
      const double x = point.first-origin.first, y = point.second-origin.second;
      const double u = c*x + s*y, v = -s*x + c*y;
      min_u = std::min(min_u, u); max_u = std::max(max_u, u);
      min_v = std::min(min_v, v); max_v = std::max(max_v, v);
    }
    const double u = (min_u+max_u)/2.0, v = (min_v+max_v)/2.0;
    Rectangle rectangle{origin.first+c*u-s*v, origin.second+s*u+c*v,
                        max_u-min_u, max_v-min_v, yaw_rad};
    if (rectangle.length < rectangle.width) {
      std::swap(rectangle.length, rectangle.width);
      rectangle.yaw_rad += kPi/2.0;
    }
    rectangle.yaw_rad = canonicalYaw(rectangle.yaw_rad);
    return rectangle;
  }

  static int compareScalar(double a, double b) {
    // Roundoff ties only: no geometric zero-area threshold or float-size padding.
    const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() *
                             std::max(std::abs(a), std::abs(b));
    if (a < b-tolerance) return -1;
    if (a > b+tolerance) return 1;
    return 0;
  }

  static bool precedes(const Rectangle& a, const Rectangle& b) {
    // Minimum area, then shorter long/short sides, then |canonical yaw|, then yaw.
    for (const auto& values : {XY{a.length*a.width, b.length*b.width},
                              XY{a.length, b.length}, XY{a.width, b.width},
                              XY{std::abs(a.yaw_rad), std::abs(b.yaw_rad)}}) {
      const int order = compareScalar(values.first, values.second);
      if (order != 0) return order < 0;
    }
    return a.yaw_rad < b.yaw_rad;
  }

  static interfaces::Object makeObb(const Cloud& cloud) {
    auto object = makeAabb(cloud);  // Retains Z extrema and Object semantics.
    std::vector<XY> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud) points.emplace_back(point.x, point.y);
    const auto hull = convexHull(std::move(points));
    // One XY point: zero XY extent and identity yaw, possibly nonzero Z extent.
    if (hull.size() == 1) return object;

    Rectangle best{};
    if (hull.size() == 2) {
      // Two points or exact collinearity: endpoints define a zero-width line box.
      const auto& a = hull.front();
      const auto& b = hull.back();
      best = {(a.first+b.first)/2.0, (a.second+b.second)/2.0,
              std::hypot(b.first-a.first, b.second-a.second), 0.0,
              canonicalYaw(std::atan2(b.second-a.second, b.first-a.first))};
    } else {
      // Every minimum-area rectangle has a side parallel to a convex-hull edge.
      // O(n log n + h^2); h is the number of unique convex-hull vertices.
      // Near-zero area hulls use this same calculation without division by area.
      const auto& origin = hull.front();
      best = rectangleAt(hull, origin, std::atan2(hull[1].second-origin.second,
                                                 hull[1].first-origin.first));
      for (std::size_t i = 1; i < hull.size(); ++i) {
        const auto& a = hull[i];
        const auto& b = hull[(i+1) % hull.size()];
        const auto candidate = rectangleAt(hull, origin, std::atan2(b.second-a.second,
                                                                   b.first-a.first));
        if (precedes(candidate, best)) best = candidate;
      }
    }
    object.position.x = best.center_x;
    object.position.y = best.center_y;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.z = std::sin(best.yaw_rad/2.0);
    object.bbox_pose.orientation.w = std::cos(best.yaw_rad/2.0);
    object.bbox_size.x = best.length;
    object.bbox_size.y = best.width;
    return object;
  }
};
}  // namespace object_detector
