#pragma once
#include <interfaces/Object.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace object_detector {
// #OBB
// 입력은 base_link_level에서 ROI·VoxelGrid·DBSCAN을 거친 한 cluster의 유한한 점이다.
// XY Convex Hull의 모든 edge 방향을 조사해 최소 면적 rectangle을 선택한다.
// PCA는 비대칭 점 분포에서 최소 면적을 보장하지 않으므로 기존 hull 계산을 유지한다.
// XY 중심은 rectangle의 중심, Z 중심·크기는 해당 cluster의 원래 min/max로 계산한다.
// bbox_size는 박스 자체 축의 전체 길이이며 quaternion은 그 축을 입력 좌표계로 회전한다.
// 긴 축을 x로 정렬하고 yaw를 [-pi/2, pi/2)로 정규화해 점 순서·반회전 모호성을 줄인다.
// 한 점·공선 cluster와 면적 동률도 기존 분기와 tie-break 순서로 처리한다.
// makeAabb는 Z 범위 초기화·비교용 helper이며 최종 검출은 항상 makeObb를 사용한다.
// ROI Core 통합 후에도 여기서 crop·mounting offset·roll/pitch·frame 역변환을 추가하지 않는다.
namespace bbox {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using XY = std::pair<double, double>;
constexpr double kPi = 3.14159265358979323846;

inline interfaces::Object makeAabb(const Cloud& cloud, const std::vector<int>& cluster) {
  auto minimum = cloud[cluster.front()];
  auto maximum = minimum;
  for (const auto index : cluster) {
    const auto& point = cloud[index];
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

inline double canonicalYaw(double yaw_rad) {
  yaw_rad = std::fmod(yaw_rad + kPi / 2.0, kPi);
  if (yaw_rad < 0.0) yaw_rad += kPi;
  return yaw_rad - kPi / 2.0;
}

inline double cross(const XY& a, const XY& b, const XY& c) {
  return (b.first-a.first)*(c.second-a.second) -
         (b.second-a.second)*(c.first-a.first);
}

inline std::vector<XY> convexHull(std::vector<XY> points) {
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

inline Rectangle rectangleAt(const std::vector<XY>& hull, const XY& origin, double yaw_rad) {
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

inline int compareScalar(double a, double b) {
  // Roundoff ties only: no geometric zero-area threshold or float-size padding.
  const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() *
                           std::max(std::abs(a), std::abs(b));
  if (a < b-tolerance) return -1;
  if (a > b+tolerance) return 1;
  return 0;
}

inline bool precedes(const Rectangle& a, const Rectangle& b) {
  // Minimum area, then shorter long/short sides, then |canonical yaw|, then yaw.
  for (const auto& values : {XY{a.length*a.width, b.length*b.width},
                            XY{a.length, b.length}, XY{a.width, b.width},
                            XY{std::abs(a.yaw_rad), std::abs(b.yaw_rad)}}) {
    const int order = compareScalar(values.first, values.second);
    if (order != 0) return order < 0;
  }
  return a.yaw_rad < b.yaw_rad;
}

inline interfaces::Object makeObb(const Cloud& cloud, const std::vector<int>& cluster) {
  auto object = makeAabb(cloud, cluster);  // Retains Z extrema and Object semantics.
  std::vector<XY> points;
  points.reserve(cluster.size());
  for (const auto index : cluster) points.emplace_back(cloud[index].x, cloud[index].y);
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
}  // namespace bbox
}  // namespace object_detector
