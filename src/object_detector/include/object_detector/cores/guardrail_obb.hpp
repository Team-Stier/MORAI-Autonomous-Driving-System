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
#include <lanelet2_core/LaneletMap.h>
#include <map>
#include <string>

namespace object_detector {
struct GuardrailObbParameters {
  lanelet::LaneletMapPtr map;
  double maximum_width_m = 0.3;
  double maximum_height_m = 1.4;
  double minimum_fragment_length_m = 0.7;
  double maximum_fragment_length_m = 3.0;
  double minimum_rail_length_m = 6.0;
  double maximum_boundary_distance_m = 1.3;
  double maximum_heading_error_rad = 0.3;
  double maximum_gap_m = 2.5;
  double maximum_offset_difference_m = 0.6;
  double memory_sec = 2.0;
  double evidence_sec = 0.5;
};
class GuardrailObb {
 public:
  explicit GuardrailObb(GuardrailObbParameters parameters) : parameters_(std::move(parameters)) {
    for (const auto& lane : parameters_.map->laneletLayer) {
      for (const auto& line : {lane.leftBound(), lane.rightBound()}) {
        if (line.attributeOr("subtype", std::string{}) != "solid") continue;
        if (boundaries_.count(line.id())) continue;
        Boundary boundary;
        double station = 0;
        for (std::size_t i = 1; i < line.size(); ++i) {
          const Point a(line[i-1].x(), line[i-1].y()), b(line[i].x(), line[i].y());
          const double length = (b-a).norm();
          boundary.push_back({a, b, station, length});
          station += length;
        }
        boundaries_.emplace(line.id(), std::move(boundary));
      }
    }
  }
  std::tuple<interfaces::Objects, interfaces::CompetitionVehicleStatus> process(
      const Clusters& clusters,
      const interfaces::CompetitionVehicleStatus& competition) {
    interfaces::Objects output;
    for (const auto& cluster : clusters) output.objects.push_back(makeObb(cluster));
    const auto suppressed = guardrails(output.objects, competition.header.stamp.toSec());
    interfaces::Objects retained;
    for (std::size_t i = 0; i < output.objects.size(); ++i)
      if (!suppressed[i]) retained.objects.push_back(std::move(output.objects[i]));
    return std::make_tuple(std::move(retained), competition);
  }

 private:
  using Point = lanelet::BasicPoint2d;
  struct Segment { Point a, b; double station, length; };
  using Boundary = std::vector<Segment>;
  struct Fragment {
    std::size_t index;
    lanelet::Id boundary;
    double begin, end, offset;
  };
  struct Evidence { Fragment fragment; double time; };
  struct Rail {
    lanelet::Id boundary;
    double begin, end, offset, time;
  };

  std::vector<bool> guardrails(const std::vector<interfaces::Object>& objects, double time) {
    rails_.erase(std::remove_if(rails_.begin(), rails_.end(), [&](const Rail& rail) {
      return time-rail.time > parameters_.memory_sec || time < rail.time;
    }), rails_.end());
    evidence_.erase(std::remove_if(evidence_.begin(), evidence_.end(), [&](const Evidence& evidence) {
      return time-evidence.time > parameters_.evidence_sec || time < evidence.time;
    }), evidence_.end());
    std::vector<Fragment> fragments;
    for (const auto& evidence : evidence_) {
      auto fragment = evidence.fragment;
      fragment.index = objects.size();
      fragments.push_back(fragment);
    }
    const std::size_t current_begin = fragments.size();
    for (std::size_t index = 0; index < objects.size(); ++index) {
      const auto& object = objects[index];
      if (object.bbox_size.x < parameters_.minimum_fragment_length_m ||
          object.bbox_size.y > parameters_.maximum_width_m ||
          object.bbox_size.z > parameters_.maximum_height_m ||
          (object.bbox_size.x > parameters_.maximum_fragment_length_m &&
           object.bbox_size.x < parameters_.minimum_rail_length_m)) continue;
      const Point center(object.position.x, object.position.y);
      const double yaw = 2*std::atan2(object.bbox_pose.orientation.z, object.bbox_pose.orientation.w);
      double closest = parameters_.maximum_boundary_distance_m;
      Fragment fragment{};
      bool matched = false;
      for (const auto& entry : boundaries_) for (const auto& segment : entry.second) {
        const Point direction = (segment.b-segment.a)/segment.length;
        const double along = (center-segment.a).dot(direction);
        const double bounded = std::max(0.0, std::min(segment.length, along));
        const double distance = (center-segment.a-bounded*direction).norm();
        const double alignment = std::abs(direction.dot(Point(std::cos(yaw), std::sin(yaw))));
        if (distance > closest || alignment < std::cos(parameters_.maximum_heading_error_rad)) continue;
        closest = distance;
        const double station = segment.station+bounded;
        fragment = {index, entry.first, station-object.bbox_size.x/2,
                    station+object.bbox_size.x/2,
                    direction.x()*(center.y()-segment.a.y())-direction.y()*(center.x()-segment.a.x())};
        matched = true;
      }
      if (matched) {
        fragments.push_back(fragment);
      }
    }
    for (std::size_t i = current_begin; i < fragments.size(); ++i) {
      if (fragments[i].end-fragments[i].begin > parameters_.maximum_fragment_length_m) continue;
      const auto count = std::count_if(fragments.begin()+current_begin, fragments.end(), [&](const Fragment& other) {
        return other.boundary == fragments[i].boundary &&
            other.end-other.begin <= parameters_.maximum_fragment_length_m;
      });
      // A single moving vehicle face must not paint a static rail along its trajectory.
      if (count >= 2) evidence_.push_back({fragments[i], time});
    }
    std::sort(fragments.begin(), fragments.end(), [](const Fragment& a, const Fragment& b) {
      return std::tie(a.boundary, a.begin, a.index) < std::tie(b.boundary, b.begin, b.index);
    });
    std::vector<bool> suppressed(objects.size(), false);
    for (std::size_t begin = 0; begin < fragments.size();) {
      std::size_t end = begin+1;
      double far = fragments[begin].end;
      double offset_min = fragments[begin].offset, offset_max = offset_min;
      while (end < fragments.size() && fragments[end].boundary == fragments[begin].boundary &&
             fragments[end].begin-far <= parameters_.maximum_gap_m &&
             std::max(offset_max, fragments[end].offset)-std::min(offset_min, fragments[end].offset)
                 <= parameters_.maximum_offset_difference_m) {
        far = std::max(far, fragments[end].end);
        offset_min = std::min(offset_min, fragments[end].offset);
        offset_max = std::max(offset_max, fragments[end].offset);
        ++end;
      }
      const double offset = (offset_min+offset_max)/2;
      bool short_fragments = true, long_rail = false;
      for (std::size_t i = begin; i < end; ++i) {
        const double length = fragments[i].end-fragments[i].begin;
        short_fragments = short_fragments && length <= parameters_.maximum_fragment_length_m;
        long_rail = long_rail || length >= parameters_.minimum_rail_length_m;
      }
      bool confirmed = long_rail || (short_fragments && end-begin >= 3 &&
          far-fragments[begin].begin >= parameters_.minimum_rail_length_m);
      for (const auto& rail : rails_) {
        if (rail.boundary == fragments[begin].boundary &&
            fragments[begin].begin <= rail.end+parameters_.maximum_gap_m &&
            far >= rail.begin-parameters_.maximum_gap_m &&
            std::abs(offset-rail.offset) <= parameters_.maximum_offset_difference_m) confirmed = true;
      }
      const bool observed_now = std::any_of(fragments.begin()+begin, fragments.begin()+end,
          [&](const Fragment& fragment) { return fragment.index < objects.size(); });
      if (confirmed && observed_now) {
        // Only observed continuous intervals extend a previously measured rail.
        Rail rail{fragments[begin].boundary, fragments[begin].begin, far, offset, time};
        for (auto it = rails_.begin(); it != rails_.end();) {
          if (it->boundary == rail.boundary && it->begin <= rail.end+parameters_.maximum_gap_m &&
              it->end >= rail.begin-parameters_.maximum_gap_m &&
              std::abs(it->offset-rail.offset) <= parameters_.maximum_offset_difference_m) {
            rail.begin = std::min(rail.begin, it->begin);
            rail.end = std::max(rail.end, it->end);
            it = rails_.erase(it);
          } else ++it;
        }
        rails_.push_back(rail);
        for (std::size_t i = begin; i < end; ++i)
          if (fragments[i].index < objects.size()) suppressed[fragments[i].index] = true;
      }
      begin = end;
    }
    return suppressed;
  }

  GuardrailObbParameters parameters_;
  std::map<lanelet::Id, Boundary> boundaries_;
  std::vector<Rail> rails_;
  std::vector<Evidence> evidence_;
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
