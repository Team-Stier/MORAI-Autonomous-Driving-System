#include "morai_path_manager/route/route_path.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace morai_path_manager {
namespace {

double squaredDistance(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return dx * dx + dy * dy;
}

double pointDistance(const RoutePoint& first, const RoutePoint& second) {
  return std::sqrt(squaredDistance(first.x, first.y, second.x, second.y));
}

bool whitespaceOnly(const std::string& line) {
  return line.find_first_not_of(" \t\r\n") == std::string::npos;
}

void validateOptions(const RouteLoadOptions& options) {
  if (!std::isfinite(options.duplicate_tolerance_m) ||
      options.duplicate_tolerance_m <= 0.0) {
    throw std::invalid_argument(
        "duplicate_tolerance_m must be finite and positive");
  }
  if (!std::isfinite(options.max_point_spacing_m) ||
      options.max_point_spacing_m <= 0.0) {
    throw std::invalid_argument(
        "max_point_spacing_m must be finite and positive");
  }
  if (options.max_point_spacing_m <= options.duplicate_tolerance_m) {
    throw std::invalid_argument(
        "max_point_spacing_m must exceed duplicate_tolerance_m");
  }
}

}  // namespace

RoutePath RoutePath::loadFromFile(const std::string& path,
                                  const RouteLoadOptions& options) {
  validateOptions(options);
  if (path.empty()) {
    throw std::invalid_argument("route path file name is empty");
  }

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open route path file: " + path);
  }

  std::vector<RoutePoint> raw_points;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(input, line)) {
    ++line_number;
    if (whitespaceOnly(line)) {
      continue;
    }

    std::istringstream parser(line);
    RoutePoint point;
    std::string trailing;
    if (!(parser >> point.x >> point.y >> point.z) || (parser >> trailing)) {
      throw std::runtime_error("invalid route record at " + path + ":" +
                               std::to_string(line_number));
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      throw std::runtime_error("non-finite route record at " + path + ":" +
                               std::to_string(line_number));
    }
    raw_points.push_back(point);
  }

  if (raw_points.size() < 3U) {
    throw std::runtime_error("route must contain at least three records");
  }

  const bool explicit_closed =
      pointDistance(raw_points.front(), raw_points.back()) <=
      options.duplicate_tolerance_m;
  if (options.require_closed_loop && !explicit_closed) {
    throw std::runtime_error("route is not explicitly closed");
  }

  RoutePath route;
  route.raw_point_count_ = raw_points.size();
  route.closed_loop_ = explicit_closed;
  route.points_.reserve(raw_points.size());
  for (RoutePoint point : raw_points) {
    if (!route.points_.empty() &&
        pointDistance(route.points_.back(), point) <=
            options.duplicate_tolerance_m) {
      continue;
    }
    if (options.flatten_z) {
      point.z = 0.0;
    }
    route.points_.push_back(point);
  }
  if (route.closed_loop_ && route.points_.size() > 1U &&
      pointDistance(route.points_.front(), route.points_.back()) <=
          options.duplicate_tolerance_m) {
    route.points_.pop_back();
  }
  if (route.points_.size() < 3U) {
    throw std::runtime_error(
        "route must contain at least three distinct XY points");
  }

  const std::size_t segment_count =
      route.closed_loop_ ? route.points_.size() : route.points_.size() - 1U;
  route.segment_lengths_.reserve(segment_count);
  for (std::size_t index = 0U; index < segment_count; ++index) {
    const std::size_t next = (index + 1U) % route.points_.size();
    const double length = pointDistance(route.points_[index],
                                        route.points_[next]);
    if (!std::isfinite(length) ||
        length <= options.duplicate_tolerance_m) {
      throw std::runtime_error("route contains a zero-length XY segment");
    }
    if (length > options.max_point_spacing_m) {
      throw std::runtime_error(
          "route segment exceeds max_point_spacing_m");
    }
    route.segment_lengths_.push_back(length);
    route.total_length_m_ += length;
    route.max_segment_length_m_ =
        std::max(route.max_segment_length_m_, length);
    route.points_[index].yaw =
        std::atan2(route.points_[next].y - route.points_[index].y,
                   route.points_[next].x - route.points_[index].x);
  }
  if (!route.closed_loop_) {
    route.points_.back().yaw =
        route.points_[route.points_.size() - 2U].yaw;
  }

  route.cumulative_distances_.assign(route.points_.size(), 0.0);
  for (std::size_t index = 1U; index < route.points_.size(); ++index) {
    route.cumulative_distances_[index] =
        route.cumulative_distances_[index - 1U] +
        route.segment_lengths_[index - 1U];
  }
  return route;
}

std::size_t RoutePath::rawPointCount() const noexcept {
  return raw_point_count_;
}

bool RoutePath::closedLoop() const noexcept { return closed_loop_; }

double RoutePath::totalLength() const noexcept { return total_length_m_; }

double RoutePath::maxSegmentLength() const noexcept {
  return max_segment_length_m_;
}

const std::vector<RoutePoint>& RoutePath::points() const noexcept {
  return points_;
}

std::vector<RoutePoint> RoutePath::globalPoints() const {
  std::vector<RoutePoint> result = points_;
  if (closed_loop_) {
    result.push_back(points_.front());
  }
  return result;
}

std::vector<RoutePoint> RoutePath::extractForward(
    std::size_t start_index, double length_m) const {
  if (start_index >= points_.size()) {
    throw std::out_of_range("route start index is out of range");
  }
  if (!std::isfinite(length_m) || length_m <= 0.0) {
    throw std::invalid_argument("forward path length must be finite and positive");
  }

  const double target = std::min(length_m, total_length_m_);
  std::vector<RoutePoint> result;
  result.reserve(points_.size() + 1U);
  result.push_back(points_[start_index]);

  std::size_t current = start_index;
  std::size_t traversed = 0U;
  double distance = 0.0;
  while (distance + 1e-12 < target &&
         traversed < segment_lengths_.size()) {
    if (!closed_loop_ && current + 1U >= points_.size()) {
      break;
    }
    distance += segment_lengths_[current];
    current = closed_loop_ ? (current + 1U) % points_.size() : current + 1U;
    result.push_back(points_[current]);
    ++traversed;
  }
  return result;
}

std::vector<RoutePoint> RoutePath::extractForwardPoints(
    std::size_t start_index, std::size_t point_count) const {
  if (start_index >= points_.size()) {
    throw std::out_of_range("route start index is out of range");
  }
  if (point_count == 0U) {
    throw std::invalid_argument("forward path point_count must be positive");
  }
  if (point_count > points_.size()) {
    throw std::invalid_argument(
        "forward path point_count exceeds route point count");
  }

  std::vector<RoutePoint> result;
  result.reserve(point_count);
  std::size_t current = start_index;
  for (std::size_t offset = 0U; offset < point_count; ++offset) {
    result.push_back(points_[current]);
    if (!closed_loop_ && current + 1U >= points_.size()) {
      break;
    }
    current = closed_loop_ ? (current + 1U) % points_.size() : current + 1U;
  }
  return result;
}

NearestResult RoutePath::nearest(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    throw std::invalid_argument("nearest query must be finite");
  }

  std::size_t best_index = 0U;
  double best_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < points_.size(); ++index) {
    const double candidate =
        squaredDistance(x, y, points_[index].x, points_[index].y);
    if (candidate < best_squared) {
      best_squared = candidate;
      best_index = index;
    }
  }
  return {best_index, std::sqrt(best_squared)};
}

double RoutePath::forwardArcDistance(std::size_t from,
                                     std::size_t to) const {
  if (!closed_loop_) {
    return cumulative_distances_[to] - cumulative_distances_[from];
  }
  if (to >= from) {
    return cumulative_distances_[to] - cumulative_distances_[from];
  }
  return total_length_m_ -
         (cumulative_distances_[from] - cumulative_distances_[to]);
}

NearestResult RoutePath::nearestInWindow(
    double x, double y, std::size_t previous_index, double backward_m,
    double forward_m) const {
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(backward_m) || !std::isfinite(forward_m) ||
      backward_m < 0.0 || forward_m < 0.0) {
    throw std::invalid_argument("nearest-window query is invalid");
  }
  if (previous_index >= points_.size()) {
    throw std::out_of_range("previous route index is out of range");
  }

  std::size_t best_index = previous_index;
  double best_squared =
      squaredDistance(x, y, points_[previous_index].x,
                      points_[previous_index].y);
  for (std::size_t index = 0U; index < points_.size(); ++index) {
    bool eligible = false;
    if (closed_loop_) {
      eligible =
          forwardArcDistance(previous_index, index) <= forward_m + 1e-12 ||
          forwardArcDistance(index, previous_index) <= backward_m + 1e-12;
    } else {
      const double delta =
          cumulative_distances_[index] -
          cumulative_distances_[previous_index];
      eligible = delta <= forward_m + 1e-12 &&
                 delta >= -backward_m - 1e-12;
    }
    if (!eligible) {
      continue;
    }

    const double candidate =
        squaredDistance(x, y, points_[index].x, points_[index].y);
    if (candidate < best_squared) {
      best_squared = candidate;
      best_index = index;
    }
  }
  return {best_index, std::sqrt(best_squared)};
}

NearestResult RoutePath::nearestWithContinuity(
    double x, double y, std::size_t previous_index, double backward_m,
    double forward_m, double reacquire_distance_m) const {
  if (!std::isfinite(reacquire_distance_m) ||
      reacquire_distance_m <= 0.0) {
    throw std::invalid_argument(
        "reacquire distance must be finite and positive");
  }
  const NearestResult local = nearestInWindow(
      x, y, previous_index, backward_m, forward_m);
  return local.distance_m > reacquire_distance_m ? nearest(x, y) : local;
}

}  // namespace morai_path_manager
