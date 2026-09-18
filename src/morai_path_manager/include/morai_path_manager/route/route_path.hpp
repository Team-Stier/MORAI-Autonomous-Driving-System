#ifndef MORAI_PATH_MANAGER_ROUTE_PATH_HPP_
#define MORAI_PATH_MANAGER_ROUTE_PATH_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace morai_path_manager {

struct RoutePoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct RouteLoadOptions {
  double duplicate_tolerance_m = 1e-6;
  double max_point_spacing_m = 1.0;
  bool flatten_z = true;
  bool require_closed_loop = true;
};

struct NearestResult {
  std::size_t index = 0U;
  double distance_m = 0.0;
};

class RoutePath {
 public:
  static RoutePath loadFromFile(const std::string& path,
                                const RouteLoadOptions& options);

  std::size_t rawPointCount() const noexcept;
  bool closedLoop() const noexcept;
  double totalLength() const noexcept;
  double maxSegmentLength() const noexcept;
  const std::vector<RoutePoint>& points() const noexcept;

  std::vector<RoutePoint> globalPoints() const;
  std::vector<RoutePoint> extractForward(std::size_t start_index,
                                         double length_m) const;
  std::vector<RoutePoint> extractForwardPoints(
      std::size_t start_index, std::size_t point_count) const;
  NearestResult nearest(double x, double y) const;
  NearestResult nearestInWindow(double x, double y,
                                std::size_t previous_index,
                                double backward_m,
                                double forward_m) const;
  NearestResult nearestWithContinuity(double x, double y,
                                      std::size_t previous_index,
                                      double backward_m,
                                      double forward_m,
                                      double reacquire_distance_m) const;

 private:
  RoutePath() = default;
  double forwardArcDistance(std::size_t from, std::size_t to) const;

  std::size_t raw_point_count_ = 0U;
  bool closed_loop_ = false;
  double total_length_m_ = 0.0;
  double max_segment_length_m_ = 0.0;
  std::vector<RoutePoint> points_;
  std::vector<double> segment_lengths_;
  std::vector<double> cumulative_distances_;
};

}  // namespace morai_path_manager

#endif  // MORAI_PATH_MANAGER_ROUTE_PATH_HPP_
