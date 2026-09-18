#pragma once
#include "morai_path_tracking/controllers/lateral/mpc/types.hpp"
#include <cstddef>
#include <vector>

namespace morai_mpc {
enum class CoordinateFrame { Global, Vehicle };
struct Projection {
  bool valid{false}; std::size_t segment{0}; double ratio{0.0}, s{0.0};
  Point2d point; double lateral_error{0.0}, heading_error{0.0};
  ReferencePoint reference;
};

struct ProgressProjection {
  Projection projection;
  double unwrapped_s{0.0};
  int lap{0};
  bool relocalized{false};
  bool backtrack_prevented{false};
  bool jump_detected{false};
};

struct PathProgressConfig {
  double backward_window{2.0};
  double forward_window{35.0};
  double relocalize_distance{8.0};
  double teleport_distance{12.0};
  double jump_threshold{10.0};
  double low_speed_forward_window{3.0};
  double low_speed_threshold{0.3};
};

class ReferencePath {
 public:
  ReferencePath() = default;
  explicit ReferencePath(const std::vector<Point2d>& points, bool closed = false,
                         CoordinateFrame frame = CoordinateFrame::Global);
  bool empty() const { return points_.empty(); }
  bool closed() const { return closed_; }
  CoordinateFrame frame() const { return frame_; }
  double length() const { return length_; }
  const std::vector<ReferencePoint>& points() const { return points_; }
  Projection project(const Pose2d& pose) const;
  Projection projectHeadingAware(const Pose2d& pose, double distance_tolerance = 1.0) const;
  Projection projectWindow(const Pose2d& pose, double center_unwrapped_s,
                           double backward, double forward,
                           double* projected_unwrapped_s) const;
  ReferencePoint sample(double s) const;
  ReferencePath resample(double ds) const;
  ReferencePath toGlobal(const Pose2d& vehicle_pose) const;
 private:
  std::vector<ReferencePoint> points_; bool closed_{false}; double length_{0.0};
  CoordinateFrame frame_{CoordinateFrame::Global};
};

class PathProgressTracker {
 public:
  explicit PathProgressTracker(PathProgressConfig config = {});
  ProgressProjection project(const ReferencePath& path, const Pose2d& pose, double speed);
  void reset();
  bool initialized() const { return initialized_; }
  double unwrappedS() const { return unwrapped_s_; }
 private:
  std::uint64_t signature(const ReferencePath& path) const;
  PathProgressConfig config_;
  bool initialized_{false};
  double unwrapped_s_{0.0};
  Pose2d last_pose_;
  std::uint64_t path_signature_{0};
};

ReferencePath makeStraight(double length, double spacing = 1.0);
ReferencePath makeArc(double radius, double angle, double spacing = 0.5);
ReferencePath makeCircle(double radius, double spacing = 0.5);
ReferencePath makeSCurve(double length, double amplitude, double spacing = 0.5);
}  // namespace morai_mpc
