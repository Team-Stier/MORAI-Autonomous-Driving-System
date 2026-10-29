#pragma once
#include <interfaces/Objects.h>
#include <interfaces/Path.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace estop {
struct EstopOverlapParameters {
  // Ego footprint around each path pose (rear axle).
  double vehicle_front_m = 3.845008, vehicle_rear_m = 0.79, vehicle_half_width_m = 1.080418;
  // Spatial tolerance: the ego footprint grows by these margins and every object box by object_margin_m.
  double longitudinal_margin_m = 0.5, lateral_margin_m = 0.3, object_margin_m = 0.2;
  // Temporal tolerance: an object is tested over [t - time_tolerance_sec, t + time_tolerance_sec] around the
  // time t the ego reaches each pose, at time_samples evenly spaced instants.
  double time_tolerance_sec = 0.5;
  int time_samples = 5;
  // Part of the path that is tested.
  double horizon_sec = 3.0, horizon_m = 40.0;
  // An object is extrapolated with its measured velocity only from this speed on; slower ones stay where seen.
  double minimum_object_speed_mps = 0.3;
  // Speed floor for poses without a timed profile, so a slow segment still advances in time.
  double minimum_path_speed_mps = 0.5;
  // Objects whose centre is behind the ego rear (base_link x < -vehicle_rear_m) are not tested.
  bool ignore_behind = true;
  // A raised stop clears once the path has stayed free of objects this long.
  double release_after_sec = 1.0;
};

// True while the local path, travelled with its speed profile, meets an object; false once it has stayed clear
// for release_after_sec.
class EstopOverlap {
 public:
  explicit EstopOverlap(EstopOverlapParameters parameters) : parameters_(parameters) {}

  std_msgs::Bool process(const interfaces::Path& path, const interfaces::Objects& objects,
                         const tf2_msgs::TFMessage& transforms) {
    std_msgs::Bool output;
    const auto match = std::find_if(transforms.transforms.begin(), transforms.transforms.end(),
      [](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
      });
    if (match == transforms.transforms.end()) {   // the path cannot be placed in the map: keep the state
      output.data = stopping_;
      return output;
    }
    tf2::Transform map_from_base;
    tf2::fromMsg(match->transform, map_from_base);
    const double now = path.header.stamp.toSec();
    if (now < last_overlap_sec_) last_overlap_sec_ = now;   // a rewound clock restarts the release timer
    if (overlaps(path, objects, map_from_base)) {
      stopping_ = true;
      last_overlap_sec_ = now;
    } else if (stopping_ && now - last_overlap_sec_ >= parameters_.release_after_sec) {
      stopping_ = false;
    }
    output.data = stopping_;
    return output;
  }

 private:
  struct Box {
    double x, y, heading, half_length, half_width;
  };
  struct Pose {
    double x, y, heading, time;
  };

  static bool separated(const Box& a, const Box& b, double axis_x, double axis_y) {
    const auto radius = [&](const Box& box) {
      const double c = std::cos(box.heading), s = std::sin(box.heading);
      return box.half_length * std::abs(c * axis_x + s * axis_y) +
             box.half_width * std::abs(-s * axis_x + c * axis_y);
    };
    const double distance = std::abs((b.x - a.x) * axis_x + (b.y - a.y) * axis_y);
    return distance > radius(a) + radius(b);
  }

  static bool overlap(const Box& a, const Box& b) {
    for (const auto& box : {a, b}) {
      const double c = std::cos(box.heading), s = std::sin(box.heading);
      if (separated(a, b, c, s) || separated(a, b, -s, c)) return false;
    }
    return true;
  }

  // Map poses of the reachable part of the path with the time the ego reaches each of them.
  std::vector<Pose> timedPoses(const interfaces::Path& path, const tf2::Transform& map_from_base) const {
    const bool timed = std::any_of(path.points.begin(), path.points.end(),
      [](const interfaces::PathPoint& point) { return !point.time_from_start.isZero(); });
    std::vector<Pose> poses;
    double time = 0.0, distance = 0.0;
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto& point = path.points[i];
      tf2::Transform base_pose;
      tf2::fromMsg(point.pose.pose, base_pose);
      const tf2::Transform map_pose = map_from_base * base_pose;
      const double x = map_pose.getOrigin().x(), y = map_pose.getOrigin().y();
      if (!poses.empty()) {
        const double step = std::hypot(x - poses.back().x, y - poses.back().y);
        distance += step;
        if (timed) {
          time = point.time_from_start.toSec();
        } else {
          const auto speed = [&](std::size_t k) {
            const double v = path.points[k].velocity;
            return std::isfinite(v) ? std::max(v, parameters_.minimum_path_speed_mps) : parameters_.minimum_path_speed_mps;
          };
          time += step / (0.5 * (speed(i - 1) + speed(i)));
        }
      }
      if (time > parameters_.horizon_sec || distance > parameters_.horizon_m) break;
      poses.push_back({x, y, tf2::getYaw(map_pose.getRotation()), time});
      // Without a timed profile a zero speed is where the ego stops; the rest of the path is not reached.
      if (!timed && i > 0 && std::isfinite(point.velocity) && point.velocity <= 1e-3) break;
    }
    return poses;
  }

  bool overlaps(const interfaces::Path& path, const interfaces::Objects& objects,
                const tf2::Transform& map_from_base) const {
    const auto poses = timedPoses(path, map_from_base);
    if (poses.empty()) return false;
    const tf2::Transform base_from_map = map_from_base.inverse();
    const auto& p = parameters_;
    const double center_offset = 0.5 * (p.vehicle_front_m - p.vehicle_rear_m);
    const int samples = std::max(1, p.time_samples);
    for (const auto& object : objects.objects) {
      const tf2::Vector3 local = base_from_map * tf2::Vector3(object.position.x, object.position.y, object.position.z);
      if (p.ignore_behind && local.x() < -p.vehicle_rear_m) continue;
      tf2::Quaternion orientation;
      tf2::fromMsg(object.bbox_pose.orientation, orientation);
      const bool moving = object.velocity_available &&
          std::hypot(object.velocity.x, object.velocity.y) >= p.minimum_object_speed_mps;
      for (const auto& pose : poses) {
        const Box ego{pose.x + center_offset * std::cos(pose.heading), pose.y + center_offset * std::sin(pose.heading),
                      pose.heading, 0.5 * (p.vehicle_front_m + p.vehicle_rear_m) + p.longitudinal_margin_m,
                      p.vehicle_half_width_m + p.lateral_margin_m};
        for (int k = 0; k < samples; ++k) {
          const double t = samples == 1 ? pose.time :
              pose.time - p.time_tolerance_sec + 2.0 * p.time_tolerance_sec * k / (samples - 1);
          const double dt = moving ? t : 0.0;
          const Box box{object.position.x + object.velocity.x * dt, object.position.y + object.velocity.y * dt,
                        tf2::getYaw(orientation), 0.5 * object.bbox_size.x + p.object_margin_m,
                        0.5 * object.bbox_size.y + p.object_margin_m};
          if (overlap(ego, box)) return true;
          if (!moving) break;   // a still object is the same at every sample
        }
      }
    }
    return false;
  }

  EstopOverlapParameters parameters_;
  bool stopping_ = false;
  double last_overlap_sec_ = -std::numeric_limits<double>::infinity();
};
}  // namespace estop
