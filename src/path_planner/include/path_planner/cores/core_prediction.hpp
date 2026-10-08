#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#ifndef STIER_PLANNING_COMMON_TYPES_HPP
#define STIER_PLANNING_COMMON_TYPES_HPP
namespace stier {

// Standalone calculation contracts, NOT replacements for Team-Stier ROS messages.
// All positions and velocity components are in the map frame, SI units.
// Object pose is the geometric box CENTER; ego pose is the REAR AXLE center.
struct Vec2 { double x = 0.0; double y = 0.0; };
struct Pose2 { Vec2 position; double yaw_rad = 0.0; };
enum class ObjectKind { Unknown, Vehicle, Pedestrian, Static };
enum class MotionModel { ConstantVelocity, ConstantTurnRate };

struct PredictedObject {
  std::uint32_t id = 0;
  ObjectKind kind = ObjectKind::Unknown;
  MotionModel model = MotionModel::ConstantVelocity;
  Pose2 pose;
  Vec2 velocity_mps;
  double length_m = 0.0;
  double width_m = 0.0;
  double radius_error_m = 0.0;  // Separate from the physical dimensions.
  double source_stamp_sec = 0.0;
  // Bounds deviation from linear interpolation within one prediction interval.
  double interpolation_acceleration_mps2 = 0.0;
};

constexpr std::size_t kPredictionSteps = 10;
struct Predictions {
  double reference_stamp_sec = 0.0;
  double step_sec = 0.5;
  std::vector<PredictedObject> current;  // t = 0, not future[0].
  // Time-major; future[0] = t0+dt, future[9] = t0+10*dt.
  // The same IDs/order must be present at every time slice.
  std::array<std::vector<PredictedObject>, kPredictionSteps> future;
};

}  // namespace stier
#endif

#ifndef STIER_PLANNING_COMMON_GEOMETRY_HPP
#define STIER_PLANNING_COMMON_GEOMETRY_HPP
namespace stier {
namespace geometry {
constexpr double kPi = 3.14159265358979323846;
inline Vec2 add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 mul(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double norm(Vec2 a) { return std::hypot(a.x, a.y); }
inline double wrap(double a) { return std::remainder(a, 2.0 * kPi); }
inline Vec2 unit(double a) { return {std::cos(a), std::sin(a)}; }
inline Vec2 normal(double a) { return {-std::sin(a), std::cos(a)}; }
inline Vec2 rotate(Vec2 p, double a) {
  const auto u = unit(a);
  return {u.x * p.x - u.y * p.y, u.y * p.x + u.x * p.y};
}
inline bool finite(Vec2 p) { return std::isfinite(p.x) && std::isfinite(p.y); }
inline bool finite(Pose2 p) { return finite(p.position) && std::isfinite(p.yaw_rad); }
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline Vec2 lerp(Vec2 a, Vec2 b, double t) { return add(a, mul(sub(b, a), t)); }
inline double yawLerp(double a, double b, double t) { return wrap(a + wrap(b - a) * t); }
inline double clamp(double a, double lo, double hi) { return std::max(lo, std::min(a, hi)); }

}  // namespace geometry
}  // namespace stier
#endif

namespace stier {

struct TrackedObject {
  std::uint32_t id = 0;
  ObjectKind kind = ObjectKind::Unknown;
  Pose2 pose;
  Vec2 velocity_mps;  // Absolute ground velocity, NOT relative velocity.
  // A valid upstream velocity is a precondition; missing-input policy is external.
  double stamp_sec = 0.0;  // State epoch, never receipt wall-clock time.
  double length_m = 0.0;
  double width_m = 0.0;
  // Optional upstream turn rate. No tracking or velocity estimation in this core.
  bool turn_rate_available = false;
  double turn_rate_radps = 0.0;
  // Engineering error bounds supplied by caller, NOT calibrated confidence.
  double position_error_m = 0.0;
  double velocity_error_mps = 0.0;
  double unmodeled_acceleration_mps2 = 0.0;
};

struct PredictionParameters {
  double step_sec = 0.5;
};

// Motion-model propagation only; no tracker, velocity estimator, map snapping,
// ROS, filesystem, wall-clock dependence, or global-path dependence.
// CV baseline: Scholler et al., arXiv:1903.07933.
// Constant-turn integration: Autoware.Auto motion-model-design.html (CVTR).
class PredictionCore {
 public:
  explicit PredictionCore(PredictionParameters parameters = {}) : parameters_(parameters) {
    if (!std::isfinite(parameters_.step_sec) || parameters_.step_sec <= 0.0 || parameters_.step_sec > 2.0)
      throw std::invalid_argument("invalid prediction parameters");
  }

  Predictions process(const std::vector<TrackedObject>& objects_map, double reference_stamp_sec) const {
    if (!std::isfinite(reference_stamp_sec) || reference_stamp_sec < 0.0)
      throw std::invalid_argument("invalid reference epoch");
    Predictions result;
    result.reference_stamp_sec = reference_stamp_sec;
    result.step_sec = parameters_.step_sec;
    result.current.reserve(objects_map.size());
    for (auto& slice : result.future) slice.reserve(objects_map.size());
    for (const auto& o : objects_map) {
      const double age = reference_stamp_sec - o.stamp_sec;
      // Mathematical domain checks only: no stale-input threshold, ID health,
      // reception state, or unknown-velocity policy. Time propagation is forward.
      if (!geometry::finite(o.pose) || !geometry::finite(o.velocity_mps) ||
          !std::isfinite(o.stamp_sec) || o.stamp_sec < 0.0 || age < 0.0 ||
          !std::isfinite(o.length_m) || o.length_m <= 0.0 ||
          !std::isfinite(o.width_m) || o.width_m <= 0.0 ||
          !nonnegative(o.position_error_m) || !nonnegative(o.velocity_error_mps) ||
          !nonnegative(o.unmodeled_acceleration_mps2) ||
          (o.turn_rate_available && (!std::isfinite(o.turn_rate_radps) ||
             std::abs(o.turn_rate_radps)*parameters_.step_sec >= geometry::kPi)))
        throw std::invalid_argument("tracked object violates map/absolute-velocity calculation contract");
      result.current.push_back(propagate(o,age));
      for (std::size_t k=0; k<kPredictionSteps; ++k)
        result.future[k].push_back(propagate(o,age+parameters_.step_sec*static_cast<double>(k+1)));
    }
    return result;
  }

  // Samples a valid prediction within its horizon, with no extrapolation.
  // Current state is retained so [0, dt] is covered too.
  static PredictedObject sample(const Predictions& predictions, std::size_t object_index, double time_sec) {
    const double horizon = predictions.step_sec * kPredictionSteps;
    if (!std::isfinite(time_sec) || time_sec < 0.0 || time_sec > horizon + 1e-9 ||
        !std::isfinite(predictions.step_sec) || predictions.step_sec <= 0.0)
      throw std::invalid_argument("prediction sampling outside horizon");
    const double scaled = std::min(time_sec,horizon)/predictions.step_sec;
    const std::size_t lo = std::min(static_cast<std::size_t>(scaled),kPredictionSteps-1);
    const auto& a = (lo==0 ? predictions.current : predictions.future[lo-1]).at(object_index);
    const auto& b = predictions.future[lo].at(object_index);
    const double u = scaled - static_cast<double>(lo);
    if (a.id != b.id) throw std::invalid_argument("prediction ID/time-slice mismatch");
    PredictedObject out = a;
    out.pose.position = geometry::lerp(a.pose.position,b.pose.position,u);
    out.pose.yaw_rad = geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u);
    out.velocity_mps = geometry::lerp(a.velocity_mps,b.velocity_mps,u);
    // Maximum interpolation error for a twice differentiable trajectory:
    // |p(t)-chord(t)| <= A*h^2/8. Included even at knots so swept intervals
    // cannot lose the allowance between samples.
    out.radius_error_m = std::max(a.radius_error_m,b.radius_error_m) +
      std::max(a.interpolation_acceleration_mps2,b.interpolation_acceleration_mps2)*
      predictions.step_sec*predictions.step_sec/8.0;
    return out;
  }

 private:
  PredictionParameters parameters_;
  static bool nonnegative(double x) { return std::isfinite(x) && x >= 0.0; }
  static double sinc(double x) {
    if (std::abs(x) < 1e-4) { const double z=x*x; return 1.0-z/6.0+z*z/120.0; }
    return std::sin(x)/x;
  }
  static PredictedObject propagate(const TrackedObject& o, double dt) {
    PredictedObject p;
    p.id=o.id; p.kind=o.kind; p.pose=o.pose; p.velocity_mps=o.velocity_mps;
    p.length_m=o.length_m; p.width_m=o.width_m; p.source_stamp_sec=o.stamp_sec;
    const double omega = o.kind==ObjectKind::Vehicle && o.turn_rate_available ? o.turn_rate_radps : 0.0;
    p.model = omega==0.0 ? MotionModel::ConstantVelocity : MotionModel::ConstantTurnRate;
    const double half_angle=omega*dt/2.0;
    p.pose.position=geometry::add(o.pose.position,geometry::mul(geometry::rotate(o.velocity_mps,half_angle),dt*sinc(half_angle)));
    p.pose.yaw_rad=geometry::wrap(o.pose.yaw_rad+omega*dt);
    p.velocity_mps=geometry::rotate(o.velocity_mps,omega*dt);
    p.radius_error_m=o.position_error_m+o.velocity_error_mps*dt+0.5*o.unmodeled_acceleration_mps2*dt*dt;
    p.interpolation_acceleration_mps2=geometry::norm(o.velocity_mps)*std::abs(omega)+o.unmodeled_acceleration_mps2;
    if (!geometry::finite(p.pose) || !geometry::finite(p.velocity_mps) || !nonnegative(p.radius_error_m) ||
        !nonnegative(p.interpolation_acceleration_mps2))
      throw std::overflow_error("nonfinite prediction arithmetic");
    return p;
  }
};
}  // namespace stier

#include <path_planner/libraries/types.hpp>
#include <tuple>

namespace path_planner {
// base_link 관측 Objects를 stier::PredictionCore로 0.5 s 간격 10개 시점까지 전파하고 route·ego를 넘긴다.
class CorePrediction {
 public:
  using Result = std::tuple<PredictedObjects, Route, nav_msgs::Odometry>;

  Result process(const interfaces::Objects& objects, const Route& route, const nav_msgs::Odometry& ego) const {
    const double stamp_sec = objects.header.stamp.toSec();
    std::vector<stier::TrackedObject> tracked;
    tracked.reserve(objects.objects.size());
    for (const auto& object : objects.objects) {
      stier::TrackedObject sample;
      sample.id = object.id;
      sample.pose.position = {object.position.x, object.position.y};
      sample.pose.yaw_rad = yawOf(object.bbox_pose.orientation);
      if (object.velocity_available) sample.velocity_mps = {object.velocity.x, object.velocity.y};
      sample.stamp_sec = stamp_sec;
      sample.length_m = object.bbox_size.x;
      sample.width_m = object.bbox_size.y;
      tracked.push_back(sample);
    }
    const auto result = core_.process(tracked, stamp_sec);

    PredictedObjects predictions;
    predictions.fill(objects);
    for (std::size_t i = 0; i < predictions.size(); ++i) {
      for (std::size_t j = 0; j < tracked.size(); ++j) {
        const auto& state = result.future[i][j];
        auto& predicted = predictions[i].objects[j];
        predicted.position.x = state.pose.position.x;
        predicted.position.y = state.pose.position.y;
        predicted.velocity.x = state.velocity_mps.x;
        predicted.velocity.y = state.velocity_mps.y;
        predicted.bbox_pose.position = predicted.position;
        // bbox를 관측 base_link +z축으로 yaw 변화량만큼 회전한다.
        const double half_angle_rad = 0.5 * stier::geometry::wrap(state.pose.yaw_rad - tracked[j].pose.yaw_rad);
        const double sin_half = std::sin(half_angle_rad), cos_half = std::cos(half_angle_rad);
        const auto& q = objects.objects[j].bbox_pose.orientation;
        auto& predicted_q = predicted.bbox_pose.orientation;
        predicted_q.x = cos_half * q.x - sin_half * q.y;
        predicted_q.y = cos_half * q.y + sin_half * q.x;
        predicted_q.z = cos_half * q.z + sin_half * q.w;
        predicted_q.w = cos_half * q.w - sin_half * q.z;
      }
    }
    return std::make_tuple(predictions, route, ego);
  }

 private:
  static double yawOf(const geometry_msgs::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  stier::PredictionCore core_;
};
}  // namespace path_planner
