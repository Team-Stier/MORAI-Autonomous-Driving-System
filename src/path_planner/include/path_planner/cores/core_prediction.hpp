#pragma once
#include <sensor_msgs/Imu.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <tuple>
#include <Eigen/Geometry>
#include <path_planner/libraries/types.hpp>

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
  // Native motion-model input; the ROS adapter derives it from velocity/acceleration.
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

}  // namespace stier

namespace path_planner {
using stier::Vec2;
using stier::Pose2;
using stier::ObjectKind;
using stier::MotionModel;
using stier::PredictedObject;
using stier::Predictions;
using stier::TrackedObject;
using stier::PredictionParameters;
using stier::kPredictionSteps;

// Motion-model propagation only; no tracker, velocity estimator, map snapping,
// ROS, filesystem, wall-clock dependence, or global-path dependence.
// CV baseline: Scholler et al., arXiv:1903.07933.
// Constant-turn integration: Autoware.Auto motion-model-design.html (CVTR).
class CorePrediction {
 private:
  struct Geometry {
    static constexpr double kPi = 3.14159265358979323846;
    static Vec2 add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
    static Vec2 sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
    static Vec2 mul(Vec2 a, double s) { return {a.x * s, a.y * s}; }
    static double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
    static double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
    static double norm(Vec2 a) { return std::hypot(a.x, a.y); }
    static double wrap(double a) { return std::remainder(a, 2.0 * kPi); }
    static Vec2 unit(double a) { return {std::cos(a), std::sin(a)}; }
    static Vec2 normal(double a) { return {-std::sin(a), std::cos(a)}; }
    static Vec2 rotate(Vec2 p, double a) {
      const auto u = unit(a);
      return {u.x * p.x - u.y * p.y, u.y * p.x + u.x * p.y};
    }
    static bool finite(Vec2 p) { return std::isfinite(p.x) && std::isfinite(p.y); }
    static bool finite(Pose2 p) { return finite(p.position) && std::isfinite(p.yaw_rad); }
    static double lerp(double a, double b, double t) { return a + (b - a) * t; }
    static Vec2 lerp(Vec2 a, Vec2 b, double t) { return add(a, mul(sub(b, a), t)); }
    static double yawLerp(double a, double b, double t) { return wrap(a + wrap(b - a) * t); }
    static double clamp(double a, double lo, double hi) { return std::max(lo, std::min(a, hi)); }

  };

 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
                            interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu, Route, PredictedObjects>;

  explicit CorePrediction(PredictionParameters parameters = {}) : parameters_(parameters) {}

  Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) const {
    std::vector<TrackedObject> tracked;
    tracked.reserve(objects.objects.size());
    for (const auto& object : objects.objects) {
      const auto& point = object.position;
      const auto& velocity = object.velocity;
      const auto& acceleration = object.acceleration;
      const auto& orientation = object.bbox_pose.orientation;
      TrackedObject input;
      input.id = object.id;
      input.kind = object.label == "vehicle" ? ObjectKind::Vehicle : ObjectKind::Unknown;
      input.pose.position = {point.x, point.y};
      input.pose.yaw_rad = std::atan2(2 * (orientation.w*orientation.z + orientation.x*orientation.y),
                                    1 - 2 * (orientation.y*orientation.y + orientation.z*orientation.z));
      input.velocity_mps = {velocity.x, velocity.y};
      input.stamp_sec = objects.header.stamp.toSec();
      input.length_m = object.bbox_size.x;
      input.width_m = object.bbox_size.y;
      const double speed_squared = velocity.x*velocity.x + velocity.y*velocity.y;
      input.turn_rate_radps = speed_squared == 0 ? 0 :
        (velocity.x*acceleration.y - velocity.y*acceleration.x) / speed_squared;
      tracked.push_back(input);
    }
    const auto calculated = predict(tracked, objects.header.stamp.toSec());
    PredictedObjects predictions;
    predictions.fill(objects);
    for (std::size_t k = 0; k < predictions.size(); ++k) {
      const double dt = parameters_.step_sec * (k + 1);
      for (std::size_t i = 0; i < objects.objects.size(); ++i) {
        const auto& input = objects.objects[i];
        const auto& predicted = calculated.future[k][i];
        auto& output = predictions[k].objects[i];
        output.position.x = predicted.pose.position.x; output.position.y = predicted.pose.position.y;
        output.position.z = input.position.z + input.velocity.z*dt;
        output.velocity.x = predicted.velocity_mps.x; output.velocity.y = predicted.velocity_mps.y;
        output.velocity.z = input.velocity.z;
        output.bbox_pose.position = output.position;
        const auto& bbox_q = input.bbox_pose.orientation;
        const Eigen::Quaterniond orientation = Eigen::Quaterniond(Eigen::AngleAxisd((tracked[i].kind == ObjectKind::Vehicle ?
                                               tracked[i].turn_rate_radps : 0)*dt,
                                              Eigen::Vector3d::UnitZ())) *
          Eigen::Quaterniond(bbox_q.w, bbox_q.x, bbox_q.y, bbox_q.z);
        output.bbox_pose.orientation.x = orientation.x(); output.bbox_pose.orientation.y = orientation.y();
        output.bbox_pose.orientation.z = orientation.z(); output.bbox_pose.orientation.w = orientation.w();
      }
    }
    return std::make_tuple(ego, competition, objects, tf, imu, route, predictions);
  }

 private:
  Predictions predict(const std::vector<TrackedObject>& objects_map, double reference_stamp_sec) const {
    Predictions result;
    result.reference_stamp_sec = reference_stamp_sec;
    result.step_sec = parameters_.step_sec;
    result.current.reserve(objects_map.size());
    for (auto& slice : result.future) slice.reserve(objects_map.size());
    for (const auto& o : objects_map) {
      const double age = reference_stamp_sec - o.stamp_sec;
      result.current.push_back(propagate(o,age));
      for (std::size_t k=0; k<kPredictionSteps; ++k)
        result.future[k].push_back(propagate(o,age+parameters_.step_sec*static_cast<double>(k+1)));
    }
    return result;
  }

 private:
  // Samples a valid prediction within its horizon, with no extrapolation.
  // Current state is retained so [0, dt] is covered too.
  static PredictedObject sample(const Predictions& predictions, std::size_t object_index, double time_sec) {
    const double horizon = predictions.step_sec * kPredictionSteps;
    const double scaled = std::min(time_sec,horizon)/predictions.step_sec;
    const std::size_t lo = std::min(static_cast<std::size_t>(scaled),kPredictionSteps-1);
    const auto& a = (lo==0 ? predictions.current : predictions.future[lo-1])[object_index];
    const auto& b = predictions.future[lo][object_index];
    const double u = scaled - static_cast<double>(lo);
    PredictedObject out = a;
    out.pose.position = Geometry::lerp(a.pose.position,b.pose.position,u);
    out.pose.yaw_rad = Geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u);
    out.velocity_mps = Geometry::lerp(a.velocity_mps,b.velocity_mps,u);
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
  static double sinc(double x) {
    if (std::abs(x) < 1e-4) { const double z=x*x; return 1.0-z/6.0+z*z/120.0; }
    return std::sin(x)/x;
  }
  static PredictedObject propagate(const TrackedObject& o, double dt) {
    PredictedObject p;
    p.id=o.id; p.kind=o.kind; p.pose=o.pose; p.velocity_mps=o.velocity_mps;
    p.length_m=o.length_m; p.width_m=o.width_m; p.source_stamp_sec=o.stamp_sec;
    const double omega = o.kind==ObjectKind::Vehicle ? o.turn_rate_radps : 0.0;
    p.model = omega==0.0 ? MotionModel::ConstantVelocity : MotionModel::ConstantTurnRate;
    const double half_angle=omega*dt/2.0;
    p.pose.position=Geometry::add(o.pose.position,Geometry::mul(Geometry::rotate(o.velocity_mps,half_angle),dt*sinc(half_angle)));
    p.pose.yaw_rad=Geometry::wrap(o.pose.yaw_rad+omega*dt);
    p.velocity_mps=Geometry::rotate(o.velocity_mps,omega*dt);
    p.radius_error_m=o.position_error_m+o.velocity_error_mps*dt+0.5*o.unmodeled_acceleration_mps2*dt*dt;
    p.interpolation_acceleration_mps2=Geometry::norm(o.velocity_mps)*std::abs(omega)+o.unmodeled_acceleration_mps2;
    return p;
  }
};
}  // namespace path_planner
