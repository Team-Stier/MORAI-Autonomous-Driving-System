#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <initializer_list>
#include <limits>
#include <map>
#include <tuple>

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

struct ReferencePoint {
  Vec2 position;
  // Signed lateral boundaries of the ALLOWED road corridor. Left is positive.
  // Caller decides lane permissions; the core never expands these boundaries.
  double min_l_m = -1.75;
  double max_l_m = 1.75;
};

struct EgoState {
  Pose2 pose;
  double stamp_sec = 0.0;
  // Measured forward rear-axle motion, NOT the desired cruise speed.
  double speed_mps = 0.0;
  double acceleration_mps2 = 0.0;
  double curvature_inv_m = 0.0;
};

struct VehicleShape {
  double front_m = 3.845008;  // Rear axle -> front bumper.
  double rear_m = 0.790;
  double width_m = 2.160835;
};

struct ProgressTime {
  double progress_m = 0.0;  // Reference station offset, NOT path arc length.
  double time_sec = 0.0;    // Relative to EgoState.stamp_sec (path start epoch).
  // Supplied assumption, not a speed chosen/optimized by the path core.
  // Piecewise-linear t(progress); every timing breakpoint is retained in output.
};

struct PathPoint {
  Pose2 pose;
  double progress_m = 0.0;
  double time_sec = 0.0;
  double curvature_inv_m = 0.0;
  double speed_mps = 0.0;
  double acceleration_mps2 = 0.0;
  // The planned overload uses cubic speed interpolation between samples:
  // endpoint v/a uniquely determine v(t); integrating it gives traveled arc
  // length. Position/yaw interpolate by that arc, NOT linearly by time.
  // The legacy schedule overload leaves v/a unused (zero).
  // Sampled geometry, NOT a control command. Retiming must preserve geometry
  // and its curvature; add samples at every speed-profile breakpoint. During
  // checkTimedPath, equal progress/pose with increasing time represents a wait.
};

struct PlanningConstraints {
  // Upstream road/mission speed ceiling. +infinity means only the configured
  // vehicle ceiling. Zero is a stop request, never a minimum moving speed.
  double maximum_speed_mps = std::numeric_limits<double>::infinity();
  // Reference station offset from the current ego projection to a stop LINE.
  // Before release_time_sec, the ENTIRE footprint must remain behind the line.
  // +infinity means no line. This is an input constraint, not signal detection.
  double stop_line_progress_m = std::numeric_limits<double>::infinity();
  double release_time_sec = std::numeric_limits<double>::infinity(); // relative to ego stamp
};

struct PathParameters {
  VehicleShape vehicle;
  double horizon_m = 25.0;
  double min_horizon_m = 0.5;  // Minimum geometric calculation extent, not stopping distance.
  double road_boundary_step_m = 0.25;  // Polygon discretization; every reference knot is retained.
  double dp_step_m = 5.0;
  double lateral_step_m = 0.5;
  double dp_slope_sample = 0.2;
  double output_step_m = 0.25;
  double corridor_half_width_m = 0.3;
  double obstacle_margin_m = 0.25;
  double road_margin_m = 0.05;
  double max_curvature_inv_m = 0.18;
  double max_lateral_accel_mps2 = 2.0;
  double max_speed_mps = 20.0;
  double max_lateral_slope = 0.8;
  double max_initial_heading_error_rad = 0.5;
  double max_initial_distance_m = 5.0;
  double reference_weight = 0.2;
  double dp_tracking_weight = 3.0;
  double slope_weight = 2.0;
  double bending_weight = 20.0;
  double jerk_weight = 5.0;
  double previous_weight = 0.5;
  double qp_rho = 10.0;
  double qp_tolerance = 1e-5;
  int qp_max_iterations = 3000;
  std::size_t max_lateral_samples = 41;
  // Numerical/engineering limits, to be calibrated with MORAI Ioniq5 control.
  // They are not assertions about physical braking capability or latency.
  double max_acceleration_mps2 = 2.0;
  double max_deceleration_mps2 = 5.0;
  double max_longitudinal_jerk_mps3 = 4.0;
  double speed_time_step_sec = 0.5;
  double trajectory_output_step_sec = 0.1;
  std::size_t speed_beam_width = 128;
  double speed_tracking_weight = 1.0;
  double acceleration_weight = 0.15;
  double longitudinal_jerk_weight = 0.05;
  // Integrated planning window; the legacy supplied-schedule horizon stays
  // unchanged. Caller must supply reference/prediction coverage for its speed.
  double trajectory_horizon_m = 100.0;
};

}  // namespace stier

namespace stier {
namespace geometry {
struct Box {
  Pose2 pose;
  double half_length = 0.0;
  double half_width = 0.0;
};
inline Box egoBox(Pose2 rear_axle, const VehicleShape& v) {
  rear_axle.position = add(rear_axle.position, mul(unit(rear_axle.yaw_rad), (v.front_m-v.rear_m)/2.0));
  return {rear_axle, (v.front_m + v.rear_m)/2.0, v.width_m/2.0};
}
inline Box objectBox(const PredictedObject& o, double margin) {
  return {o.pose, o.length_m/2.0 + o.radius_error_m + margin,
                  o.width_m/2.0 + o.radius_error_m + margin};
}
inline double extent(const Box& b, Vec2 axis) {
  return b.half_length * std::abs(dot(unit(b.pose.yaw_rad), axis)) +
         b.half_width * std::abs(dot(normal(b.pose.yaw_rad), axis));
}
inline std::array<Vec2, 4> corners(const Box& b) {
  const Vec2 x = mul(unit(b.pose.yaw_rad), b.half_length);
  const Vec2 y = mul(normal(b.pose.yaw_rad), b.half_width);
  return {{add(b.pose.position, add(x,y)), add(b.pose.position,sub(x,y)),
           sub(b.pose.position,add(x,y)), add(b.pose.position,sub(y,x))}};
}

// Closed segment/rectangle intersection, including contact. Testing boundary
// segments catches inward notches even when all four vehicle corners are inside.
inline bool segmentIntersectsBox(Vec2 a, Vec2 b, const Box& box) {
  a=rotate(sub(a,box.pose.position),-box.pose.yaw_rad);
  b=rotate(sub(b,box.pose.position),-box.pose.yaw_rad);
  const Vec2 d=sub(b,a);
  double enter=0.0, leave=1.0;
  for (const auto axis : {std::array<double,3>{{a.x,d.x,box.half_length}},
                          std::array<double,3>{{a.y,d.y,box.half_width}}}) {
    const double extent=axis[2]+1e-9;
    if (std::abs(axis[1])<1e-12) {
      if (std::abs(axis[0])>extent) return false;
    } else {
      double lo=(-extent-axis[0])/axis[1], hi=(extent-axis[0])/axis[1];
      if (lo>hi) std::swap(lo,hi);
      enter=std::max(enter,lo); leave=std::min(leave,hi);
      if (enter>leave) return false;
    }
  }
  return true;
}

// For a simple polygon, a connected box is wholly inside iff one interior
// point is inside and no polygon boundary intersects the box. Not corner-only.
inline bool polygonContainsBox(const std::vector<Vec2>& polygon,const Box& box) {
  if (polygon.size()<3) return false;
  bool inside=false;
  const Vec2 p=box.pose.position;
  for (std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    const Vec2 a=polygon[j], b=polygon[i];
    if (segmentIntersectsBox(a,b,box)) return false;
    if ((a.y>p.y)!=(b.y>p.y) &&
        p.x-a.x<(p.y-a.y)*(b.x-a.x)/(b.y-a.y)) inside=!inside;
  }
  return inside;
}

// Encloses all vehicle footprints when rear-axle position interpolates linearly
// and yaw follows the shortest rotation between the endpoint poses.
inline Box sweptEgoEnvelope(Pose2 a,Pose2 b,const VehicleShape& shape,double margin) {
  const Box first=egoBox(a,shape),last=egoBox(b,shape);
  const double angle=std::abs(wrap(b.yaw_rad-a.yaw_rad));
  const double yaw=yawLerp(a.yaw_rad,b.yaw_rad,0.5);
  const Vec2 delta=sub(last.pose.position,first.pose.position);
  const double offset=std::abs(shape.front_m-shape.rear_m)/2.0;
  const double padding=std::hypot(first.half_length,first.half_width)*angle/2.0+
                       offset*angle*angle/8.0+margin;
  return {{lerp(first.pose.position,last.pose.position,0.5),yaw},
          first.half_length+std::abs(dot(delta,unit(yaw)))/2.0+padding,
          first.half_width+std::abs(dot(delta,normal(yaw)))/2.0+padding};
}

// Continuous SAT for linearly translating rectangles. Mid-angle boxes are
// inflated by a circumscribed-radius rotation bound, so rotating boxes are
// conservatively covered throughout the interval. Contact counts as collision.
// Centers interpolate LINEARLY; curvature model interpolation error belongs in
// the object's uncertainty radius. This is not checking only the endpoints.
inline bool sweptOverlap(const Box& a0, const Box& a1, const Box& b0, const Box& b1) {
  const auto envelope = [](const Box& b, const Box& e) {
    const double hl = std::max(b.half_length, e.half_length);
    const double hw = std::max(b.half_width, e.half_width);
    const double delta = std::abs(wrap(e.pose.yaw_rad - b.pose.yaw_rad));
    const double pad = std::hypot(hl, hw) * delta / 2.0;
    return Box{{b.pose.position, yawLerp(b.pose.yaw_rad,e.pose.yaw_rad,0.5)}, hl+pad, hw+pad};
  };
  const Box a = envelope(a0,a1), b = envelope(b0,b1);
  const Vec2 p = sub(b0.pose.position,a0.pose.position);
  const Vec2 d = sub(sub(b1.pose.position,b0.pose.position),sub(a1.pose.position,a0.pose.position));
  double entry = 0.0, leave = 1.0;
  for (Vec2 axis : {unit(a.pose.yaw_rad),normal(a.pose.yaw_rad),unit(b.pose.yaw_rad),normal(b.pose.yaw_rad)}) {
    const double radius = extent(a,axis) + extent(b,axis) + 1e-9;
    const double start = dot(p,axis), velocity = dot(d,axis);
    if (std::abs(velocity) < 1e-12) {
      if (std::abs(start) > radius) return false;
    } else {
      double t0 = (-radius-start)/velocity, t1 = (radius-start)/velocity;
      if (t0 > t1) std::swap(t0,t1);
      entry = std::max(entry,t0); leave = std::min(leave,t1);
      if (entry > leave) return false;
    }
  }
  return true;
}

inline double curvature(Vec2 a, Vec2 b, Vec2 c) {
  const auto ab = sub(b,a), bc = sub(c,b), ac = sub(c,a);
  const double den = norm(ab)*norm(bc)*norm(ac);
  return den > 1e-12 ? 2.0*cross(ab,bc)/den : 0.0;
}
}  // namespace geometry
}  // namespace stier

namespace stier {

// Forward, road-reference-based DP + constrained finite-difference QP.
// Geometry DP/QP plus bounded arc-time speed search. No ROS, mission decisions,
// actuator commands, receipt health or stale-plan reuse.
// Algorithm design references: Apollo EM planner (arXiv:1807.08048), and the
// arc-time v/a/jerk + centripetal constraints in arXiv:2112.02133. Quintic
// lateral boundary conditions follow Werling et al., ICRA 2010, with d(s)
// rather than d(t) so that a stopped car does not change lane in place.
// box-constrained ADMM equations in https://osqp.org/docs/solver/ . The small
// Eigen solver here is NOT OSQP; it uses ADMM plus a bounded active-set KKT
// refinement, and has no infeasibility certificate.
class PathGenerationCore {
 public:
  explicit PathGenerationCore(PathParameters parameters = {}) : p_(parameters) { validateParameters(); }

  // Planned v/a are ground speed and tangential acceleration along the actual
  // rear-axle path. Reference progress is only for correspondence with the map.
  // Every result spans the available prediction interval; blocked motion is
  // represented by a physically reachable stop/wait, never a truncated prefix.
  std::vector<PathPoint> process(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                                const Predictions& predictions, const PlanningConstraints& limits,
                                const std::vector<PathPoint>& previous = {}) const {
    requireSceneDomain(ego,reference_map,predictions);
    requirePlanningDomain(ego,limits);
    // This independent calculation-window budget applies only to integrated
    // search. Do not invalidate finer legacy supplied-schedule configurations.
    if(p_.trajectory_horizon_m/p_.output_step_m>500 || p_.trajectory_horizon_m/p_.dp_step_m>100)
      throw std::invalid_argument("unbounded integrated planner discretization");
    const double offset=ego.stamp_sec-predictions.reference_stamp_sec;
    const double duration=predictions.step_sec*kPredictionSteps-offset;
    if (offset<0 || duration<=0 || ego.speed_mps>p_.max_speed_mps ||
        ego.acceleration_mps2>p_.max_acceleration_mps2 ||
        ego.acceleration_mps2<-p_.max_deceleration_mps2) return {};
    const Reference reference(reference_map);
    const auto origin=reference.project(ego.pose.position,0,reference.length());
    // Admission belongs to the scene, not to any single geometric proposal.
    // In particular, a hold-offset candidate must not bypass these guards.
    if(origin.distance>p_.max_initial_distance_m ||
       std::abs(geometry::wrap(ego.pose.yaw_rad-reference.at(origin.s).yaw))>p_.max_initial_heading_error_rad ||
       reference.ambiguous(ego.pose.position,origin,ego.pose.yaw_rad)) return {};
    const auto boundary=reference.boundary(p_.road_boundary_step_m);
    const std::vector<ProgressTime> unused;
    const Context c{reference,predictions,unused,boundary,0,offset};
    std::vector<std::vector<PathPoint>> proposals;
    proposals.push_back(generatePath(ego,reference_map,predictions,
                       {{0,0},{p_.trajectory_horizon_m,duration}},previous,true,p_.trajectory_horizon_m));
    if(std::abs(origin.l)>1e-6) {
      std::vector<PathPoint> hold;
      const double extent=std::min(p_.trajectory_horizon_m,reference.length()-origin.s-
                         std::hypot(p_.vehicle.front_m,p_.vehicle.width_m/2)-p_.road_margin_m-1e-6);
      if(extent>=std::max(p_.min_horizon_m,2*p_.output_step_m)) {
        const int count=static_cast<int>(std::ceil(extent/p_.output_step_m));
        for(int i=0;i<=count;++i) {
          const double s=extent*i/count;
          hold.push_back(lateralPoint(reference,origin.s+s,s,origin.l,0,0));
        }
        proposals.push_back(std::move(hold));
      }
    }
    // Timing guesses only choose geometric obstacle sides for DP/QP. They are
    // never published, nor treated as measured ego speed. Final timing is solved
    // anew for each candidate and checked against the original predictions.
    const double cap=std::min(p_.max_speed_mps,limits.maximum_speed_mps);
    if(!predictions.current.empty()) {
      std::vector<double> guesses{cap,std::max(ego.speed_mps,cap/2)};
      guesses.erase(std::unique(guesses.begin(),guesses.end()),guesses.end());
      for(double speed : guesses) {
        if(speed<=0) continue;
        proposals.push_back(generatePath(ego,reference_map,predictions,
            {{0,0},{p_.trajectory_horizon_m,p_.trajectory_horizon_m/speed}},previous,false,p_.trajectory_horizon_m));
      }
    }
    double best=std::numeric_limits<double>::infinity();
    std::vector<PathPoint> result;
    for(std::size_t i=0;i<proposals.size();++i) {
      auto& shape=proposals[i];
      if(shape.empty()) continue;
      // Guessed timing is discarded below. Identical geometry has identical
      // arc-time search inputs, even if two obstacle-side guesses produced it.
      bool duplicate=false;
      for(std::size_t j=0;j<i && !duplicate;++j) {
        const auto& prior=proposals[j];
        duplicate=shape.size()==prior.size() && std::equal(shape.begin(),shape.end(),prior.begin(),
          [](const PathPoint& a,const PathPoint& b) {
            return a.pose.position.x==b.pose.position.x && a.pose.position.y==b.pose.position.y &&
                   a.pose.yaw_rad==b.pose.yaw_rad && a.progress_m==b.progress_m &&
                   a.curvature_inv_m==b.curvature_inv_m;
          });
      }
      if(duplicate) continue;
      const auto matched=matchInitialCurve(c,ego,shape);
      if(matched.empty()) continue;
      const ArcPath arc(matched);
      const double stop_arc=stopArc(ego,reference,arc,limits);
      if(stop_arc<0 && limits.release_time_sec>0) continue;
      auto path=planSpeed(c,arc,ego,limits,duration,stop_arc);
      if(path.empty() || !checkPlannedPath(ego,reference_map,predictions,limits,path)) continue;
      const double cost=trajectoryCost(c,path,cap);
      if(cost<best) { best=cost; result=std::move(path); }
    }
    return result;
  }

  // Rechecks the cubic-speed / arc-interpolated representation above. This is
  // NOT checkTimedPath's legacy linear-in-time interpolation contract.
  bool checkPlannedPath(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                        const Predictions& predictions, const PlanningConstraints& limits,
                        const std::vector<PathPoint>& path) const {
    requireSceneDomain(ego,reference_map,predictions);
    requirePlanningDomain(ego,limits);
    if (path.size()<2) return false;
    for (std::size_t i=0;i<path.size();++i) {
      const auto& q=path[i];
      if (!geometry::finite(q.pose) || !std::isfinite(q.curvature_inv_m) ||
          !std::isfinite(q.speed_mps) || !std::isfinite(q.acceleration_mps2) ||
          !std::isfinite(q.progress_m) || !std::isfinite(q.time_sec) ||
          (i==0 && (q.progress_m!=0 || q.time_sec!=0)) ||
          (i>0 && (q.time_sec<=path[i-1].time_sec || q.progress_m<path[i-1].progress_m)))
        throw std::invalid_argument("invalid planned trajectory");
    }
    const double duration=predictions.step_sec*kPredictionSteps-(ego.stamp_sec-predictions.reference_stamp_sec);
    if (ego.stamp_sec<predictions.reference_stamp_sec || path.back().time_sec>duration+1e-12 ||
        geometry::norm(geometry::sub(path.front().pose.position,ego.pose.position))>1e-6 ||
        std::abs(geometry::wrap(path.front().pose.yaw_rad-ego.pose.yaw_rad))>1e-6 ||
        std::abs(path.front().speed_mps-ego.speed_mps)>1e-8 ||
        std::abs(path.front().acceleration_mps2-ego.acceleration_mps2)>1e-8 ||
        std::abs(path.front().curvature_inv_m-ego.curvature_inv_m)>1e-8) return false;
    const Reference reference(reference_map);
    const auto boundary=reference.boundary(p_.road_boundary_step_m);
    const std::vector<ProgressTime> unused;
    const Context c{reference,predictions,unused,boundary,0,ego.stamp_sec-predictions.reference_stamp_sec};
    const auto recovery=ceilingRecovery(ego,limits);
    if (!roadClear(c,path.front().pose,path.front().pose) || collision(c,path.front(),path.front())) return false;
    const auto line=stopLine(ego,reference,limits);
    for (std::size_t i=1;i<path.size();++i) {
      const auto& a=path[i-1]; const auto& b=path[i];
      const double distance=geometry::norm(geometry::sub(b.pose.position,a.pose.position));
      const double angle=std::abs(geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad));
      if (!std::isfinite(distance) || (distance==0 && (angle>1e-10 || a.progress_m!=b.progress_m)) ||
          (distance>0 && (b.progress_m==a.progress_m ||
             geometry::dot(geometry::sub(b.pose.position,a.pose.position),geometry::unit(a.pose.yaw_rad))<=0 ||
             geometry::dot(geometry::sub(b.pose.position,a.pose.position),geometry::unit(b.pose.yaw_rad))<=0))) return false;
      const SpeedEdge edge{0,a.speed_mps,a.acceleration_mps2,b.speed_mps,b.acceleration_mps2,
                           a.time_sec,b.time_sec-a.time_sec};
      const double curvature=std::max({std::abs(a.curvature_inv_m),std::abs(b.curvature_inv_m),
                                      distance>0 ? angle/distance : 0});
      if (!speedAdmissible(edge,ego,limits,curvature,recovery) ||
          std::abs(edge.distance(edge.h)-distance)>2e-6 ||
          !roadClear(c,a.pose,b.pose)) return false;
      const std::vector<PathPoint> pair{a,b};
      const ArcPath arc(pair);
      if (!edgeClear(c,arc,edge)) return false;
      if (std::isfinite(limits.stop_line_progress_m) && a.time_sec<limits.release_time_sec) {
        const double h=std::min(edge.h,limits.release_time_sec-a.time_sec);
        const auto until=arc.at(edge.distance(h));
        if (!behindLine(line,a.pose,until.pose)) return false;
      }
    }
    return sampledGeometryClear(path,true);
  }
  // Retain source compatibility for legacy braced schedules, including {}.
  std::vector<PathPoint> process(const EgoState& ego, const std::vector<ReferencePoint>& reference,
                                const Predictions& predictions, std::initializer_list<ProgressTime> schedule,
                                const std::vector<PathPoint>& previous = {}) const {
    return process(ego,reference,predictions,std::vector<ProgressTime>(schedule),previous);
  }

  // Empty means this finite search produced no usable point sequence. It is
  // neither a brake command nor proof that every possible maneuver is blocked.
  std::vector<PathPoint> process(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                     const Predictions& predictions_map, const std::vector<ProgressTime>& schedule,
                     const std::vector<PathPoint>& previous_path_map = {}) const {
    return generatePath(ego,reference_map,predictions_map,schedule,previous_path_map,false);
  }

 private:
  // Samples a valid prediction within its horizon, with no extrapolation.
  // Current state is retained so [0, dt] is covered too.
  static PredictedObject samplePrediction(const Predictions& predictions, std::size_t object_index, double time_sec) {
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


  // Geometric proposals may ignore guessed timing, but never road/shape limits.
  // Only final speed planning against the real predictions can approve motion.
  std::vector<PathPoint> generatePath(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                     const Predictions& predictions_map, const std::vector<ProgressTime>& schedule,
                     const std::vector<PathPoint>& previous_path_map, bool geometric_only,
                     double requested_horizon=0) const {
    requireSceneDomain(ego,reference_map,predictions_map);
    requireForwardSchedule(schedule);
    for (const auto& point : previous_path_map)
      if (!geometry::finite(point.pose)) throw std::invalid_argument("nonfinite previous path");
    const Reference reference(reference_map);
    const auto origin=reference.project(ego.pose.position,0.0,reference.length());
    if (origin.distance > p_.max_initial_distance_m)
      return {};
    const auto initial=reference.at(origin.s);
    const double heading_error=geometry::wrap(ego.pose.yaw_rad-initial.yaw);
    if (std::abs(heading_error)>p_.max_initial_heading_error_rad)
      return {};
    // Avoid silently picking an unrelated branch on overlapping references.
    if (reference.ambiguous(ego.pose.position,origin,ego.pose.yaw_rad))
      return {};
    const double prediction_offset=ego.stamp_sec-predictions_map.reference_stamp_sec;
    const double available_time=predictions_map.step_sec*kPredictionSteps-prediction_offset;
    if (prediction_offset<0.0 || available_time<=0.0) return {};
    const double maximum_horizon=requested_horizon>0 ? requested_horizon : p_.horizon_m;
    // On a curve the front outer corner can project farther than the center of
    // the bumper. Reserve its rear-axle radius, then still validate the polygon.
    const double front_reserve=requested_horizon>0 ? std::hypot(p_.vehicle.front_m,p_.vehicle.width_m/2) : p_.vehicle.front_m;
    const double horizon=std::min({maximum_horizon,reference.length()-origin.s-front_reserve-p_.road_margin_m-1e-6,
                                  geometric_only ? maximum_horizon : progressAt(schedule,available_time)});
    if (horizon+1e-9<std::max(p_.min_horizon_m,2.0*p_.output_step_m)) return {};
    const auto boundary=reference.boundary(p_.road_boundary_step_m);
    const Context c{reference,predictions_map,schedule,boundary,origin.s,prediction_offset,geometric_only};
    const double initial_metric=initial.metric*(1.0-initial.curvature*origin.l);
    if (initial_metric<=0.05)
      return {};
    const double initial_dl=initial_metric*std::tan(heading_error);

    struct Node { double l=0.0; double dl=0.0; double cost=std::numeric_limits<double>::infinity(); int parent=-1; };
    const int layers=std::max(1,static_cast<int>(std::ceil(horizon/p_.dp_step_m)));
    const double dp_ds=horizon/layers;
    std::vector<std::vector<Node>> lattice(static_cast<std::size_t>(layers)+1);
    lattice[0].push_back(Node{origin.l,initial_dl,0.0,-1});
    for (int i=1;i<=layers;++i) {
      const auto road=reference.at(origin.s+i*dp_ds);
      const double lower=road.lower+p_.vehicle.width_m/2.0+p_.road_margin_m;
      const double upper=road.upper-p_.vehicle.width_m/2.0-p_.road_margin_m;
      if (lower>upper) return {};
      std::vector<double> offsets;
      for (double l=std::ceil(lower/p_.lateral_step_m)*p_.lateral_step_m;l<=upper+1e-9;l+=p_.lateral_step_m) {
        offsets.push_back(l);
        if (offsets.size()>p_.max_lateral_samples)
          return {};
      }
      offsets.push_back(geometry::clamp(0.0,lower,upper));
      offsets.push_back(geometry::clamp(origin.l,lower,upper));
      std::sort(offsets.begin(),offsets.end());
      offsets.erase(std::unique(offsets.begin(),offsets.end(),[](double a,double b){return std::abs(a-b)<1e-8;}),offsets.end());
      const double prior=previousL(reference,origin.s+i*dp_ds,previous_path_map);
      for (double l : offsets) {
       for (double dl : {-p_.dp_slope_sample,0.0,p_.dp_slope_sample}) {
        if (i==layers && dl!=0.0) continue;
        Node best; best.l=l; best.dl=dl;
        const auto& before=lattice[static_cast<std::size_t>(i-1)];
        for (std::size_t j=0;j<before.size();++j) {
          if (!std::isfinite(before[j].cost)) continue;
          const double delta=l-before[j].l;
          if (std::abs(delta)/dp_ds>p_.max_lateral_slope) continue;
          const auto edge=makeEdge(c,(i-1)*dp_ds,before[j].l,i*dp_ds,l,before[j].dl,dl);
          if (!checkPath(c,edge)) continue;
          const double cost=before[j].cost+p_.reference_weight*l*l*dp_ds+
              p_.slope_weight*(delta*delta/dp_ds+dl*dl)+p_.previous_weight*(l-prior)*(l-prior)*dp_ds;
          if (cost<best.cost) { best.cost=cost; best.parent=static_cast<int>(j); }
        }
        lattice[static_cast<std::size_t>(i)].push_back(best);
       }
      }
      const auto& layer=lattice[static_cast<std::size_t>(i)];
      if (std::none_of(layer.begin(),layer.end(),[](const Node& n){return std::isfinite(n.cost);}))
        return {};
    }
    auto& last=lattice.back();
    int selected=static_cast<int>(std::min_element(last.begin(),last.end(),[](const Node& a,const Node& b){return a.cost<b.cost;})-last.begin());
    std::vector<double> coarse(static_cast<std::size_t>(layers)+1);
    std::vector<double> coarse_dl(static_cast<std::size_t>(layers)+1);
    for (int i=layers;i>0;--i) {
      const auto& n=lattice[static_cast<std::size_t>(i)][static_cast<std::size_t>(selected)];
      coarse[static_cast<std::size_t>(i)]=n.l; coarse_dl[static_cast<std::size_t>(i)]=n.dl; selected=n.parent;
    }
    coarse[0]=origin.l;
    coarse_dl[0]=initial_dl;

    const int n=std::max(3,static_cast<int>(std::ceil(horizon/p_.output_step_m))+1);
    const double ds=horizon/(n-1);
    Eigen::VectorXd guide(n), lower(n), upper(n), prior(n);
    std::vector<PathPoint> guide_path;
    for (int i=0;i<n;++i) {
      const double s=i*ds;
      const int layer=std::min(static_cast<int>(s/dp_ds),layers-1);
      const double l=quintic(coarse[static_cast<std::size_t>(layer)],coarse[static_cast<std::size_t>(layer+1)],
                             coarse_dl[static_cast<std::size_t>(layer)],coarse_dl[static_cast<std::size_t>(layer+1)],
                             dp_ds,(s-layer*dp_ds)/dp_ds);
      guide[i]=l; prior[i]=previousL(reference,origin.s+s,previous_path_map);
      const auto r=reference.at(origin.s+s);
      guide_path.push_back({{geometry::add(r.position,geometry::mul(geometry::normal(r.yaw),l)),r.yaw},s,timeAt(schedule,s),0.0});
      lower[i]=std::max(l-p_.corridor_half_width_m,r.lower+p_.vehicle.width_m/2.0+p_.road_margin_m);
      upper[i]=std::min(l+p_.corridor_half_width_m,r.upper-p_.vehicle.width_m/2.0-p_.road_margin_m);
      if (lower[i]>upper[i]) return {};
    }
    setGeometry(guide_path);
    guide_path.front().pose=ego.pose;
    // Keep the discrete obstacle side chosen by DP. A QP cost alone cannot
    // encode left/right alternatives or safely cut across a forbidden region.
    for (int i=0;i<n;++i) {
      const auto road=reference.at(origin.s+i*ds);
      const Vec2 normal=geometry::normal(road.yaw);
      const auto own=geometry::egoBox(guide_path[static_cast<std::size_t>(i)].pose,p_.vehicle);
      for (std::size_t j=0;!geometric_only && j<predictions_map.current.size();++j) {
        const auto other=geometry::objectBox(samplePrediction(predictions_map,j,
            prediction_offset+guide_path[static_cast<std::size_t>(i)].time_sec),p_.obstacle_margin_m);
        const auto d=geometry::sub(other.pose.position,own.pose.position);
        double best_width=-1.0, best_lo=lower[i], best_hi=upper[i];
        for (Vec2 axis : {geometry::unit(own.pose.yaw_rad),geometry::normal(own.pose.yaw_rad),
                          geometry::unit(other.pose.yaw_rad),geometry::normal(other.pose.yaw_rad)}) {
          const double delta=geometry::dot(d,axis);
          const double slack=std::abs(delta)-geometry::extent(own,axis)-geometry::extent(other,axis)-1e-6;
          if (slack<=0.0) continue;
          const double coefficient=(delta>=0.0 ? 1.0 : -1.0)*geometry::dot(normal,axis);
          double lo_axis=lower[i],hi_axis=upper[i];
          if (coefficient>1e-12) hi_axis=std::min(hi_axis,guide[i]+slack/coefficient);
          else if (coefficient<-1e-12) lo_axis=std::max(lo_axis,guide[i]+slack/coefficient);
          if (hi_axis-lo_axis>best_width) { best_width=hi_axis-lo_axis; best_lo=lo_axis; best_hi=hi_axis; }
        }
        if (best_width<0.0) return {};
        lower[i]=best_lo; upper[i]=best_hi;
      }
      if (lower[i]>upper[i]) return {};
    }
    lower[0]=upper[0]=origin.l;
    Eigen::VectorXd solution;
    if (!solveQp(guide,prior,lower,upper,ds,initial_dl,solution)) return {};
    std::vector<PathPoint> result;
    result.reserve(static_cast<std::size_t>(n));
    for (int i=0;i<n;++i) {
      const auto r=reference.at(origin.s+i*ds);
      result.push_back({{geometry::add(r.position,geometry::mul(geometry::normal(r.yaw),solution[i])),r.yaw},i*ds,timeAt(schedule,i*ds),0.0});
    }
    setGeometry(result);
    // Initial pose/yaw are execution boundary conditions, not a reference reset.
    if (geometry::norm(geometry::sub(result.front().pose.position,ego.pose.position))>1e-3) return {};
    if (std::abs(geometry::wrap(result.front().pose.yaw_rad-ego.pose.yaw_rad))>0.05) return {};
    result.front().pose=ego.pose;
    result=withScheduleKnots(result,schedule);
    if (!checkPath(c,result)) return {};
    return result;
  }

 public:
  // Pure numerical recheck after retiming/resampling a generated path. Times
  // are relative to ego.stamp_sec, not predictions.reference_stamp_sec.
  // No generation, time adjustment, truncation, fallback or execution policy.
  // Geometry/yaw/curvature must still describe the supplied sampled path.
  // All speed-profile breakpoints must be included; identical-pose waits are
  // allowed. A true result covers ONLY the modeled constraints below, not
  // longitudinal acceleration/jerk, initial speed or actuator dynamics.
  bool checkTimedPath(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                      const Predictions& predictions_map, const std::vector<PathPoint>& path_map) const {
    requireSceneDomain(ego,reference_map,predictions_map);
    for (std::size_t i=0;i<path_map.size();++i) {
      const auto& point=path_map[i];
      if (!geometry::finite(point.pose) || !std::isfinite(point.curvature_inv_m) ||
          !std::isfinite(point.progress_m) || !std::isfinite(point.time_sec) ||
          (i==0 && (point.progress_m!=0.0 || point.time_sec!=0.0)) ||
          (i>0 && (point.progress_m<path_map[i-1].progress_m || point.time_sec<=path_map[i-1].time_sec)))
        throw std::invalid_argument("invalid timed forward path");
      if (i>0 && ((point.progress_m==path_map[i-1].progress_m)!=
                 (geometry::norm(geometry::sub(point.pose.position,path_map[i-1].pose.position))==0.0)))
        throw std::invalid_argument("stationary position and stationary progress must agree");
    }
    if (path_map.size()<2) return false;
    if (geometry::norm(geometry::sub(path_map.front().pose.position,ego.pose.position))>1e-6 ||
        std::abs(geometry::wrap(path_map.front().pose.yaw_rad-ego.pose.yaw_rad))>1e-6) return false;
    const double offset=ego.stamp_sec-predictions_map.reference_stamp_sec;
    // Unlike generation, validation must not silently shorten the requested path.
    // Account only for roundoff when inverting t(s) at the prediction endpoint,
    // not an operational grace period or permission to extrapolate predictions.
    const double horizon=predictions_map.step_sec*kPredictionSteps;
    const double roundoff=16.0*std::numeric_limits<double>::epsilon()*std::max(1.0,horizon);
    if (offset<0.0 || offset+path_map.back().time_sec>horizon+roundoff) return false;
    const Reference reference(reference_map);
    const auto boundary=reference.boundary(p_.road_boundary_step_m);
    const std::vector<ProgressTime> unused_schedule;
    const Context c{reference,predictions_map,unused_schedule,boundary,0.0,offset};
    return checkPath(c,path_map) && sampledGeometryClear(path_map,false);
  }

 private:
  PathParameters p_;
  struct RoadSample {
    Vec2 position; double yaw; double lower; double upper;
    double metric=1.0;  // |dr/ds|; chord station is not exact Hermite arc length.
    double curvature=0.0;
  };
  struct Projection { double s=0.0; double l=0.0; double distance=std::numeric_limits<double>::infinity(); };

  class Reference {
   public:
    explicit Reference(const std::vector<ReferencePoint>& input) {
      for (const auto& p : input) {
        if (!points.empty() && geometry::norm(geometry::sub(p.position,points.back().position))<1e-6) {
          points.back().min_l_m=std::max(points.back().min_l_m,p.min_l_m);
          points.back().max_l_m=std::min(points.back().max_l_m,p.max_l_m);
        } else points.push_back(p);
      }
      if (points.size()<2) throw std::invalid_argument("reference has fewer than two distinct points");
      station.push_back(0.0);
      for (std::size_t i=1;i<points.size();++i) {
        const double next=station.back()+geometry::norm(geometry::sub(points[i].position,points[i-1].position));
        if (!std::isfinite(next)) throw std::invalid_argument("reference station arithmetic overflow");
        station.push_back(next);
      }
    }
    double length() const { return station.back(); }
    // The allowed road is the resulting piecewise-linear closed polygon.
    // Retain EVERY knot: a narrow notch must not disappear between grid samples.
    std::vector<Vec2> boundary(double step) const {
      std::vector<Vec2> left,right;
      const auto append=[&](double s) {
        const auto r=at(s);
        const Vec2 normal=geometry::normal(r.yaw);
        left.push_back(geometry::add(r.position,geometry::mul(normal,r.upper)));
        right.push_back(geometry::add(r.position,geometry::mul(normal,r.lower)));
      };
      for (std::size_t i=0;i+1<station.size();++i) {
        const double length=station[i+1]-station[i];
        const double count=std::ceil(length/step);
        if (!std::isfinite(count) || count>100000 || left.size()+count>100000)
          throw std::invalid_argument("road polygon discretization too large");
        const int n=std::max(1,static_cast<int>(count));
        for (int k=0;k<n;++k) append(station[i]+length*k/n);
      }
      append(station.back());
      left.insert(left.end(),right.rbegin(),right.rend());
      // Remove only exactly collinear subdivisions, never tolerance-based
      // smoothing. Narrow notches and every actual width/turn change survive.
      std::vector<Vec2> polygon;
      for(const auto point:left) {
        while(polygon.size()>1) {
          const auto a=geometry::sub(polygon.back(),polygon[polygon.size()-2]);
          const auto b=geometry::sub(point,polygon.back());
          if(geometry::cross(a,b)!=0 || geometry::dot(a,b)<=0) break;
          polygon.pop_back();
        }
        polygon.push_back(point);
      }
      return polygon;
    }
    RoadSample at(double s) const {
      s=geometry::clamp(s,0.0,length());
      const std::size_t i=segment(s);
      const double u=(s-station[i])/(station[i+1]-station[i]);
      const double h=station[i+1]-station[i];
      const Vec2 direction=geometry::mul(geometry::sub(points[i+1].position,points[i].position),1.0/h);
      if (geometry::norm(geometry::sub(tangent(i),direction))<1e-12 &&
          geometry::norm(geometry::sub(tangent(i+1),direction))<1e-12)
        return {geometry::lerp(points[i].position,points[i+1].position,u),std::atan2(direction.y,direction.x),
                geometry::lerp(points[i].min_l_m,points[i+1].min_l_m,u),geometry::lerp(points[i].max_l_m,points[i+1].max_l_m,u)};
      const Vec2 m0=geometry::mul(tangent(i),h),m1=geometry::mul(tangent(i+1),h);
      const Vec2 delta=geometry::sub(points[i+1].position,points[i].position);
      const double u2=u*u,u3=u2*u;
      // Local differences avoid cancellation at large map/UTM coordinates.
      const Vec2 pos=geometry::add(points[i].position,geometry::add(geometry::mul(delta,-2*u3+3*u2),
          geometry::add(geometry::mul(m0,u3-2*u2+u),geometry::mul(m1,u3-u2))));
      const Vec2 d=geometry::add(geometry::mul(delta,-6*u2+6*u),
          geometry::add(geometry::mul(m0,3*u2-4*u+1),geometry::mul(m1,3*u2-2*u)));
      const Vec2 dd=geometry::add(geometry::mul(delta,-12*u+6),
          geometry::add(geometry::mul(m0,6*u-4),geometry::mul(m1,6*u-2)));
      const double speed=geometry::norm(d);
      if (speed<1e-9) throw std::invalid_argument("reference tangent degeneracy/cusp");
      return {pos,std::atan2(d.y,d.x),
              geometry::lerp(points[i].min_l_m,points[i+1].min_l_m,u),geometry::lerp(points[i].max_l_m,points[i+1].max_l_m,u),
              speed/h,(d.x*dd.y-d.y*dd.x)/(speed*speed*speed)};
    }
    Projection project(Vec2 p,double lo,double hi) const {
      Projection best;
      const std::size_t begin=segment(geometry::clamp(lo,0.0,length()));
      const std::size_t end=segment(geometry::clamp(hi,0.0,length()));
      for (std::size_t i=begin;i<=end;++i) {
        const auto d=geometry::sub(points[i+1].position,points[i].position);
        const double len=station[i+1]-station[i];
        const double u=geometry::clamp(geometry::dot(geometry::sub(p,points[i].position),d)/(len*len),0.0,1.0);
        double s=station[i]+u*len;
        for (int k=0;k<5;++k) {
          const auto r=at(s);
          const double next=geometry::clamp(s+geometry::dot(geometry::sub(p,r.position),geometry::unit(r.yaw)),station[i],station[i+1]);
          if (std::abs(next-s)<1e-8) break;
          s=next;
        }
        const auto r=at(s);
        const auto delta=geometry::sub(p,r.position);
        const double distance=geometry::norm(delta);
        if (distance<best.distance) best={s,geometry::dot(geometry::normal(r.yaw),delta),distance};
      }
      return best;
    }
    bool ambiguous(Vec2 p,const Projection& best,double yaw) const {
      for (std::size_t i=0;i+1<points.size();++i) {
        if (std::abs(station[i]-best.s)<5.0) continue;
        const auto q=project(p,station[i],station[i]);
        const auto r=at(q.s);
        if (std::abs(q.s-best.s)>5.0 && q.distance<best.distance+0.05 && std::abs(geometry::wrap(r.yaw-yaw))<0.5) return true;
      }
      return false;
    }
   private:
    std::vector<ReferencePoint> points;
    std::vector<double> station;
    Vec2 tangent(std::size_t i) const {
      const auto segment_direction=[&](std::size_t j) {
        return geometry::mul(geometry::sub(points[j+1].position,points[j].position),1.0/(station[j+1]-station[j]));
      };
      if (i==0) return segment_direction(0);
      if (i+1==points.size()) return segment_direction(i-1);
      return geometry::mul(geometry::add(segment_direction(i-1),segment_direction(i)),0.5);
    }
    std::size_t segment(double s) const {
      auto it=std::upper_bound(station.begin(),station.end(),s);
      return std::min(static_cast<std::size_t>(std::max<std::ptrdiff_t>(0,it-station.begin()-1)),points.size()-2);
    }
  };

  struct Context {
    const Reference& reference;
    const Predictions& predictions;
    const std::vector<ProgressTime>& schedule;
    const std::vector<Vec2>& boundary;
    double start_s;
    double prediction_offset;
    bool geometric_only=false;
  };

  // A cubic velocity joins endpoint v/a exactly. Position is its quartic
  // integral. This bounded-control lattice uses the s,v,a state of Apollo's
  // arc-time formulation; it is a finite beam search, not Apollo's spline QP.
  struct SpeedEdge {
    double s0,v0,a0,v1,a1,t0,h;
    double c2() const { return 3*(v1-v0)/(h*h)-(2*a0+a1)/h; }
    double c3() const { return 2*(v0-v1)/(h*h*h)+(a0+a1)/(h*h); }
    double speed(double t) const { return v0+t*(a0+t*(c2()+t*c3())); }
    double acceleration(double t) const { return a0+t*(2*c2()+3*t*c3()); }
    double jerk(double t) const { return 2*c2()+6*t*c3(); }
    double distance(double t) const { return t*(v0+t*(a0/2+t*(c2()/3+t*c3()/4))); }
    std::vector<double> extrema() const {
      std::vector<double> times{0,h};
      const auto add=[&](double t){ if(t>0 && t<h) times.push_back(t); };
      const double a=3*c3(),b=2*c2(),c=a0;
      if (std::abs(a)<1e-12) { if(std::abs(b)>1e-12) add(-c/b); }
      else {
        const double d=b*b-4*a*c;
        if(d>=0) { add((-b-std::sqrt(d))/(2*a)); add((-b+std::sqrt(d))/(2*a)); }
      }
      if(std::abs(c3())>1e-12) add(-c2()/(3*c3()));
      return times;
    }
  };
  struct SpeedState { double s,v,a,cost; int parent; };

  void retainSpeedState(std::vector<SpeedState>& cell,const SpeedState& state) const {
    // Exact states have identical future dynamics; rounded cells do NOT.
    // Keep bounded representatives for cost, braking reserve and arrival.
    for(auto& q:cell) {
      if(q.s==state.s && q.v==state.v && q.a==state.a) {
        if(state.cost<q.cost) q=state;
        return;
      }
    }
    cell.push_back(state);
    const auto best=std::min_element(cell.begin(),cell.end(),[](const SpeedState& a,const SpeedState& b) {
      return a.cost<b.cost;
    });
    const auto braking=std::min_element(cell.begin(),cell.end(),[&](const SpeedState& a,const SpeedState& b) {
      return a.s+brakingDistance(a.v,a.a)<b.s+brakingDistance(b.v,b.a);
    });
    const auto arrival=std::max_element(cell.begin(),cell.end(),[](const SpeedState& a,const SpeedState& b) {
      return a.s<b.s;
    });
    std::vector<SpeedState> retained{*best};
    if(braking!=best) retained.push_back(*braking);
    if(arrival!=best && arrival!=braking) retained.push_back(*arrival);
    cell=std::move(retained);
  }
  void pruneSpeedLayer(std::vector<SpeedState>& next,double cap) const {
    std::stable_sort(next.begin(),next.end(),[](const SpeedState& a,const SpeedState& b){return a.cost<b.cost;});
    if(next.size()<=p_.speed_beam_width) return;
    if(p_.speed_beam_width==1) { next.resize(1); return; }
    const auto braking=std::min_element(next.begin(),next.end(),[&](const SpeedState& a,const SpeedState& b) {
      return a.s+brakingDistance(a.v,a.a)<b.s+brakingDistance(b.v,b.a);
    });
    std::vector<bool> selected(next.size(),false); selected[0]=true;
    const std::size_t brake_index=static_cast<std::size_t>(braking-next.begin());
    selected[brake_index]=true;
    std::size_t count=brake_index==0 ? 1 : 2;
    const std::size_t budget=std::max<std::size_t>(1,p_.speed_beam_width/4);
    const std::size_t used=brake_index==0 ? 0 : 1;
    // Same-time progress bands preserve different arrival times. Separate
    // exact waits and acceleration regimes; reserve <=1/4 of the beam, then
    // fill by cost. This is finite search diversity, NOT a completeness proof.
    const double width=std::max(.5,cap*p_.speed_time_step_sec);
    const auto key=[&](const SpeedState& q) {
      return std::make_tuple(std::llround(q.s/width),q.a<0 ? -1 : q.a>0 ? 1 : 0,q.v==0 && q.a==0);
    };
    std::map<decltype(key(next.front())),std::size_t> bands;
    for(std::size_t i=0;i<next.size();++i) bands.emplace(key(next[i]),i);
    bands.erase(key(next[0])); bands.erase(key(next[brake_index]));
    std::vector<std::size_t> representatives;
    for(const auto& band:bands) representatives.push_back(band.second);
    const std::size_t slots=std::min(budget-used,representatives.size());
    for(std::size_t k=0;k<slots;++k) {
      const std::size_t index=slots==1 ? representatives.size()/2 : k*(representatives.size()-1)/(slots-1);
      selected[representatives[index]]=true; ++count;
    }
    for(std::size_t i=0;count<p_.speed_beam_width;++i)
      if(!selected[i]) { selected[i]=true; ++count; }
    std::vector<SpeedState> retained; retained.reserve(p_.speed_beam_width);
    for(std::size_t i=0;i<next.size();++i) if(selected[i]) retained.push_back(next[i]);
    next=std::move(retained);
  }

  struct CeilingPhase {
    double t=0,h=0,v=0,a=0,j=0;
    double speed(double dt) const { return v+a*dt+j*dt*dt/2; }
  };
  struct CeilingRecovery {
    std::array<CeilingPhase,3> phase{};
    std::size_t count=0;
    double cap=0;
  };
  // Hard, jerk-limited recovery envelope, independent of objective weights.
  // A fastest S-curve from measured v/a to the lowered ceiling prevents each
  // receding-horizon call from deferring deceleration indefinitely. Braking
  // harder for an obstacle is allowed. Excess negative initial acceleration
  // may unavoidably undershoot the ceiling while it is ramped back to zero.
  CeilingRecovery ceilingRecovery(const EgoState& ego,const PlanningConstraints& limits) const {
    CeilingRecovery out;
    out.cap=std::min(p_.max_speed_mps,limits.maximum_speed_mps);
    const double j=p_.max_longitudinal_jerk_mps3,d=p_.max_deceleration_mps2;
    double v=ego.speed_mps,a=ego.acceleration_mps2,t=0;
    if(v+std::pow(std::max(0.0,a),2)/(2*j)<=out.cap+1e-8) return out;
    const auto append=[&](double h,double jerk) {
      if(h<=0) return;
      out.phase[out.count++]={t,h,v,a,jerk};
      v+=a*h+jerk*h*h/2; a+=jerk*h; t+=h;
    };
    if(a<0 && v-out.cap<a*a/(2*j)) { append(-a/j,j); return out; }
    const double b=std::min(d,std::sqrt(std::max(0.0,j*(v-out.cap)+a*a/2)));
    append((a+b)/j,-j);
    if(b>0) {
      append(std::max(0.0,(v-out.cap-b*b/(2*j))/b),0);
      append(b/j,j);
    }
    return out;
  }
  static double maximumCubic(double c0,double c1,double c2,double c3,double lo,double hi) {
    const auto value=[&](double t){return c0+t*(c1+t*(c2+t*c3));};
    double result=std::max(value(lo),value(hi));
    const auto at=[&](double t){if(t>lo && t<hi) result=std::max(result,value(t));};
    if(std::abs(c3)<1e-12) { if(std::abs(c2)>1e-12) at(-c1/(2*c2)); }
    else {
      const double discriminant=4*c2*c2-12*c3*c1;
      if(discriminant>=0) {
        at((-2*c2-std::sqrt(discriminant))/(6*c3));
        at((-2*c2+std::sqrt(discriminant))/(6*c3));
      }
    }
    return result;
  }
  bool belowRecovery(const SpeedEdge& edge,const CeilingRecovery& recovery) const {
    double end=0;
    for(std::size_t i=0;i<recovery.count;++i) {
      const auto& phase=recovery.phase[i];
      const double lo=std::max(0.0,phase.t-edge.t0),hi=std::min(edge.h,phase.t+phase.h-edge.t0);
      end=phase.t+phase.h;
      if(hi<lo) continue;
      const double delta=edge.t0-phase.t;
      if(maximumCubic(edge.v0-phase.speed(delta),edge.a0-phase.a-phase.j*delta,
                      edge.c2()-phase.j/2,edge.c3(),lo,hi)>1e-8) return false;
    }
    const double lo=std::max(0.0,end-edge.t0);
    return lo>edge.h || maximumCubic(edge.v0-recovery.cap,edge.a0,edge.c2(),edge.c3(),lo,edge.h)<=1e-8;
  }

  struct ArcPath {
    const std::vector<PathPoint>& points;
    std::vector<double> station;
    std::vector<double> position_curvature;
    std::vector<double> interpolation_knots;
    explicit ArcPath(const std::vector<PathPoint>& input):points(input),station(1,0) {
      for(std::size_t i=1;i<input.size();++i)
        station.push_back(station.back()+geometry::norm(geometry::sub(input[i].pose.position,input[i-1].pose.position)));
      position_curvature.resize(input.size(),0);
      for(std::size_t i=1;i+1<input.size();++i)
        position_curvature[i]=std::abs(geometry::curvature(input[i-1].pose.position,input[i].pose.position,
                                                         input[i+1].pose.position));
      if(input.size()>2) {
        position_curvature.front()=position_curvature[1];
        position_curvature.back()=position_curvature[input.size()-2];
      }
      // The immutable arc is sampled by many speed candidates. Cache the
      // existing breakpoint predicate once; its tolerances and meaning do not
      // change. Position/yaw/curvature/progress changes must all be retained.
      for(std::size_t i=0;i<input.size();++i) {
        if(i>0 && i+1<input.size()) {
          const auto& a=input[i-1]; const auto& b=input[i]; const auto& c=input[i+1];
          const auto d0=geometry::sub(b.pose.position,a.pose.position),d1=geometry::sub(c.pose.position,b.pose.position);
          const double n0=geometry::norm(d0),n1=geometry::norm(d1);
          if(n0>0 && n1>0 && geometry::dot(d0,d1)>0 && std::abs(geometry::cross(d0,d1))<=1e-12*n0*n1 &&
             std::abs(geometry::wrap(a.pose.yaw_rad-b.pose.yaw_rad))<=1e-12 &&
             std::abs(geometry::wrap(b.pose.yaw_rad-c.pose.yaw_rad))<=1e-12 &&
             std::abs(a.curvature_inv_m-b.curvature_inv_m)<=1e-12 &&
             std::abs(b.curvature_inv_m-c.curvature_inv_m)<=1e-12 &&
             std::abs((b.progress_m-a.progress_m)/n0-(c.progress_m-b.progress_m)/n1)<=1e-10) continue;
        }
        interpolation_knots.push_back(station[i]);
      }
    }
    double length() const { return station.back(); }
    PathPoint at(double s) const {
      if(s<=0 || length()==0) return points.front();
      if(s>=length()) return points.back();
      const auto it=std::upper_bound(station.begin(),station.end(),s);
      const std::size_t i=static_cast<std::size_t>(it-station.begin());
      const double u=(s-station[i-1])/(station[i]-station[i-1]);
      const auto& a=points[i-1]; const auto& b=points[i];
      return {{geometry::lerp(a.pose.position,b.pose.position,u),geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u)},
              geometry::lerp(a.progress_m,b.progress_m,u),0,
              geometry::lerp(a.curvature_inv_m,b.curvature_inv_m,u)};
    }
    double curvature(double lo,double hi) const {
      double k=std::max(std::abs(at(lo).curvature_inv_m),std::abs(at(hi).curvature_inv_m));
      auto it=std::upper_bound(station.begin(),station.end(),lo);
      for(;it!=station.end() && *(it-1)<=hi;++it) {
        const std::size_t i=static_cast<std::size_t>(it-station.begin());
        k=std::max({k,std::abs(points[i-1].curvature_inv_m),std::abs(points[i].curvature_inv_m),
                    position_curvature[i-1],position_curvature[i],
                    std::abs(geometry::wrap(points[i].pose.yaw_rad-points[i-1].pose.yaw_rad))/(*it-*(it-1))});
      }
      return k;
    }
  };

  void requirePlanningDomain(const EgoState& ego,const PlanningConstraints& limits) const {
    for(double x : {ego.speed_mps,ego.acceleration_mps2,ego.curvature_inv_m})
      if(!std::isfinite(x)) throw std::invalid_argument("nonfinite initial motion");
    if(ego.speed_mps<0) throw std::invalid_argument("reverse motion is outside forward planning domain");
    for(double x : {limits.maximum_speed_mps,limits.stop_line_progress_m,limits.release_time_sec})
      if(std::isnan(x) || x<0) throw std::invalid_argument("negative/nonfinite planning constraint");
  }

  bool sampledGeometryClear(const std::vector<PathPoint>& path,bool planned) const {
    std::vector<std::size_t> support;
    for(std::size_t i=1;i<path.size();++i) {
      const auto d=geometry::sub(path[i].pose.position,path[i-1].pose.position);
      const double length=geometry::norm(d);
      if(length>1e-9) {
        const double direction=std::atan2(d.y,d.x);
        // Arc interpolation subdivides the same geometric chord; its yaw is
        // the underlying curve tangent, not the piecewise-linear chord.
        // Allow only the angle error of one configured spatial sample.
        const double tolerance=p_.max_curvature_inv_m*p_.output_step_m+1e-5;
        if(std::abs(geometry::wrap(direction-path[i-1].pose.yaw_rad))>tolerance ||
           std::abs(geometry::wrap(direction-path[i].pose.yaw_rad))>tolerance) return false;
      }
    }
    const ArcPath arc(path);
    // Retain every actual turn. Uniform resampling alone can alias geometry
    // whose spacing is finer than output_step_m. Only collinear subdivisions
    // (including waits) are removed; their timing remains in the original path.
    for(std::size_t i=0;i<path.size();++i) {
      if(!support.empty() && arc.station[i]-arc.station[support.back()]<=1e-9) continue;
      while(support.size()>1) {
        const auto a=geometry::sub(path[support.back()].pose.position,
                                   path[support[support.size()-2]].pose.position);
        const auto b=geometry::sub(path[i].pose.position,path[support.back()].pose.position);
        if(geometry::dot(a,b)<=0 || std::abs(std::atan2(geometry::cross(a,b),geometry::dot(a,b)))>1e-8) break;
        support.pop_back();
      }
      support.push_back(i);
    }
    const auto admissible=[&](double k,double lo,double hi) {
      if(k>p_.max_curvature_inv_m+1e-6) return false;
      auto first=std::upper_bound(arc.station.begin(),arc.station.end(),lo);
      for(std::size_t j=static_cast<std::size_t>(first-arc.station.begin());
          j<path.size() && arc.station[j-1]<=hi;++j) {
        const auto& a=path[j-1]; const auto& b=path[j];
        if(planned) {
          const SpeedEdge edge{0,a.speed_mps,a.acceleration_mps2,b.speed_mps,b.acceleration_mps2,
                               a.time_sec,b.time_sec-a.time_sec};
          for(double t:edge.extrema())
            if(std::pow(edge.speed(t),2)*k>p_.max_lateral_accel_mps2+1e-6) return false;
        } else {
          const double speed=(arc.station[j]-arc.station[j-1])/(b.time_sec-a.time_sec);
          if(speed*speed*k>p_.max_lateral_accel_mps2+1e-6) return false;
        }
      }
      return true;
    };
    for(std::size_t i=1;i+1<support.size();++i) {
      const auto a=path[support[i-1]].pose.position,b=path[support[i]].pose.position,
                 c=path[support[i+1]].pose.position;
      double k=std::abs(geometry::curvature(a,b,c));
      const auto left=geometry::sub(b,a),right=geometry::sub(c,b);
      const double l=geometry::norm(left),r=geometry::norm(right);
      // A final time endpoint can truncate an interpolated spatial chord.
      // Positions alone cannot distinguish that case from an actual short
      // corner: only use the preceding support when both the supplied curve
      // curvature and its independent yaw change support that interpretation.
      // Interior turns, however finely spaced, always keep local curvature.
      if(i+2==support.size() && r<l && r<p_.output_step_m) {
        const double turn=std::atan2(geometry::cross(left,right),geometry::dot(left,right));
        const double sign=turn<0 ? -1 : 1;
        const double endpoint_k=std::abs(turn)/l;
        const double declared_k=std::max({sign*path[support[i-1]].curvature_inv_m,
                                        sign*path[support[i]].curvature_inv_m,
                                        sign*path[support[i+1]].curvature_inv_m});
        const double yaw_k=sign*geometry::wrap(path[support[i]].pose.yaw_rad-
                                              path[support[i-1]].pose.yaw_rad)/l;
        if(endpoint_k<=declared_k+1e-5 && endpoint_k<=yaw_k+1e-5) k=std::min(k,endpoint_k);
      }
      if(!admissible(k,arc.station[support[i-1]],arc.station[support[i+1]])) return false;
    }
    // Reconstruct geometry at equal spatial intervals, not at irregular time
    // samples. A tiny time sample or truncated final chord beside a polygon
    // vertex must not amplify the underlying spatial discretization curvature.
    const double step=p_.output_step_m;
    for(double s=step;s+step<=arc.length();s+=step) {
      const double k=std::abs(geometry::curvature(arc.at(s-step).pose.position,arc.at(s).pose.position,
                                                arc.at(s+step).pose.position));
      if(!admissible(k,s-step,s+step)) return false;
    }
    return true;
  }
  RoadSample stopLine(const EgoState& ego,const Reference& reference,const PlanningConstraints& limits) const {
    const auto origin=reference.project(ego.pose.position,0,reference.length());
    return reference.at(origin.s+limits.stop_line_progress_m);
  }
  bool behindLine(const RoadSample& line,Pose2 a,Pose2 b) const {
    const auto envelope=geometry::sweptEgoEnvelope(a,b,p_.vehicle,p_.road_margin_m);
    const auto normal=geometry::unit(line.yaw);
    return geometry::dot(geometry::sub(envelope.pose.position,line.position),normal)+
           geometry::extent(envelope,normal)<-1e-8;
  }
  double stopArc(const EgoState& ego,const Reference& reference,const ArcPath& arc,
                 const PlanningConstraints& limits) const {
    if(!std::isfinite(limits.stop_line_progress_m) || limits.release_time_sec==0)
      return std::numeric_limits<double>::infinity();
    const auto origin=reference.project(ego.pose.position,0,reference.length());
    // A line beyond the supplied reference is outside this geometric window.
    if(origin.s+limits.stop_line_progress_m>reference.length())
      return std::numeric_limits<double>::infinity();
    const auto line=stopLine(ego,reference,limits);
    if(!behindLine(line,arc.points.front().pose,arc.points.front().pose)) return -1;
    for(std::size_t i=1;i<arc.points.size();++i) {
      if(behindLine(line,arc.points[i-1].pose,arc.points[i].pose)) continue;
      double lo=arc.station[i-1],hi=arc.station[i];
      for(int k=0;k<40;++k) {
        const double mid=(lo+hi)/2;
        if(behindLine(line,arc.points[i-1].pose,arc.at(mid).pose)) lo=mid; else hi=mid;
      }
      return lo;
    }
    return std::numeric_limits<double>::infinity();
  }

  // Convert rear-axle yaw/curvature to lateral derivatives on the Hermite
  // reference (whose chord station is NOT exact arc). This includes the
  // reference metric/curvature derivatives and offset curve terms.
  std::array<double,2> lateralDerivatives(const Reference& ref,double s,double l,double yaw,double k) const {
    const auto r=ref.at(s),a=ref.at(std::max(0.0,s-.01)),b=ref.at(std::min(ref.length(),s+.01));
    const double span=std::min(ref.length(),s+.01)-std::max(0.0,s-.01);
    const double mp=(b.metric-a.metric)/span,kp=(b.curvature-a.curvature)/span;
    const double f=r.metric*(1-r.curvature*l);
    const double dl=f*std::tan(geometry::wrap(yaw-r.yaw));
    const double fp=mp*(1-r.curvature*l)-r.metric*kp*l-r.metric*r.curvature*dl;
    const double ddl=(k*std::pow(f*f+dl*dl,1.5)+dl*fp-r.metric*r.curvature*dl*dl)/f-f*r.metric*r.curvature;
    return {{dl,ddl}};
  }
  PathPoint lateralPoint(const Reference& ref,double station,double progress,double l,double dl,double ddl) const {
    const auto r=ref.at(station),a=ref.at(std::max(0.0,station-.01)),b=ref.at(std::min(ref.length(),station+.01));
    const double span=std::min(ref.length(),station+.01)-std::max(0.0,station-.01);
    const double f=r.metric*(1-r.curvature*l);
    const double fp=(b.metric-a.metric)/span*(1-r.curvature*l)-
                    r.metric*(b.curvature-a.curvature)/span*l-r.metric*r.curvature*dl;
    const double k=(f*(f*r.metric*r.curvature+ddl)-dl*(fp-r.metric*r.curvature*dl))/std::pow(f*f+dl*dl,1.5);
    return {{geometry::add(r.position,geometry::mul(geometry::normal(r.yaw),l)),
             geometry::wrap(r.yaw+std::atan2(dl,f))},progress,progress,k};
  }
  std::vector<PathPoint> matchInitialCurve(const Context& c,const EgoState& ego,
                                         const std::vector<PathPoint>& shape) const {
    const auto origin=c.reference.project(ego.pose.position,0,c.reference.length());
    const double h=std::min(10.0,shape.back().progress_m);
    auto end=std::lower_bound(shape.begin(),shape.end(),h,[](const PathPoint& q,double s){return q.progress_m<s;});
    if(end==shape.end()) end=shape.end()-1;
    const double extent=end->progress_m;
    const auto road=c.reference.at(origin.s+extent);
    const double l1=geometry::dot(geometry::sub(end->pose.position,road.position),geometry::normal(road.yaw));
    const auto start=lateralDerivatives(c.reference,origin.s,origin.l,ego.pose.yaw_rad,ego.curvature_inv_m);
    const auto finish=lateralDerivatives(c.reference,origin.s+extent,l1,end->pose.yaw_rad,end->curvature_inv_m);
    if(!std::isfinite(start[0]) || !std::isfinite(start[1]) || !std::isfinite(finish[0]) || !std::isfinite(finish[1])) return {};
    // Quintic Hermite, matching position, first and second derivatives at both
    // ends. Unlike overwriting a curvature field, this changes actual geometry.
    const double c0=origin.l,c1=start[0]*extent,c2=start[1]*extent*extent/2;
    const double d=l1-c0-c1-c2,v=finish[0]*extent-c1-2*c2,a=finish[1]*extent*extent-2*c2;
    const double c3=10*d-4*v+a/2,c4=-15*d+7*v-a,c5=6*d-3*v+a/2;
    std::vector<PathPoint> out;
    const int count=std::max(2,static_cast<int>(std::ceil(extent/p_.output_step_m)));
    for(int i=0;i<=count;++i) {
      const double u=static_cast<double>(i)/count;
      const double l=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))));
      const double dl=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/extent;
      const double ddl=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(extent*extent);
      out.push_back(lateralPoint(c.reference,origin.s+u*extent,u*extent,l,dl,ddl));
    }
    for(auto it=end+1;it!=shape.end();++it) { auto q=*it; q.time_sec=q.progress_m; out.push_back(q); }
    out.front().pose=ego.pose; out.front().curvature_inv_m=ego.curvature_inv_m;
    const Context geometric{c.reference,c.predictions,c.schedule,c.boundary,origin.s,c.prediction_offset,true};
    if(!checkPath(geometric,out)) return {};
    return out;
  }
  double edgeCost(const Context& c,const ArcPath& arc,const SpeedEdge& edge,double origin_s,double cap) const {
    // Four-point Gauss-Legendre integrates squared cubic v, quadratic a and
    // linear j exactly (degree <=6). The reference term is numerical quadrature
    // of the same continuous definition in search and final candidate ranking.
    const double nodes[]{.06943184420297371,.33000947820757187,.66999052179242813,.93056815579702629};
    const double weights[]{.17392742256872693,.32607257743127307,.32607257743127307,.17392742256872693};
    double cost=-.2*edge.distance(edge.h);
    for(int i=0;i<4;++i) {
      const double t=nodes[i]*edge.h;
      const auto q=arc.at(edge.s0+edge.distance(t));
      const auto r=c.reference.at(origin_s+q.progress_m);
      const double l=geometry::dot(geometry::sub(q.pose.position,r.position),geometry::normal(r.yaw));
      cost+=edge.h*weights[i]*(p_.speed_tracking_weight*std::pow(edge.speed(t)-cap,2)+
        p_.acceleration_weight*std::pow(edge.acceleration(t),2)+
        p_.longitudinal_jerk_weight*std::pow(edge.jerk(t),2)+p_.reference_weight*l*l);
    }
    return cost;
  }
  double trajectoryCost(const Context& c,const std::vector<PathPoint>& path,double cap) const {
    double cost=0; const ArcPath arc(path);
    const auto origin=c.reference.project(path.front().pose.position,0,c.reference.length());
    for(std::size_t i=1;i<path.size();++i) {
      const auto& a=path[i-1]; const auto& b=path[i];
      const SpeedEdge edge{arc.station[i-1],a.speed_mps,a.acceleration_mps2,b.speed_mps,b.acceleration_mps2,
                           a.time_sec,b.time_sec-a.time_sec};
      cost+=edgeCost(c,arc,edge,origin.s,cap);
    }
    return cost;
  }

  bool speedAdmissible(const SpeedEdge& edge,const EgoState& ego,const PlanningConstraints& limits,
                       double curvature,const CeilingRecovery& recovery) const {
    const double cap=std::min(p_.max_speed_mps,limits.maximum_speed_mps);
    // An upstream ceiling reduction cannot erase measured momentum. Permit
    // only a non-accelerating transition above it (or the unavoidable initial
    // positive-acceleration ramp-down at maximum configured jerk).
    const double initial_peak=ego.speed_mps+std::pow(std::max(0.0,ego.acceleration_mps2),2)/
                                                (2*p_.max_longitudinal_jerk_mps3);
    const bool initial_ramp=edge.t0<std::max(0.0,ego.acceleration_mps2)/p_.max_longitudinal_jerk_mps3 &&
                            initial_peak>cap;
    const double ceiling=edge.v0<=cap+1e-8 && !initial_ramp ? cap : std::max(cap,initial_peak);
    if(!std::isfinite(curvature) || curvature>p_.max_curvature_inv_m+1e-6) return false;
    for(double t:edge.extrema()) {
      const double v=edge.speed(t),a=edge.acceleration(t),j=edge.jerk(t);
      if(!std::isfinite(v) || !std::isfinite(a) || !std::isfinite(j) || v<-1e-8 ||
         v>ceiling+1e-8 || v>p_.max_speed_mps+1e-8 || a>p_.max_acceleration_mps2+1e-8 ||
         a<-p_.max_deceleration_mps2-1e-8 || std::abs(j)>p_.max_longitudinal_jerk_mps3+1e-8 ||
         v*v*curvature>p_.max_lateral_accel_mps2+1e-6) return false;
    }
    if(edge.v0>cap+1e-8 || initial_ramp) {
      auto times=edge.extrema();
      const double release=ego.acceleration_mps2/p_.max_longitudinal_jerk_mps3-edge.t0;
      if(release>0 && release<edge.h) times.push_back(release);
      if(std::abs(edge.c3())>1e-12) {
        const double t=(-p_.max_longitudinal_jerk_mps3-2*edge.c2())/(6*edge.c3());
        if(t>0 && t<edge.h) times.push_back(t);
      }
      // Conservative: an edge starting above the ceiling cannot accelerate
      // even after crossing below it inside that edge. The next edge can.
      for(double t:times)
        if(edge.acceleration(t)>std::max(0.0,ego.acceleration_mps2-
                 p_.max_longitudinal_jerk_mps3*(edge.t0+t))+1e-8) return false;
    }
    return belowRecovery(edge,recovery);
  }

  std::vector<PathPoint> sampleEdge(const ArcPath& arc,const SpeedEdge& edge) const {
    std::vector<double> times{0,edge.h};
    const double end=edge.s0+edge.distance(edge.h);
    // Retain every change in spatial/yaw/curvature interpolation. A redundant
    // straight knot is NOT a breakpoint; omitting it avoids near-coincident
    // geometry and speed-phase samples and ill-conditioned cubic coefficients.
    auto knot=std::upper_bound(arc.interpolation_knots.begin(),arc.interpolation_knots.end(),edge.s0+1e-10);
    for(;knot!=arc.interpolation_knots.end() && *knot<end-1e-10;++knot) {
      double lo=0,hi=edge.h;
      for(int k=0;k<45;++k) {
        const double mid=(lo+hi)/2;
        if(edge.s0+edge.distance(mid)<*knot) lo=mid; else hi=mid;
      }
      times.push_back((lo+hi)/2);
    }
    std::sort(times.begin(),times.end());
    times.erase(std::unique(times.begin(),times.end(),[](double a,double b){return std::abs(a-b)<1e-10;}),times.end());
    // Subdivide AFTER inserting geometry knots. Unioning two independent grids
    // creates near-coincident samples and ill-conditioned Hermite coefficients.
    std::vector<double> refined{times.front()};
    for(std::size_t i=1;i<times.size();++i) {
      const double span=times[i]-times[i-1];
      const int count=std::max(1,static_cast<int>(std::ceil(span/p_.trajectory_output_step_sec)));
      for(int k=1;k<=count;++k) refined.push_back(times[i-1]+span*k/count);
    }
    std::vector<PathPoint> out;
    for(double t:refined) {
      auto q=arc.at(edge.s0+edge.distance(t));
      q.time_sec=edge.t0+t; q.speed_mps=edge.speed(t); q.acceleration_mps2=edge.acceleration(t);
      // Endpoint arithmetic must retain exact zero v/a for a stopped state.
      if(t==0) { q.speed_mps=edge.v0; q.acceleration_mps2=edge.a0; }
      if(t==edge.h) { q.speed_mps=edge.v1; q.acceleration_mps2=edge.a1; }
      out.push_back(q);
    }
    return out;
  }
  bool edgeClear(const Context& c,const ArcPath& arc,const SpeedEdge& edge) const {
    if(c.predictions.current.empty()) return true;
    const auto path=sampleEdge(arc,edge);
    double max_acceleration=0;
    for(double t:edge.extrema()) max_acceleration=std::max(max_acceleration,std::abs(edge.acceleration(t)));
    const double radius=std::hypot(std::max(p_.vehicle.front_m,p_.vehicle.rear_m),p_.vehicle.width_m/2);
    for(std::size_t i=1;i<path.size();++i) {
      const auto& a=path[i-1]; const auto& b=path[i];
      const double dt=b.time_sec-a.time_sec;
      const double d=geometry::norm(geometry::sub(b.pose.position,a.pose.position));
      const double angle=std::abs(geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad));
      // |a| dt²/8 bounds cubic-speed position deviation from a linear time
      // chord. Inflate for both translation and yaw's arc/time discrepancy.
      const double pad=max_acceleration*dt*dt/8*(1+(d>1e-12 ? radius*angle/d : 0));
      if(collision(c,a,b,pad)) return false;
    }
    return true;
  }

  // Minimum jerk-limited stopping distance on this fixed arc. Ramp to negative
  // peak acceleration, optionally hold -D, then ramp to a=0 at v=0. States
  // below v=a²/(2J) with a<0 cannot reach rest without reversing in this model.
  double brakingDistance(double v,double a) const {
    const double j=p_.max_longitudinal_jerk_mps3,d=p_.max_deceleration_mps2;
    if(a<0 && v<a*a/(2*j)-1e-8) return std::numeric_limits<double>::infinity();
    const double peak=std::sqrt(j*v+a*a/2);
    // The release-only boundary v=a²/(2J) has zero initial ramp. Rounded
    // sqrt can put peak one ulp below |a|; the feasibility guard above, not
    // the sign of that rounding residual, decides whether reversal is needed.
    const double b=std::min(d,peak),ramp=std::max(0.0,(a+b)/j);
    double s=v*ramp+a*ramp*ramp/2-j*ramp*ramp*ramp/6;
    v+=a*ramp-j*ramp*ramp/2;
    if(b==0) return 0;
    const double hold=std::max(0.0,(v-b*b/(2*j))/b);
    s+=v*hold-b*hold*hold/2; v-=b*hold;
    const double release=b/j;
    return std::max(0.0,s+v*release-b*release*release/2+j*release*release*release/6);
  }
  double staticStopArc(const Context& c,const ArcPath& arc) const {
    double stop=std::numeric_limits<double>::infinity();
    for(std::size_t j=0;j<c.predictions.current.size();++j) {
      const auto& o=c.predictions.current[j];
      bool stationary=geometry::norm(o.velocity_mps)<1e-10;
      for(const auto& slice:c.predictions.future)
        stationary=stationary && geometry::norm(geometry::sub(slice[j].pose.position,o.pose.position))<1e-10 &&
                   std::abs(geometry::wrap(slice[j].pose.yaw_rad-o.pose.yaw_rad))<1e-10;
      if(!stationary) continue;
      // Include the largest supplied prediction uncertainty, not just t=0.
      auto inflated=o;
      for(const auto& slice:c.predictions.future)
        inflated.radius_error_m=std::max(inflated.radius_error_m,slice[j].radius_error_m);
      const auto other=geometry::objectBox(inflated,p_.obstacle_margin_m);
      const auto overlaps=[&](Pose2 a,Pose2 b) {
        const auto envelope=geometry::sweptEgoEnvelope(a,b,p_.vehicle,0);
        return geometry::sweptOverlap(envelope,envelope,other,other);
      };
      if(overlaps(arc.points.front().pose,arc.points.front().pose)) return -1;
      for(std::size_t i=1;i<arc.points.size() && arc.station[i-1]<stop;++i) {
        if(!overlaps(arc.points[i-1].pose,arc.points[i].pose)) continue;
        double lo=arc.station[i-1],hi=arc.station[i];
        for(int k=0;k<40;++k) {
          const double mid=(lo+hi)/2;
          if(overlaps(arc.points[i-1].pose,arc.at(mid).pose)) hi=mid; else lo=mid;
        }
        stop=std::min(stop,lo-1e-5); break;
      }
    }
    return stop;
  }

  std::vector<PathPoint> planSpeed(const Context& c,const ArcPath& arc,const EgoState& ego,
                                 const PlanningConstraints& limits,double duration,double stop_arc) const {
    auto start=arc.points.front(); start.time_sec=0;
    if(!roadClear(c,start.pose,start.pose) || collision(c,start,start)) return {};
    const double cap=std::min(p_.max_speed_mps,limits.maximum_speed_mps);
    const double origin_s=c.reference.project(start.pose.position,0,c.reference.length()).s;
    const auto recovery=ceilingRecovery(ego,limits);
    const double persistent_stop=std::min(staticStopArc(c,arc),
                                         limits.release_time_sec>=duration ? stop_arc :
                                         std::numeric_limits<double>::infinity());
    const double available_space=std::min(persistent_stop,arc.length());
    if(available_space<0 || brakingDistance(ego.speed_mps,ego.acceleration_mps2)>available_space) return {};
    const bool finish_stopped=cap==0 || persistent_stop<std::max(cap,ego.speed_mps)*duration;
    std::vector<double> clock{0};
    for(double t=p_.speed_time_step_sec;t<duration-1e-10;t+=p_.speed_time_step_sec) clock.push_back(t);
    // Prioritize analytic events over almost coincident regular samples.
    // Absolute-time subtraction on ~1e-9 s Hermite intervals amplifies ulp
    // error into spurious acceleration/jerk during the independent recheck.
    std::vector<double> events;
    const auto event=[&](double t) { if(t>0 && t<duration) events.push_back(t); };
    if(limits.release_time_sec>0 && limits.release_time_sec<duration) event(limits.release_time_sec);
    for(std::size_t i=0;i<recovery.count;++i) {
      const double t=recovery.phase[i].t+recovery.phase[i].h;
      event(t);
    }
    if(finish_stopped) {
      auto stop_limits=limits; stop_limits.maximum_speed_mps=0;
      const auto stop_recovery=ceilingRecovery(ego,stop_limits);
      for(std::size_t i=0;i<stop_recovery.count;++i) {
        const double t=stop_recovery.phase[i].t+stop_recovery.phase[i].h;
        event(t);
      }
    }
    // Only regular samples can be replaced. A short saturated-acceleration
    // phase or a stop just before the horizon still needs both analytic knots.
    clock.erase(std::remove_if(clock.begin(),clock.end(),[&](double t) {
      return t>0 && std::any_of(events.begin(),events.end(),[&](double q){return std::abs(t-q)<1e-6;});
    }),clock.end());
    clock.insert(clock.end(),events.begin(),events.end());
    clock.push_back(duration); std::sort(clock.begin(),clock.end());
    clock.erase(std::unique(clock.begin(),clock.end(),[](double a,double b){return std::abs(a-b)<1e-10;}),clock.end());
    using State=SpeedState;
    std::vector<std::vector<State>> layers{{{0,ego.speed_mps,ego.acceleration_mps2,0,-1}}};
    for(std::size_t layer=1;layer<clock.size();++layer) {
      const double dt=clock[layer]-clock[layer-1];
      std::map<std::tuple<long long,long long,long long>,std::vector<State>> cells;
      const auto& before=layers.back();
      for(std::size_t index=0;index<before.size();++index) {
        const auto& q=before[index];
        const double jerk=p_.max_longitudinal_jerk_mps3;
        const double a_lo=std::max(-p_.max_deceleration_mps2,q.a-jerk*dt);
        const double a_hi=std::min(p_.max_acceleration_mps2,q.a+jerk*dt);
        std::vector<double> accelerations{a_lo,a_hi,geometry::clamp(0.0,a_lo,a_hi),
          geometry::clamp(-p_.max_deceleration_mps2/2,a_lo,a_hi),
          geometry::clamp(p_.max_acceleration_mps2/2,a_lo,a_hi)};
        for(double j : {-1.0,-.5,0.0,.5,1.0})
          accelerations.push_back(geometry::clamp(q.a+j*jerk*dt,a_lo,a_hi));
        std::sort(accelerations.begin(),accelerations.end());
        accelerations.erase(std::unique(accelerations.begin(),accelerations.end()),accelerations.end());
        const double reach=std::min(arc.length(),q.s+q.v*dt+std::max(0.0,q.a)*dt*dt/2+jerk*dt*dt*dt/6);
        const double nearby_curvature=arc.curvature(q.s,reach);
        const double curve_cap=nearby_curvature>1e-12 ?
          std::min(cap,std::sqrt(p_.max_lateral_accel_mps2/nearby_curvature)) : cap;
        for(double a:accelerations) {
          // Endpoint jerk of cubic Hermite v(t) is affine in v1. Intersect
          // both exact jerk intervals BEFORE allocating/checking an arc edge.
          const double lo=std::max(0.0,q.v+std::max((4*q.a+2*a)*dt-jerk*dt*dt,
                                                   (2*q.a+4*a)*dt-jerk*dt*dt)/6);
          const double hi=std::min(p_.max_speed_mps,q.v+std::min((4*q.a+2*a)*dt+jerk*dt*dt,
                                                                 (2*q.a+4*a)*dt+jerk*dt*dt)/6);
          if(lo>hi+1e-12) continue;
          const double upper=std::max(lo,hi);
          std::vector<double> velocities{lo,upper};
          for(double v : {q.v+(q.a+a)*dt/2,0.0,cap,q.v,curve_cap})
            velocities.push_back(geometry::clamp(v,lo,upper));
          std::sort(velocities.begin(),velocities.end());
          velocities.erase(std::unique(velocities.begin(),velocities.end()),velocities.end());
          for(double v:velocities) {
            if(v<0 || v>p_.max_speed_mps) continue;
            const SpeedEdge edge{q.s,q.v,q.a,v,a,clock[layer-1],dt};
            // Interior acceleration/speed and lowered-ceiling recovery still
            // require the full polynomial check. Do cheap dynamics before
            // curvature traversal, temporal footprint sampling or SAT.
            if(!speedAdmissible(edge,ego,limits,0,recovery)) continue;
            const double s=q.s+edge.distance(dt);
            if(!std::isfinite(s) || s<q.s-1e-9 || s>arc.length()+1e-9 ||
               s+brakingDistance(v,a)>available_space ||
               (clock[layer-1]<limits.release_time_sec &&
                q.s+edge.distance(std::min(dt,limits.release_time_sec-clock[layer-1]))>stop_arc+1e-9)) continue;
            const double curvature=arc.curvature(q.s,s);
            if(curvature>p_.max_curvature_inv_m+1e-6) continue;
            bool lateral_clear=true;
            for(double t:edge.extrema())
              if(std::pow(edge.speed(t),2)*curvature>p_.max_lateral_accel_mps2+1e-6) lateral_clear=false;
            if(!lateral_clear || !edgeClear(c,arc,edge)) continue;
            const double cost=q.cost+edgeCost(c,arc,edge,origin_s,cap);
            // Exact rest must not share a quantization cell with creeping or
            // nonzero acceleration. Those states cannot substitute for a wait.
            const auto velocity_cell=v==0 ? 0LL : 1+std::llround(v/.25);
            const auto acceleration_cell=a==0 ? 0LL : (a>0 ? 1LL : -1LL)+std::llround(a/.5);
            const auto key=std::make_tuple(std::llround(s/.25),velocity_cell,acceleration_cell);
            const State next{s,v,a,cost,static_cast<int>(index)};
            retainSpeedState(cells[key],next);
          }
        }
      }
      std::vector<State> next;
      for(const auto& cell:cells) next.insert(next.end(),cell.second.begin(),cell.second.end());
      if(layer+1==clock.size() && finish_stopped)
        next.erase(std::remove_if(next.begin(),next.end(),[](const State& q){return q.v!=0 || q.a!=0;}),next.end());
      if(next.empty()) return {};
      pruneSpeedLayer(next,cap);
      layers.push_back(std::move(next));
    }
    int selected=0;
    std::vector<SpeedEdge> edges;
    for(std::size_t i=layers.size()-1;i>0;--i) {
      const auto& q=layers[i][static_cast<std::size_t>(selected)];
      const auto& a=layers[i-1][static_cast<std::size_t>(q.parent)];
      edges.push_back({a.s,a.v,a.a,q.v,q.a,clock[i-1],clock[i]-clock[i-1]});
      selected=q.parent;
    }
    std::reverse(edges.begin(),edges.end());
    std::vector<PathPoint> out;
    for(const auto& edge:edges) {
      const auto sampled=sampleEdge(arc,edge);
      out.insert(out.end(),sampled.begin()+(out.empty() ? 0 : 1),sampled.end());
    }
    out.front().pose=ego.pose; out.front().curvature_inv_m=ego.curvature_inv_m;
    return out;
  }
  void validateParameters() const {
    for (double x : {p_.vehicle.front_m,p_.vehicle.rear_m,p_.vehicle.width_m,p_.horizon_m,p_.dp_step_m,
                    p_.lateral_step_m,p_.dp_slope_sample,p_.output_step_m,p_.corridor_half_width_m,p_.max_curvature_inv_m,
                    p_.max_lateral_accel_mps2,p_.max_speed_mps,p_.max_lateral_slope,p_.max_initial_heading_error_rad,
                    p_.max_initial_distance_m,p_.qp_rho,p_.qp_tolerance,p_.min_horizon_m,p_.road_boundary_step_m,
                    p_.max_acceleration_mps2,p_.max_deceleration_mps2,p_.max_longitudinal_jerk_mps3,
                    p_.speed_time_step_sec,p_.trajectory_output_step_sec,p_.trajectory_horizon_m})
      if (!std::isfinite(x) || x<=0.0) throw std::invalid_argument("nonpositive/nonfinite path parameter");
    for (double x : {p_.obstacle_margin_m,p_.road_margin_m,p_.reference_weight,p_.dp_tracking_weight,
                    p_.slope_weight,p_.bending_weight,p_.jerk_weight,p_.previous_weight,
                    p_.speed_tracking_weight,p_.acceleration_weight,p_.longitudinal_jerk_weight})
      if (!std::isfinite(x) || x<0.0) throw std::invalid_argument("negative/nonfinite path weight or margin");
    if (p_.horizon_m/p_.output_step_m>500 || p_.horizon_m/p_.dp_step_m>100 ||
        p_.output_step_m>0.5 || p_.road_boundary_step_m>0.5 || p_.min_horizon_m>p_.horizon_m ||
        p_.qp_max_iterations<1 ||
        p_.max_lateral_samples<1 || p_.reference_weight+p_.dp_tracking_weight<=0.0 ||
        p_.max_initial_heading_error_rad>=1.0 || p_.dp_slope_sample>p_.max_lateral_slope ||
        p_.speed_time_step_sec<.1 || p_.speed_time_step_sec>1 || p_.trajectory_output_step_sec<.01 ||
        p_.trajectory_output_step_sec>.1 || p_.speed_beam_width<1 || p_.speed_beam_width>4096)
      throw std::invalid_argument("unbounded or inconsistent planner discretization");
  }
  // Only domains needed by interpolation/indexing/optimization are checked.
  // No transport freshness, receipt health, frame selection, or recovery policy.
  void requireSceneDomain(const EgoState& ego,const std::vector<ReferencePoint>& ref,const Predictions& pred) const {
    if (!geometry::finite(ego.pose) || !std::isfinite(ego.stamp_sec) || ego.stamp_sec<0.0 ||
        ref.size()<2 ||
        !std::isfinite(pred.reference_stamp_sec) || pred.reference_stamp_sec<0.0 ||
        !std::isfinite(pred.step_sec) || pred.step_sec<=0.0 || pred.step_sec>2.0)
      throw std::invalid_argument("invalid calculation epoch, horizon or array dimensions");
    for (const auto& r : ref)
      if (!geometry::finite(r.position) || !std::isfinite(r.min_l_m) || !std::isfinite(r.max_l_m) || r.min_l_m>=r.max_l_m)
        throw std::invalid_argument("invalid reference boundary");
    const auto validate_slice=[&](const std::vector<PredictedObject>& slice) {
      if (slice.size()!=pred.current.size()) throw std::invalid_argument("prediction slice loses objects");
      for (std::size_t j=0;j<slice.size();++j) {
        const auto& o=slice[j];
        if (o.id!=pred.current[j].id || !geometry::finite(o.pose) || !geometry::finite(o.velocity_mps) ||
            !std::isfinite(o.length_m) || o.length_m<=0.0 || !std::isfinite(o.width_m) || o.width_m<=0.0 ||
            !std::isfinite(o.radius_error_m) || o.radius_error_m<0.0 ||
            !std::isfinite(o.interpolation_acceleration_mps2) || o.interpolation_acceleration_mps2<0.0 ||
            !std::isfinite(o.source_stamp_sec) || o.source_stamp_sec<0.0 || o.source_stamp_sec>pred.reference_stamp_sec ||
            o.source_stamp_sec!=pred.current[j].source_stamp_sec || o.length_m!=pred.current[j].length_m ||
            o.width_m!=pred.current[j].width_m || o.kind!=pred.current[j].kind || o.model!=pred.current[j].model)
          throw std::invalid_argument("invalid prediction object or ID ordering");
      }
    };
    validate_slice(pred.current);
    for (const auto& slice : pred.future) validate_slice(slice);
  }
  static void requireForwardSchedule(const std::vector<ProgressTime>& schedule) {
    if (schedule.size()<2 || schedule.front().progress_m!=0.0 || schedule.front().time_sec!=0.0)
      throw std::invalid_argument("schedule must start at (0 m, 0 s) and have at least two points");
    for (std::size_t i=0;i<schedule.size();++i) {
      if (!std::isfinite(schedule[i].progress_m) || !std::isfinite(schedule[i].time_sec) ||
          (i>0 && (schedule[i].progress_m<=schedule[i-1].progress_m || schedule[i].time_sec<=schedule[i-1].time_sec)))
        throw std::invalid_argument("forward schedule must be strictly increasing");
    }
  }
  static double timeAt(const std::vector<ProgressTime>& schedule,double s) {
    const auto it=std::lower_bound(schedule.begin(),schedule.end(),s,[](const ProgressTime& a,double b){return a.progress_m<b;});
    if (it==schedule.begin()) return it->time_sec;
    if (it==schedule.end()) return schedule.back().time_sec;
    const auto& a=*(it-1);
    return geometry::lerp(a.time_sec,it->time_sec,(s-a.progress_m)/(it->progress_m-a.progress_m));
  }
  static double progressAt(const std::vector<ProgressTime>& schedule,double time) {
    const auto it=std::lower_bound(schedule.begin(),schedule.end(),time,
                                  [](const ProgressTime& a,double b){return a.time_sec<b;});
    if (it==schedule.begin()) return it->progress_m;
    if (it==schedule.end()) return schedule.back().progress_m;
    const auto& a=*(it-1);
    return geometry::lerp(a.progress_m,it->progress_m,(time-a.time_sec)/(it->time_sec-a.time_sec));
  }
  // Preserve the piecewise-linear geometry, but retain every change in its
  // supplied timing. Otherwise a short fast/slow interval can be averaged away
  // between output samples, invalidating both speed and collision checks.
  static std::vector<PathPoint> withScheduleKnots(const std::vector<PathPoint>& path,
                                                 const std::vector<ProgressTime>& schedule) {
    if (path.empty()) return {};
    std::vector<PathPoint> out{path.front()};
    auto knot=std::upper_bound(schedule.begin(),schedule.end(),path.front().progress_m,
                              [](double s,const ProgressTime& p){return s<p.progress_m;});
    for (std::size_t i=1;i<path.size();++i) {
      const auto& a=path[i-1]; const auto& b=path[i];
      while (knot!=schedule.end() && knot->progress_m<b.progress_m) {
        const double u=(knot->progress_m-a.progress_m)/(b.progress_m-a.progress_m);
        // Max endpoint magnitude is conservative for the interpolated curvature
        // constraint; geometry and yaw follow the existing sampled path.
        const double curvature=std::abs(a.curvature_inv_m)>std::abs(b.curvature_inv_m) ?
                               a.curvature_inv_m : b.curvature_inv_m;
        out.push_back({{geometry::lerp(a.pose.position,b.pose.position,u),
                        geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u)},
                       knot->progress_m,knot->time_sec,curvature});
        ++knot;
      }
      out.push_back(b);
      if (knot!=schedule.end() && knot->progress_m==b.progress_m) ++knot;
    }
    return out;
  }
  static double quintic(double a,double b,double initial_dl,double final_dl,double h,double u) {
    const double d=b-a, v=initial_dl*h,w=final_dl*h;
    return a+v*u+(10*d-6*v-4*w)*u*u*u+(-15*d+8*v+7*w)*u*u*u*u+(6*d-3*v-3*w)*u*u*u*u*u;
  }
  static void setGeometry(std::vector<PathPoint>& points) {
    for (std::size_t i=0;i<points.size();++i) {
      const auto before=points[i==0 ? 0 : i-1].pose.position;
      const auto after=points[i+1==points.size() ? i : i+1].pose.position;
      const auto d=geometry::sub(after,before);
      points[i].pose.yaw_rad=std::atan2(d.y,d.x);
      if (i>0 && i+1<points.size()) points[i].curvature_inv_m=geometry::curvature(before,points[i].pose.position,after);
    }
    if (points.size()>2) { points.front().curvature_inv_m=points[1].curvature_inv_m; points.back().curvature_inv_m=points[points.size()-2].curvature_inv_m; }
  }
  std::vector<PathPoint> makeEdge(const Context& c,double s0,double l0,double s1,double l1,double dl,double end_dl) const {
    const int steps=std::max(2,static_cast<int>(std::ceil((s1-s0)/p_.output_step_m)));
    std::vector<PathPoint> out;
    for (int i=0;i<=steps;++i) {
      const double u=static_cast<double>(i)/steps, s=geometry::lerp(s0,s1,u);
      const auto r=c.reference.at(c.start_s+s);
      const double l=quintic(l0,l1,dl,end_dl,s1-s0,u);
      out.push_back({{geometry::add(r.position,geometry::mul(geometry::normal(r.yaw),l)),r.yaw},s,timeAt(c.schedule,s),0.0});
    }
    setGeometry(out);
    const auto first=c.reference.at(c.start_s+s0),last=c.reference.at(c.start_s+s1);
    out.front().pose.yaw_rad=geometry::wrap(first.yaw+std::atan2(dl,first.metric*(1-first.curvature*l0)));
    out.back().pose.yaw_rad=geometry::wrap(last.yaw+std::atan2(end_dl,last.metric*(1-last.curvature*l1)));
    return withScheduleKnots(out,c.schedule);
  }
  double previousL(const Reference& ref,double s,const std::vector<PathPoint>& path) const {
    if (path.empty()) return 0.0;
    const auto target=ref.at(s);
    double best=std::numeric_limits<double>::infinity(), l=0.0;
    for (const auto& point : path) {
      const auto d=geometry::sub(point.pose.position,target.position);
      const double longitudinal=std::abs(geometry::dot(d,geometry::unit(target.yaw)));
      if (longitudinal<best && longitudinal<1.0 && std::abs(geometry::wrap(point.pose.yaw_rad-target.yaw))<0.8) {
        best=longitudinal; l=geometry::dot(d,geometry::normal(target.yaw));
      }
    }
    return l;
  }
  bool roadClear(const Context& c,Pose2 first,Pose2 last,int depth=0) const {
    if (geometry::polygonContainsBox(c.boundary,
          geometry::sweptEgoEnvelope(first,last,p_.vehicle,p_.road_margin_m))) return true;
    if (depth>=6 || (geometry::norm(geometry::sub(first.position,last.position))<1e-9 &&
                    std::abs(geometry::wrap(first.yaw_rad-last.yaw_rad))<1e-9)) return false;
    if (!geometry::polygonContainsBox(c.boundary,
          geometry::sweptEgoEnvelope(first,first,p_.vehicle,p_.road_margin_m)) ||
        !geometry::polygonContainsBox(c.boundary,
          geometry::sweptEgoEnvelope(last,last,p_.vehicle,p_.road_margin_m))) return false;
    // Refine an over-conservative enclosure, never replace it with point tests.
    const Pose2 mid{geometry::lerp(first.position,last.position,0.5),
                    geometry::yawLerp(first.yaw_rad,last.yaw_rad,0.5)};
    return roadClear(c,first,mid,depth+1) && roadClear(c,mid,last,depth+1);
  }
  bool collision(const Context& c,const PathPoint& a,const PathPoint& b,double motion_pad=0) const {
    const double t0=a.time_sec+c.prediction_offset,t1=b.time_sec+c.prediction_offset;
    std::vector<double> cuts{t0};
    for (std::size_t k=1;k<=kPredictionSteps;++k) {
      const double t=k*c.predictions.step_sec;
      if (t>t0+1e-10 && t<t1-1e-10) cuts.push_back(t);
    }
    cuts.push_back(t1);
    for (std::size_t k=1;k<cuts.size();++k) {
      const auto ego_at=[&](double t) {
        const double u=t1>t0 ? (t-t0)/(t1-t0) : 0.0;
        return geometry::egoBox({geometry::lerp(a.pose.position,b.pose.position,u),geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u)},p_.vehicle);
      };
      auto e0=ego_at(cuts[k-1]),e1=ego_at(cuts[k]);
      const double angle=std::abs(geometry::wrap(e1.pose.yaw_rad-e0.pose.yaw_rad));
      const double offset=std::abs(p_.vehicle.front_m-p_.vehicle.rear_m)/2.0;
      const double center_arc_pad=offset*angle*angle/8.0;
      e0.half_length+=center_arc_pad+motion_pad; e0.half_width+=center_arc_pad+motion_pad;
      e1.half_length+=center_arc_pad+motion_pad; e1.half_width+=center_arc_pad+motion_pad;
      for (std::size_t j=0;j<c.predictions.current.size();++j) {
        const auto o0=samplePrediction(c.predictions,j,cuts[k-1]);
        const auto o1=samplePrediction(c.predictions,j,cuts[k]);
        if (geometry::sweptOverlap(e0,e1,geometry::objectBox(o0,p_.obstacle_margin_m),geometry::objectBox(o1,p_.obstacle_margin_m))) return true;
      }
    }
    return false;
  }
  bool checkPath(const Context& c,const std::vector<PathPoint>& path) const {
    for (std::size_t i=0;i<path.size();++i) {
      if (!geometry::finite(path[i].pose) || !std::isfinite(path[i].curvature_inv_m) ||
          std::abs(path[i].curvature_inv_m)>p_.max_curvature_inv_m+1e-6) return false;
      if (i==0) {
        if (!roadClear(c,path[i].pose,path[i].pose) || (!c.geometric_only && collision(c,path[i],path[i]))) return false;
        continue;
      }
      const auto& a=path[i-1]; const auto& b=path[i];
      const double dt=b.time_sec-a.time_sec;
      const double distance=geometry::norm(geometry::sub(b.pose.position,a.pose.position));
      if (!std::isfinite(dt) || dt<=0.0 || !std::isfinite(distance)) return false;
      const double angle=std::abs(geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad));
      // Waiting is allowed in a supplied final timing, but rotating a car in
      // place is not. Forward generation itself still uses increasing progress.
      if (distance==0.0 && (angle>1e-10 || a.progress_m!=b.progress_m)) return false;
      const double speed=distance/dt;
      const double curvature=std::max({std::abs(a.curvature_inv_m),std::abs(b.curvature_inv_m),
                                      distance>0.0 ? angle/distance : 0.0});
      if (distance>0.0 && (geometry::dot(geometry::sub(b.pose.position,a.pose.position),geometry::unit(a.pose.yaw_rad))<=0.0 ||
                          geometry::dot(geometry::sub(b.pose.position,a.pose.position),geometry::unit(b.pose.yaw_rad))<=0.0)) return false;
      if (curvature>p_.max_curvature_inv_m+1e-6 || (!c.geometric_only &&
          (!std::isfinite(speed) || speed>p_.max_speed_mps || speed*speed*curvature>p_.max_lateral_accel_mps2+1e-6)))
        return false;
      if (!roadClear(c,a.pose,b.pose) || (!c.geometric_only && collision(c,a,b))) return false;
    }
    return true;
  }
  bool solveQp(const Eigen::VectorXd& guide,const Eigen::VectorXd& prior,const Eigen::VectorXd& lo,
                   const Eigen::VectorXd& hi,double ds,double initial_dl,Eigen::VectorXd& output) const {
    const Eigen::Index n=guide.size();
    Eigen::MatrixXd d1=Eigen::MatrixXd::Zero(n-1,n),d2=Eigen::MatrixXd::Zero(n-2,n),d3=Eigen::MatrixXd::Zero(std::max<Eigen::Index>(0,n-3),n);
    for (Eigen::Index i=0;i<n-1;++i) { d1(i,i)=-1.0/ds; d1(i,i+1)=1.0/ds; }
    for (Eigen::Index i=0;i<n-2;++i) { d2(i,i)=1.0/(ds*ds); d2(i,i+1)=-2.0/(ds*ds); d2(i,i+2)=1.0/(ds*ds); }
    for (Eigen::Index i=0;i<n-3;++i) { d3(i,i)=-1.0/(ds*ds*ds); d3(i,i+1)=3.0/(ds*ds*ds); d3(i,i+2)=-3.0/(ds*ds*ds); d3(i,i+3)=1.0/(ds*ds*ds); }
    Eigen::MatrixXd h=2.0*((p_.reference_weight+p_.dp_tracking_weight+p_.previous_weight)*Eigen::MatrixXd::Identity(n,n)+
      p_.slope_weight*d1.transpose()*d1+p_.bending_weight*d2.transpose()*d2+p_.jerk_weight*d3.transpose()*d3);
    Eigen::VectorXd q=-2.0*(p_.dp_tracking_weight*guide+p_.previous_weight*prior);
    Eigen::MatrixXd a(3*n-3,n);
    a.topRows(n).setIdentity(); a.middleRows(n,n-1)=d1; a.bottomRows(n-2)=d2;
    Eigen::VectorXd lower(3*n-3),upper(3*n-3);
    lower.head(n)=lo; upper.head(n)=hi;
    lower.segment(n,n-1).setConstant(-p_.max_lateral_slope); upper.segment(n,n-1).setConstant(p_.max_lateral_slope);
    lower[n]=upper[n]=initial_dl;
    // l'' is only an approximate curvature constraint; sampled map curvature
    // and speed-dependent lateral acceleration are checked after optimization.
    lower.tail(n-2).setConstant(-p_.max_curvature_inv_m); upper.tail(n-2).setConstant(p_.max_curvature_inv_m);
    // Eliminate the initial position/slope equalities exactly. Penalizing these
    // with ADMM alone both converges slowly and perturbs the execution boundary.
    Eigen::VectorXd fixed=Eigen::VectorXd::Zero(n);
    fixed[0]=lo[0]; fixed[1]=fixed[0]+ds*initial_dl;
    if (fixed[1]<lo[1] || fixed[1]>hi[1]) return false;
    q=(q+h*fixed).tail(n-2).eval();
    h=h.bottomRightCorner(n-2,n-2).eval();
    const Eigen::VectorXd fixed_constraint=a*fixed;
    lower-=fixed_constraint; upper-=fixed_constraint;
    a=a.rightCols(n-2).eval();
    const Eigen::MatrixXd ata=a.transpose()*a;
    double rho=p_.qp_rho;
    Eigen::LDLT<Eigen::MatrixXd> factor(h+rho*ata);
    if (factor.info()!=Eigen::Success) return false;
    Eigen::VectorXd x=guide.tail(n-2),z=a*x,u=Eigen::VectorXd::Zero(a.rows());
    for (Eigen::Index j=0;j<z.size();++j) z[j]=geometry::clamp(z[j],lower[j],upper[j]);
    for (int iteration=1;iteration<=p_.qp_max_iterations;++iteration) {
      x=factor.solve(-q+rho*a.transpose()*(z-u));
      if (!x.allFinite()) return false;
      const Eigen::VectorXd ax=a*x;
      const Eigen::VectorXd relaxed=1.6*ax-0.6*z;
      z=relaxed+u;
      for (Eigen::Index j=0;j<z.size();++j) z[j]=geometry::clamp(z[j],lower[j],upper[j]);
      u+=relaxed-z;
      const double primal_residual=(ax-z).lpNorm<Eigen::Infinity>();
      const double dual_residual=(h*x+q+rho*a.transpose()*u).lpNorm<Eigen::Infinity>();
      if (primal_residual<=p_.qp_tolerance && dual_residual<=p_.qp_tolerance) {
        output=fixed; output.tail(n-2)=x; return true;
      }
      if (iteration%100==0 && refineActiveSet(h,q,a,lower,upper,z,x)) {
        output=fixed; output.tail(n-2)=x; return true;
      }
      if (iteration%50==0) {
        const double ratio=primal_residual/std::max(dual_residual,1e-15);
        if (ratio>3.0 || ratio<1.0/3.0) {
          const double next=geometry::clamp(rho*std::sqrt(ratio),0.1,1e4);
          u*=rho/next; rho=next;
          factor.compute(h+rho*ata);
          if (factor.info()!=Eigen::Success) return false;
        }
      }
    }
    // Iteration limit is not proof of physical infeasibility.
    return false;
  }
  bool refineActiveSet(const Eigen::MatrixXd& h,const Eigen::VectorXd& q,const Eigen::MatrixXd& a,
                       const Eigen::VectorXd& lower,const Eigen::VectorXd& upper,const Eigen::VectorXd& z,
                       Eigen::VectorXd& x) const {
    struct Active { Eigen::Index row; bool upper; };
    std::vector<Active> active;
    for (Eigen::Index i=0;i<a.rows();++i) {
      if (a.row(i).squaredNorm()<1e-16) continue;
      if (std::abs(z[i]-lower[i])<1e-8) active.push_back({i,false});
      else if (std::abs(z[i]-upper[i])<1e-8) active.push_back({i,true});
    }
    const Eigen::Index n=h.rows();
    for (int iteration=0;iteration<40;++iteration) {
      const Eigen::Index m=static_cast<Eigen::Index>(active.size());
      Eigen::MatrixXd kkt=Eigen::MatrixXd::Zero(n+m,n+m);
      Eigen::VectorXd rhs(n+m);
      kkt.topLeftCorner(n,n)=h; rhs.head(n)=-q;
      for (Eigen::Index j=0;j<m;++j) {
        const auto item=active[static_cast<std::size_t>(j)];
        kkt.block(n+j,0,1,n)=a.row(item.row);
        kkt.block(0,n+j,n,1)=a.row(item.row).transpose();
        rhs[n+j]=item.upper ? upper[item.row] : lower[item.row];
      }
      Eigen::FullPivLU<Eigen::MatrixXd> factor(kkt);
      if (!factor.isInvertible()) return false;
      const Eigen::VectorXd candidate=factor.solve(rhs);
      if (!candidate.allFinite() || (kkt*candidate-rhs).lpNorm<Eigen::Infinity>()>p_.qp_tolerance) return false;
      const Eigen::VectorXd ax=a*candidate.head(n);
      double violation=p_.qp_tolerance;
      Active add{-1,false};
      for (Eigen::Index i=0;i<a.rows();++i) {
        if (lower[i]-ax[i]>violation) { violation=lower[i]-ax[i]; add={i,false}; }
        if (ax[i]-upper[i]>violation) { violation=ax[i]-upper[i]; add={i,true}; }
      }
      if (add.row>=0) {
        if (std::any_of(active.begin(),active.end(),[&](const Active& v){return v.row==add.row;})) return false;
        active.push_back(add); continue;
      }
      Eigen::Index remove=-1;
      double wrong_sign=p_.qp_tolerance;
      for (Eigen::Index j=0;j<m;++j) {
        const auto item=active[static_cast<std::size_t>(j)];
        if (std::abs(upper[item.row]-lower[item.row])<1e-12) continue;
        const double invalid=item.upper ? -candidate[n+j] : candidate[n+j];
        if (invalid>wrong_sign) { wrong_sign=invalid; remove=j; }
      }
      if (remove>=0) { active.erase(active.begin()+remove); continue; }
      const Eigen::VectorXd dual=h*candidate.head(n)+q+kkt.topRightCorner(n,m)*candidate.tail(m);
      if (dual.lpNorm<Eigen::Infinity>()>p_.qp_tolerance) return false;
      x=candidate.head(n);
      return true;
    }
    return false;
  }
};
}  // namespace stier

#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_core/geometry/LineString.h>

namespace path_planner {
struct CorePathPlannerParameters {
  stier::PathParameters path;
};

// route 중심선·좌우 경계 corridor와 ego pose로 map에 옮긴 base_link 예측으로
// stier::PathGenerationCore를 호출하고 결과 pose를 map 경로로 반환한다.
class CorePathPlanner {
 public:
  CorePathPlanner(CorePathPlannerParameters parameters, SharedMap&) : core_(parameters.path) {}

  Message process(const nav_msgs::Odometry& ego, const Route& route, const PredictedObjects& predictions) {
    stier::EgoState state;
    state.pose.position = {ego.pose.pose.position.x, ego.pose.pose.position.y};
    state.pose.yaw_rad = yawOf(ego.pose.pose.orientation);
    state.stamp_sec = ego.header.stamp.toSec();
    state.speed_mps = ego.twist.twist.linear.x;
    if (state.stamp_sec != previous_stamp_sec_) {
      if (has_previous_)
        acceleration_mps2_ = (state.speed_mps - previous_speed_mps_) / (state.stamp_sec - previous_stamp_sec_);
      previous_stamp_sec_ = state.stamp_sec;
      previous_speed_mps_ = state.speed_mps;
      has_previous_ = true;
    }
    state.acceleration_mps2 = acceleration_mps2_;

    std::vector<stier::ReferencePoint> reference;
    for (const auto& lanelet : *route) {
      const auto left = lanelet.leftBound2d();
      const auto right = lanelet.rightBound2d();
      for (const auto& point : lanelet.centerline2d()) {
        const lanelet::BasicPoint2d xy(point.x(), point.y());
        stier::ReferencePoint sample;
        sample.position = {xy.x(), xy.y()};
        sample.min_l_m = -lanelet::geometry::distance2d(right, xy);
        sample.max_l_m = lanelet::geometry::distance2d(left, xy);
        reference.push_back(sample);
      }
    }

    stier::Predictions objects_map;
    objects_map.reference_stamp_sec = predictions[0].header.stamp.toSec();
    objects_map.step_sec = 0.5;
    const double cos_yaw = std::cos(state.pose.yaw_rad);
    const double sin_yaw = std::sin(state.pose.yaw_rad);
    for (std::size_t i = 0; i < stier::kPredictionSteps; ++i) {
      for (const auto& object : predictions[i].objects) {
        stier::PredictedObject sample;
        sample.id = object.id;
        sample.pose.position = {state.pose.position.x + cos_yaw * object.position.x - sin_yaw * object.position.y,
                                state.pose.position.y + sin_yaw * object.position.x + cos_yaw * object.position.y};
        sample.pose.yaw_rad = stier::geometry::wrap(state.pose.yaw_rad + yawOf(object.bbox_pose.orientation));
        if (object.velocity_available)
          sample.velocity_mps = {cos_yaw * object.velocity.x - sin_yaw * object.velocity.y,
                                 sin_yaw * object.velocity.x + cos_yaw * object.velocity.y};
        sample.length_m = object.bbox_size.x;
        sample.width_m = object.bbox_size.y;
        sample.source_stamp_sec = objects_map.reference_stamp_sec;
        objects_map.future[i].push_back(sample);
      }
    }
    objects_map.current = objects_map.future[0];

    Message path;
    for (const auto& point : core_.process(state, reference, objects_map, stier::PlanningConstraints{})) {
      interfaces::PathPoint sample;
      sample.pose.pose.position.x = point.pose.position.x;
      sample.pose.pose.position.y = point.pose.position.y;
      sample.pose.pose.orientation.z = std::sin(point.pose.yaw_rad / 2.0);
      sample.pose.pose.orientation.w = std::cos(point.pose.yaw_rad / 2.0);
      path.points.push_back(sample);
    }
    return path;
  }

 private:
  static double yawOf(const geometry_msgs::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  stier::PathGenerationCore core_;
  bool has_previous_ = false;
  double previous_stamp_sec_ = 0.0;
  double previous_speed_mps_ = 0.0;
  double acceleration_mps2_ = 0.0;
};
}  // namespace path_planner
