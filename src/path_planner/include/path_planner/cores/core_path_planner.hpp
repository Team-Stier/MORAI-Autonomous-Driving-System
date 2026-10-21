#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <initializer_list>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <sensor_msgs/Imu.h>
#include <ros/console.h>
#include <path_planner/libraries/types.hpp>
#include <path_planner/banks/shared_map.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

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
  // Equal progress/pose with increasing time represents a wait.
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
  // Soft cost target only; the maximum speed and stop requests remain authoritative.
  double preferred_speed_mps = std::numeric_limits<double>::infinity();
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
  double pedestrian_caution_speed_mps = 0.0;  // 0 disables the optional VRU approach ceiling.
  double pedestrian_caution_distance_m = 0.0;
  double pedestrian_near_preferred_speed_mps = 0.0;  // 0: keep the existing cost target.
  double max_curvature_inv_m = 0.18;
  double max_lateral_accel_mps2 = 2.0;
  double preferred_lateral_accel_mps2 = 0.0;  // 0: use the hard limit as the cost target.
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
  double following_time_headway_sec = 1.5;
  double following_min_gap_m = 2.0;
  // Integrated planning window; the legacy supplied-schedule horizon stays
  // unchanged. Caller must supply reference/prediction coverage for its speed.
  double trajectory_horizon_m = 100.0;
  double normal_speed_limit_mps = 0.0;  // 0: native process의 임무 속도 주입 비활성.
  double exception_target_speed_mps = 0.0;
  double speed_limit_response_sec = 0.5;
  std::set<std::string> speed_exemption_link_ids;
};

}  // namespace stier

namespace path_planner {
using stier::Vec2;
using stier::Pose2;
using stier::ObjectKind;
using stier::MotionModel;
using stier::PredictedObject;
using stier::Predictions;
using stier::ReferencePoint;
using stier::EgoState;
using stier::VehicleShape;
using stier::ProgressTime;
using stier::PathPoint;
using stier::PlanningConstraints;
using stier::PathParameters;
using stier::kPredictionSteps;

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
// #PLANNER_PORT_9773586
// 계산 본문은 choialsgh08-hash의 검증 커밋 9773586에서 보존해 이식한다.
// process의 기존 ROS 인자/반환, map 객체, 뒷차축 ego 기준과 헤더 주입 계약은 유지한다.
// Geometry 범위와 native 계산 API 이름(plan)만 최신 Core 형식에 맞게 치환한다.
// DP/QP·차체 경계·충돌·곡률·가감속 제약의 계산식은 변경하지 않는다.
// Ground Truth는 최신 morai_test의 /objects(map)로 공급하고 LiDAR 검출을 다시 적용하지 않는다.
class CorePathPlanner {
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

struct Box {
  Pose2 pose;
  double half_length = 0.0;
  double half_width = 0.0;
};
static Box egoBox(Pose2 rear_axle, const VehicleShape& v) {
  rear_axle.position = add(rear_axle.position, mul(unit(rear_axle.yaw_rad), (v.front_m-v.rear_m)/2.0));
  return {rear_axle, (v.front_m + v.rear_m)/2.0, v.width_m/2.0};
}
static Box objectBox(const PredictedObject& o, double margin) {
  return {o.pose, o.length_m/2.0 + o.radius_error_m + margin,
                  o.width_m/2.0 + o.radius_error_m + margin};
}
static double extent(const Box& b, Vec2 axis) {
  return b.half_length * std::abs(dot(unit(b.pose.yaw_rad), axis)) +
         b.half_width * std::abs(dot(normal(b.pose.yaw_rad), axis));
}
static std::array<Vec2, 4> corners(const Box& b) {
  const Vec2 x = mul(unit(b.pose.yaw_rad), b.half_length);
  const Vec2 y = mul(normal(b.pose.yaw_rad), b.half_width);
  return {{add(b.pose.position, add(x,y)), add(b.pose.position,sub(x,y)),
           sub(b.pose.position,add(x,y)), add(b.pose.position,sub(y,x))}};
}

// #COLLISION_BROAD_PHASE
// map 좌표의 모든 예측 시점과 경로 꼭짓점을 감싸는 보수적 범위다.
// 접촉하거나 수치 범위가 불확실하면 기존 정밀 검사를 그대로 수행한다.
struct Bounds {
  double min_x=std::numeric_limits<double>::infinity();
  double min_y=std::numeric_limits<double>::infinity();
  double max_x=-std::numeric_limits<double>::infinity();
  double max_y=-std::numeric_limits<double>::infinity();
  void include(Vec2 p) {
    min_x=std::min(min_x,p.x); min_y=std::min(min_y,p.y);
    max_x=std::max(max_x,p.x); max_y=std::max(max_y,p.y);
  }
  bool separated(const Bounds& other,double padding) const {
    const double scale=std::max({1.0,std::abs(min_x),std::abs(min_y),std::abs(max_x),std::abs(max_y),
        std::abs(other.min_x),std::abs(other.min_y),std::abs(other.max_x),std::abs(other.max_y)});
    padding+=1e-6+128*std::numeric_limits<double>::epsilon()*scale;
    if(!std::isfinite(padding) || !std::isfinite(scale)) return false;
    return max_x+padding<other.min_x || other.max_x+padding<min_x ||
           max_y+padding<other.min_y || other.max_y+padding<min_y;
  }
};

// Closed segment/rectangle intersection, including contact. Testing boundary
// segments catches inward notches even when all four vehicle corners are inside.
static bool segmentIntersectsBox(Vec2 a, Vec2 b, const Box& box) {
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
static bool polygonContainsBox(const std::vector<Vec2>& polygon,const Box& box) {
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

// #ROAD_BOUNDARY_INDEX
// map 도로 경계 전체를 유지하면서 차체와 만날 수 없는 선분만 먼저 제외한다.
// 트리는 선분의 보수적 AABB를 저장한다. 실제 접촉 판정은 기존 함수 그대로다.
// 내부/외부 판정도 기존 수평 ray의 식과 경계 포함 규칙을 유지한다.
// ROI를 줄이거나 오목 경계·차체 모서리·회전 구간의 안전 검사를 생략하지 않는다.
struct BoundaryIndex {
  struct Node {
    double min_x, min_y, max_x, max_y;
    std::size_t begin, end, left=0, right=0;
  };
  const std::vector<Vec2>& polygon;
  std::vector<std::size_t> order;
  std::vector<Node> nodes;

  explicit BoundaryIndex(const std::vector<Vec2>& input) : polygon(input) {
    order.reserve(input.size()); nodes.reserve(2*input.size());
    for(std::size_t i=0;i<input.size();++i) order.push_back(i);
    if(!order.empty()) build(0,order.size());
  }
  std::size_t build(std::size_t begin,std::size_t end) {
    const double inf=std::numeric_limits<double>::infinity();
    Node node{inf,inf,-inf,-inf,begin,end};
    for(std::size_t k=begin;k<end;++k) {
      const std::size_t i=order[k],j=i==0?polygon.size()-1:i-1;
      for(Vec2 p:{polygon[i],polygon[j]}) {
        node.min_x=std::min(node.min_x,p.x); node.max_x=std::max(node.max_x,p.x);
        node.min_y=std::min(node.min_y,p.y); node.max_y=std::max(node.max_y,p.y);
      }
    }
    const std::size_t id=nodes.size(); nodes.push_back(node);
    if(end-begin>8) {
      const bool x_axis=node.max_x-node.min_x>node.max_y-node.min_y;
      const std::size_t mid=begin+(end-begin)/2;
      const auto midpoint=[&](std::size_t i) {
        const std::size_t j=i==0?polygon.size()-1:i-1;
        return x_axis ? polygon[i].x/2+polygon[j].x/2 : polygon[i].y/2+polygon[j].y/2;
      };
      std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,
        [&](std::size_t a,std::size_t b){return midpoint(a)<midpoint(b);});
      const auto left=build(begin,mid),right=build(mid,end);
      nodes[id].left=left; nodes[id].right=right;
    }
    return id;
  }
  bool intersects(std::size_t id,const Box& box,double x0,double y0,double x1,double y1) const {
    const auto& n=nodes[id];
    if(n.max_x<x0 || n.min_x>x1 || n.max_y<y0 || n.min_y>y1) return false;
    if(n.end-n.begin>8)
      return intersects(n.left,box,x0,y0,x1,y1) || intersects(n.right,box,x0,y0,x1,y1);
    for(std::size_t k=n.begin;k<n.end;++k) {
      const std::size_t i=order[k],j=i==0?polygon.size()-1:i-1;
      if(segmentIntersectsBox(polygon[j],polygon[i],box)) return true;
    }
    return false;
  }
  bool inside(std::size_t id,Vec2 p) const {
    const auto& n=nodes[id];
    if(p.y<n.min_y || p.y>n.max_y) return false;
    if(n.end-n.begin>8) return inside(n.left,p)!=inside(n.right,p);
    bool parity=false;
    for(std::size_t k=n.begin;k<n.end;++k) {
      const std::size_t i=order[k],j=i==0?polygon.size()-1:i-1;
      const Vec2 a=polygon[j],b=polygon[i];
      if((a.y>p.y)!=(b.y>p.y) && p.x-a.x<(p.y-a.y)*(b.x-a.x)/(b.y-a.y)) parity=!parity;
    }
    return parity;
  }
  // #ROAD_INTERVAL_QUERY
  // 이동하는 차체 전체의 보수적 AABB와 겹칠 수 있는 경계 선분만 반환한다.
  // 제외 판정에는 반올림 여유를 넣고, 애매한 경우 모두 유지한다.
  // 원래 선분 순서로 정렬하여 뒤의 SAT·구간 차집합 계산 순서를 보존한다.
  void collectBounds(std::size_t id, const Bounds& query, std::vector<std::size_t>& found) const {
    const auto& n=nodes[id];
    const Bounds node_bounds{n.min_x,n.min_y,n.max_x,n.max_y};
    if(node_bounds.separated(query,0)) return;
    if(n.end-n.begin>8) {
      collectBounds(n.left,query,found); collectBounds(n.right,query,found);
      return;
    }
    for(std::size_t k=n.begin;k<n.end;++k) {
      const std::size_t i=order[k], j=i==0?polygon.size()-1:i-1;
      Bounds edge; edge.include(polygon[i]); edge.include(polygon[j]);
      if(!edge.separated(query,0)) found.push_back(i);
    }
  }
  std::vector<std::size_t> queryBounds(const Bounds& query) const {
    std::vector<std::size_t> found;
    if(!nodes.empty()) collectBounds(0,query,found);
    std::sort(found.begin(),found.end());
    return found;
  }

  bool containsBox(const Box& box) const {
    if(polygon.size()<3) return false;
    const double cs=std::abs(std::cos(box.pose.yaw_rad)),sn=std::abs(std::sin(box.pose.yaw_rad));
    const double scale=std::max({1.0,std::abs(box.pose.position.x),std::abs(box.pose.position.y),
                                box.half_length,box.half_width});
    const double padding=1e-8+32*std::numeric_limits<double>::epsilon()*scale;
    const double hx=(box.half_length+1e-9)*cs+(box.half_width+1e-9)*sn+padding;
    const double hy=(box.half_length+1e-9)*sn+(box.half_width+1e-9)*cs+padding;
    const double x0=box.pose.position.x-hx,x1=box.pose.position.x+hx;
    const double y0=box.pose.position.y-hy,y1=box.pose.position.y+hy;
    if(!std::isfinite(x0) || !std::isfinite(x1) || !std::isfinite(y0) || !std::isfinite(y1))
      return polygonContainsBox(polygon,box);
    return !intersects(0,box,x0,y0,x1,y1) && inside(0,box.pose.position);
  }
};

// Encloses all vehicle footprints when rear-axle position interpolates linearly
// and yaw follows the shortest rotation between the endpoint poses.
static Box sweptEgoEnvelope(Pose2 a,Pose2 b,const VehicleShape& shape,double margin) {
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
static bool sweptOverlap(const Box& a0, const Box& a1, const Box& b0, const Box& b1) {
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

static double curvature(Vec2 a, Vec2 b, Vec2 c) {
  const auto ab = sub(b,a), bc = sub(c,b), ac = sub(c,a);
  const double den = norm(ab)*norm(bc)*norm(ac);
  return den > 1e-12 ? 2.0*cross(ab,bc)/den : 0.0;
}
  };

  // #LEGAL_LANE_CORRIDOR
  // route만 사용하면 점선으로 변경 가능한 옆 차선도 경계 밖으로 취급된다.
  // 기존 SharedMap을 읽어 RoutingGraph의 합법 left/right 관계이며 경계 ID도
  // 공유하는 옆 차선을 추가한다. 반대 방향으로만 진입 가능한 옆 차선은
  // 차체의 변경 완료 공간에 포함하되 아래 방향 검사로 역방향 진입을 금지한다. 탐색 폭과 별개로 실제 map lanelet polygon의 합집합을 저장해 차체
  // 전체와 연속 이동 envelope가 그 안에 있는지도 검사한다. 실선·역방향·빈
  // 공간을 직사각형 폭 확장으로 주행 가능하게 만들지 않는다. 계산 지도는
  // Core 생성 시 주입하며 다른 Core/지도 객체를 수정하지 않는다.
  using RoadPoint = boost::geometry::model::d2::point_xy<double>;
  using RoadPolygon = boost::geometry::model::polygon<RoadPoint>;
  using RoadPolygons = boost::geometry::model::multi_polygon<RoadPolygon>;
  using LaneKey = std::pair<lanelet::Id,bool>;
  static LaneKey laneKey(const lanelet::ConstLanelet& lane) { return {lane.id(),lane.inverted()}; }
  struct NativeRoadRegion {
    std::vector<LaneKey> ids;
    RoadPolygons polygons;
    std::shared_ptr<const NativeRoadRegion> recovery_original;
    double initial_outside_area_m2=0;
    bool margin_only_recovery=false;
    RoadPolygons recovery_outside;
    struct IndexedRing {
      const std::vector<Vec2> points;
      Geometry::BoundaryIndex index;
      explicit IndexedRing(std::vector<Vec2> input) : points(std::move(input)),index(points) {}
      IndexedRing(const IndexedRing&)=delete;
      IndexedRing& operator=(const IndexedRing&)=delete;
    };
    struct Component {
      std::shared_ptr<const IndexedRing> outer;
      std::vector<std::shared_ptr<const IndexedRing>> holes;
    };
    std::vector<Component> components;
    struct OneWayBoundary {
      std::shared_ptr<const IndexedRing> line;
      bool positive_allowed;
    };
    std::vector<OneWayBoundary> one_way_boundaries;
    // #LANE_CHANGE_DIRECTION
    // solid_dashed처럼 한쪽 변경만 허용하는 경계는 polygon 합집합만으로
    // 방향을 표현할 수 없다. GlobalPath의 현재 차선 판정과 같은 뒷차축 기준점이
    // 금지 방향으로 경계 선분을 통과하는 이동을 거부한다. 몸체 전체의 도로
    // 포함 여부는 별도의 기존 swept footprint 검사로 계속 확인한다.
    bool allowsMotion(Vec2 first,Vec2 last) const {
      Geometry::Bounds bounds;bounds.include(first);bounds.include(last);
      const auto motion=Geometry::sub(last,first);
      for(const auto& rule:one_way_boundaries) {
        const auto& points=rule.line->points;
        for(const auto i:rule.line->index.queryBounds(bounds)) {
          if(i==0)continue; // 열린 경계선의 끝→시작 가상 선분은 사용하지 않는다.
          const auto a=points[i-1],b=points[i],edge=Geometry::sub(b,a);
          const double epsilon=1e-8*std::max(1.0,Geometry::norm(edge));
          const double side0=Geometry::cross(edge,Geometry::sub(first,a));
          const double side1=Geometry::cross(edge,Geometry::sub(last,a));
          const bool forbidden=rule.positive_allowed?
              side0>=-epsilon && side1<-epsilon:side0<=epsilon && side1>epsilon;
          if(!forbidden)continue;
          const double along0=Geometry::cross(motion,Geometry::sub(a,first));
          const double along1=Geometry::cross(motion,Geometry::sub(b,first));
          const double tolerance=1e-8*std::max(1.0,Geometry::norm(motion));
          if((along0<=tolerance && along1>=-tolerance)||
             (along1<=tolerance && along0>=-tolerance))return false;
        }
      }
      return true;
    }
    // #EXACT_ROAD_INDEX
    // 합집합 계산은 route가 바뀔 때 한 번 한다. 매 후보에서 전체 지도에
    // polygon 연산을 반복하지 않고 동일 외곽선·내부 구멍의 BVH를 조회한다.
    // 네 꼭짓점만 검사하면 놓치는 오목 경계와 차체가 덮는 구멍도 선분과
    // 차체의 닫힌 교차 검사로 거부한다. 경계나 구멍을 간소화하지 않는다.
    void prepare() {
      components.clear();
      const auto ring=[](const RoadPolygon::ring_type& input) {
        std::vector<Vec2> points;
        for(const auto& point:input)points.push_back({point.x(),point.y()});
        return std::make_shared<const IndexedRing>(std::move(points));
      };
      for(const auto& polygon:polygons) {
        Component component;component.outer=ring(polygon.outer());
        for(const auto& hole:polygon.inners())component.holes.push_back(ring(hole));
        components.push_back(std::move(component));
      }
    }
    static RoadPolygon boxPolygon(const Geometry::Box& box) {
      RoadPolygon polygon;
      for(const auto& corner:Geometry::corners(box))polygon.outer().emplace_back(corner.x,corner.y);
      boost::geometry::correct(polygon);return polygon;
    }
    double outsideArea(const Geometry::Box& box) const {
      RoadPolygons outside;
      boost::geometry::difference(boxPolygon(box),polygons,outside);
      return std::abs(boost::geometry::area(outside));
    }
    double reentryOutsideArea(const Geometry::Box& box) const {
      if(recovery_original->containsBox(box))return 0;
      RoadPolygons overlap;
      boost::geometry::intersection(boxPolygon(box),recovery_outside,overlap);
      return std::abs(boost::geometry::area(overlap));
    }
    bool containsBox(const Geometry::Box& box) const {
      Geometry::Bounds bounds;
      for(const auto& corner:Geometry::corners(box))bounds.include(corner);
      for(const auto& component:components) {
        if(!component.outer->index.containsBox(box))continue;
        bool touches_hole=false;
        for(const auto& hole:component.holes) {
          if(hole->points.size()<3)continue;
          if(hole->index.inside(0,box.pose.position)) { touches_hole=true;break; }
          for(const auto i:hole->index.queryBounds(bounds)) {
            const auto j=i==0?hole->points.size()-1:i-1;
            if(Geometry::segmentIntersectsBox(hole->points[j],hole->points[i],box)) {
              touches_hole=true;break;
            }
          }
          if(touches_hole)break;
        }
        if(!touches_hole)return true;
      }
      return false;
    }
  };
  std::shared_ptr<const NativeRoadRegion> native_region_cache_;
  mutable std::shared_ptr<const NativeRoadRegion> active_native_region_;
  // process의 한 계산 동안만 실제 map 영역을 적용한다. 독립 plan() 호출은
  // 기존 ReferencePoint가 지정한 영역을 그대로 사용하고 map 상태가 새지 않는다.
  struct NativeRegionScope {
    std::shared_ptr<const NativeRoadRegion>& target;
    std::shared_ptr<const NativeRoadRegion> previous;
    NativeRegionScope(std::shared_ptr<const NativeRoadRegion>& slot,
                      std::shared_ptr<const NativeRoadRegion> region)
        : target(slot),previous(slot) { target=std::move(region); }
    ~NativeRegionScope() { target=std::move(previous); }
  };
  lanelet::Optional<lanelet::ConstLanelet> legalNeighbor(const lanelet::ConstLanelet& lane,
                                                       bool left) const {
    if(!routing_graph_) return {};
    const auto neighbor=left?routing_graph_->left(lane):routing_graph_->right(lane);
    if(neighbor && (left?lane.leftBound().id()==neighbor->rightBound().id():
                       lane.rightBound().id()==neighbor->leftBound().id())) return neighbor;
    return {};
  }
  // #LANE_CHANGE_COMPLETION
  // 전역 경로는 뒷차축이 옆 lanelet에 들어가는 즉시 현재 차선을 바꿀 수 있다.
  // 차체가 두 차선에 걸쳐 있는 동안 출발 차선을 지우면 합법 변경도 시작점의
  // 도로 검사에서 탈락한다. 출발 쪽에서 현재 차선으로 들어오는 합법 관계도
  // 공유 경계와 진행 방향을 확인해 차체 공간에 포함한다. 단순 adjacent 관계만
  // 으로 확장하지 않으며, 아래 one_way_boundaries는 출발→목표 방향만 허용해
  // 현재 차선에서 출발 차선으로 되돌아가는 금지 방향은 계속 거부한다.
  lanelet::Optional<lanelet::ConstLanelet> corridorNeighbor(const lanelet::ConstLanelet& lane,
                                                          bool left) const {
    const auto outgoing=legalNeighbor(lane,left);
    if(outgoing || !routing_graph_)return outgoing;
    const auto incoming=left?routing_graph_->adjacentLeft(lane):routing_graph_->adjacentRight(lane);
    if(!incoming || !(left?lane.leftBound().id()==incoming->rightBound().id():
                         lane.rightBound().id()==incoming->leftBound().id()))return {};
    const auto target=legalNeighbor(*incoming,!left);
    if(target && laneKey(*target)==laneKey(lane))return incoming;
    return {};
  }
  std::shared_ptr<const NativeRoadRegion> nativeRoadRegion(const Route& route) {
    if(!routing_graph_) return nullptr;
    std::map<LaneKey,lanelet::ConstLanelet> lanes;
    for(const auto& lane:*route) {
      lanes.emplace(laneKey(lane),lane);
      for(bool left:{false,true}) {
        const auto neighbor=corridorNeighbor(lane,left);
        if(neighbor)lanes.emplace(laneKey(*neighbor),*neighbor);
      }
    }
    std::vector<LaneKey> ids;
    for(const auto& item:lanes)ids.push_back(item.first);
    if(native_region_cache_ && native_region_cache_->ids==ids)return native_region_cache_;
    auto region=std::make_shared<NativeRoadRegion>();region->ids=std::move(ids);
    for(const auto& item:lanes)for(bool left:{false,true}) {
      const auto neighbor=legalNeighbor(item.second,left);
      if(!neighbor)continue;
      const auto reverse=legalNeighbor(*neighbor,!left);
      if(reverse && laneKey(*reverse)==item.first)continue;
      const auto bound=left?item.second.leftBound3d():item.second.rightBound3d();
      std::vector<Vec2> points;
      for(const auto& point:bound)points.push_back({point.x(),point.y()});
      region->one_way_boundaries.push_back({
          std::make_shared<const NativeRoadRegion::IndexedRing>(std::move(points)),left});
    }
    for(const auto& item:lanes) {
      RoadPolygon polygon;
      for(const auto& point:item.second.polygon3d())
        polygon.outer().emplace_back(point.x(),point.y());
      boost::geometry::correct(polygon);
      if(region->polygons.empty()) region->polygons.push_back(std::move(polygon));
      else {
        RoadPolygons merged;
        boost::geometry::union_(region->polygons,polygon,merged);
        region->polygons=std::move(merged);
      }
    }
    region->prepare();
    native_region_cache_=region;
    return region;
  }

  // #REAR_FOOTPRINT_ROAD_HISTORY
  // 전역 route의 predecessor 하나가 10cm처럼 짧으면, 다음 lanelet 진입 시
  // 뒷범퍼가 놓인 실제 도로까지 계산 영역에서 사라질 수 있다. map 좌표의
  // route 앞에 합법적인 종방향 previous 관계만 따라 차체 대각선 길이만큼
  // 과거 도로를 보존한다. 짧은 lanelet 여러 개도 누적 길이로 처리한다.
  // 방향을 포함한 방문 집합으로 순환을 막으며 predecessor가 여러 개이면
  // 선택 근거가 없으므로 추가하지 않는다. 옆 차선·반대 차선·없는 도로를
  // 확장하지 않고, 생성 경로의 기준점과 전진·차체·충돌 검사는 유지한다.
  Route rearFootprintRoute(const Route& route) const {
    if(!routing_graph_) return route;
    lanelet::ConstLanelets lanes(route->begin(),route->end());
    std::set<LaneKey> visited;
    for(const auto& lane:lanes) visited.insert(laneKey(lane));
    const double body_span=std::hypot(p_.vehicle.front_m+p_.vehicle.rear_m+2*p_.road_margin_m,
                                      p_.vehicle.width_m+2*p_.road_margin_m);
    double covered=0;
    while(covered<body_span) {
      const auto incoming=routing_graph_->previous(lanes.front(),false);
      if(incoming.size()!=1 || !visited.insert(laneKey(incoming.front())).second) break;
      const auto line=incoming.front().centerline3d();
      for(std::size_t i=1;i<line.size();++i)
        covered+=std::hypot(line[i].x()-line[i-1].x(),line[i].y()-line[i-1].y());
      lanes.insert(lanes.begin(),incoming.front());
    }
    return lanelet::routing::LaneletPath(std::move(lanes));
  }

 public:
  explicit CorePathPlanner(PathParameters parameters = {}) : p_(parameters) { validateParameters(); }
  CorePathPlanner(PathParameters parameters, SharedMap& shared) : CorePathPlanner(parameters) {
    const auto rules=lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany,lanelet::Participants::Vehicle);
    routing_graph_=lanelet::routing::RoutingGraph::build(*shared.map,*rules);
  }


  // Planned v/a are ground speed and tangential acceleration along the actual
  // rear-axle path. Reference progress is only for correspondence with the map.
  // Every result spans the available prediction interval; blocked motion is
  // represented by a physically reachable stop/wait, never a truncated prefix.
  std::vector<PathPoint> plan(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                                const Predictions& predictions, const PlanningConstraints& requested_limits,
                                const std::vector<PathPoint>& previous = {}) const {
    auto limits=requested_limits;
    requireSceneDomain(ego,reference_map,predictions);
    requirePlanningDomain(ego,limits);
    // This independent calculation-window budget applies only to integrated
    // search. Do not invalidate finer legacy supplied-schedule configurations.
    if(p_.trajectory_horizon_m/p_.output_step_m>600 || p_.trajectory_horizon_m/p_.dp_step_m>100)
      throw std::invalid_argument("unbounded integrated planner discretization");
    const double offset=ego.stamp_sec-predictions.reference_stamp_sec;
    const double duration=predictions.step_sec*kPredictionSteps-offset;
    if (offset<0 || duration<=0 ||
        ego.acceleration_mps2>p_.max_acceleration_mps2 ||
        ego.acceleration_mps2<-p_.max_deceleration_mps2) return {};
    // 절대 차량 곡률 한도는 유지한다. 실측 횡가속도 초과는 아래의
    // 고정된 가장 빠른 감속 포락선 안에서만 복귀하도록 시간 계획한다.
    if(std::max(0.0,std::abs(ego.curvature_inv_m)-1e-8)>
       p_.max_curvature_inv_m+1e-6+1e-12)return {};
    limits=lateralRecoveryLimits(ego,limits);
    const Reference reference(reference_map);
    const auto origin=reference.project(ego.pose.position,0,reference.length());
    // #PEDESTRIAN_APPROACH_CEILING
    // 정지 보행자를 오차 없는 정적 물체로만 취급하면 가까운 거리에서
    // 움직인 뒤에야 경로 계산과 제동 전달이 시작된다. 같은 map 관측을
    // 계획 시각으로 보간하고 전방 도로 주변의 보행자 접근을 미리 감속한다.
    // 실제 bbox·관측 속도·예측 중심선은 바꾸지 않는다. preview 안의 도로
    // 폭에 객체 반경·차체 반폭·기존 장애물 여유를 더해 적용 대상을 고른다.
    // 해당할 때만 요청 상한과 caution 속도의 최솟값을 사용한다. 실측 v/a는
    // 보존하고 기존 ceiling recovery·jerk·가감속 및 전체 충돌/도로 검사로
    // 낮아진 상한에 도달한다. 정지를 새로 강제하거나 0속도 요청을 높이지 않는다.
    // 두 설정이 0이면 기존 계산이다. 거리와 속도는 센서 탐지 범위·계획/전달
    // 지연·제동 응답에 맞춰 검증할 시험 계수이며 안전 보증값이 아니다.
    // 특정 객체 ID·지도 좌표·GT 여부에 의존하지 않는다. LiDAR도 같은 label/절대
    // 속도·bbox·연속 추적 ID 계약이 필요하며 unknown을 보행자로 추정하지 않는다.
    // 같은 관측 객체의 bbox가 자세에 따라 조금 변해도 시작한 감속은 유지한다.
    // 현재 관측에 남고 전방 거리 안에 있는 ID만 보존하며, 뒤범퍼 통과·거리
    // 이탈·관측 목록에서 제외되면 해제한다. 위치/운동을 추정하거나 누락 객체를
    // 만들어 넣지 않는다. 충돌 검사에는 매 호출의 원래 bbox·예측을 사용한다.
    std::set<std::uint32_t> active_pedestrian_caution;
    if(p_.pedestrian_caution_speed_mps>0 && p_.pedestrian_caution_distance_m>0) {
      for(std::size_t index=0;index<predictions.current.size();++index) {
        if(predictions.current[index].kind!=ObjectKind::Pedestrian) continue;
        const auto object=samplePrediction(predictions,index,offset);
        const double radius=std::hypot(object.length_m,object.width_m)/2+object.radius_error_m;
        const auto delta=Geometry::sub(object.pose.position,ego.pose.position);
        if(Geometry::norm(delta)>p_.pedestrian_caution_distance_m+radius) continue;
        const auto spot=reference.project(object.pose.position,origin.s,
            std::min(reference.length(),origin.s+p_.pedestrian_caution_distance_m));
        const auto road=reference.at(spot.s);
        const double padding=radius+p_.vehicle.width_m/2+p_.obstacle_margin_m;
        if(Geometry::dot(delta,Geometry::unit(ego.pose.yaw_rad))<-p_.vehicle.rear_m-radius) continue;
        if(!pedestrian_caution_ids_.count(object.id) &&
           (spot.distance>std::max(std::abs(road.lower),std::abs(road.upper))+padding ||
            spot.l<road.lower-padding || spot.l>road.upper+padding)) continue;
        active_pedestrian_caution.insert(object.id);
        limits.maximum_speed_mps=std::min(limits.maximum_speed_mps,p_.pedestrian_caution_speed_mps);
        // #PEDESTRIAN_NEAR_APPROACH
        // 전방 map 도로 주변 보행자가 기존 공간 창(horizon_m) 안에 들어오면
        // 가까운 접근의 비용 목표만 낮춰 돌발 횡단 전에 감속 여유를 확보한다.
        // 상한·실측 초기 v/a·정지 요청·원래 충돌/도로/동역학 검사는 유지한다.
        // 목표는 하한이 아니므로 필요한 정지는 계속 가능하고, 안전한 출발도
        // 막지 않는다. GT ID나 위치를 쓰지 않아 같은 계약의 LiDAR 추적에도 적용된다.
        if(p_.pedestrian_near_preferred_speed_mps>0 && spot.s-origin.s<=p_.horizon_m)
          limits.preferred_speed_mps=std::min(limits.preferred_speed_mps,
                                             p_.pedestrian_near_preferred_speed_mps);
      }
    }
    pedestrian_caution_ids_=std::move(active_pedestrian_caution);
    // #SPEED_DEPENDENT_CALCULATION_WINDOW
    // 일반 구간에서는 검증한 기존 네 기하 창(기본 100/75/50/37.5m)을 유지한다.
    // 고주로 목표 또는 이미 빠른 실측 상태일 때만 확장한 공간 계산 범위를 쓴다.
    // 같은 0.25m 표본과 전체 도로·충돌·시간 검사를 유지하며 일반 곡선/정적
    // 회피의 유효 후보를 새로운 고속 계산 범위의 비율로 바꾸지 않는다.
    const double planning_horizon_m=p_.normal_speed_limit_mps>0 &&
        limits.maximum_speed_mps<=p_.normal_speed_limit_mps && ego.speed_mps<=p_.normal_speed_limit_mps ?
        std::min(p_.trajectory_horizon_m,4*p_.horizon_m) : p_.trajectory_horizon_m;
    const auto initial_road=reference.at(origin.s);
    const bool nominal_heading=std::abs(Geometry::wrap(ego.pose.yaw_rad-initial_road.yaw))<=
                               p_.max_initial_heading_error_rad;
    // #NOISY_REFERENCE_ADMISSION
    // 조밀한 map knot의 순간 접선과 공간적으로 안정된 도로 방향은 다를 수
    // 있다. 순간 곡률이 차량 한도를 넘는지와 무관하게 기존 C2 후보와 같은 dp_step_m의
    // 공간 샘플로 구한 도로 방향에도 동일한 초기 방향 오차 한도를 적용한다.
    // 기존 Frenet 생성기의 방향 검사·원래 도로 polygon·투영 모호성 검사는 유지한다.
    // 이 조건은 C2 후보를 검사할 기회만 주며 차체·곡률·속도 검증을 대신하지 않는다.
    const bool recovery_heading=!nominal_heading &&
        std::abs(Geometry::wrap(ego.pose.yaw_rad-cartesianEndpoint(reference,origin.s).yaw))<=
            p_.max_initial_heading_error_rad;
    // Admission belongs to the scene, not to any single geometric proposal.
    // In particular, a hold-offset candidate must not bypass these guards.
    if(origin.distance>p_.max_initial_distance_m ||
       (!nominal_heading && !recovery_heading) ||
       reference.ambiguous(ego.pose.position,origin,ego.pose.yaw_rad)) return {};
    const auto original_geometry=roadGeometry(reference_map,reference);
    const NativeRegionScope reentry_scope(active_native_region_,roadReentryRegion(ego,original_geometry));
    const auto geometry=roadGeometry(reference_map,reference);
    const auto& boundary=geometry->boundary;
    const std::vector<ProgressTime> unused;
    const Context c{reference,predictions,unused,boundary,0,offset,false,&geometry->index,geometry->native_region.get()};
    // #LONG_CENTER_GEOMETRY
    // map 중심선의 작은 knot 흔들림 때문에 긴 Frenet 후보가 탈락하면,
    // 기존 탐색은 곧바로 25m 창을 골라 그 끝에 제동 가능한 속도를 계획했다.
    // 직선에서도 이 가상의 공간 끝이 다가왔다 사라져 가감속이 반복된다.
    // 기존 Cartesian C2 계산을 horizon_m 이하의 구간으로 이어 긴 공간 창을
    // 만든다. 한 다항식으로 100m까지 횡 복귀를 미루지 않고 매 구간 끝에서
    // 목표 중앙·방향·곡률을 맞춰 기존 차선 변경 거리를 유지한다.
    // map 입력·뒷차축 초기 자세/곡률·끝 차선 중앙은 유지하고, 가장 긴 창부터
    // 원래 차체/도로/횡기울기/곡률 및 전체 시간 계획 검사를 통과해야 선택한다.
    // 실제 정적 장애물이 이 중앙 형상을 막으면 먼저 택하지 않고 기존 DP/QP
    // 회피 탐색을 유지한다. 동적 객체·정지선·속도·가감속·jerk 한도도 그대로다.
    // 실패한 후보를 발행하거나 ROI/허용 도로를 넓히지 않으며, 네 창은 기존
    // trajectory_horizon_m 안에 한정한다. 이후 ROI나 도로 계약이 바뀌어도
    // 동일한 전체 차체 경계 검사를 생략하면 안 된다.
    // #SPEED_FEASIBLE_LANE_CHANGE_LENGTH
    // 조기 차선 변경의 목표가 현재 차선 옆에 있으면 25m C2 연결은 고속에서
    // 횡가속도 한도를 만족하지 못할 수 있다. 기존 네 길이의 짧은 연결을 먼저
    // 검사한 뒤, 같은 C2 계산기의 연결 길이만 기존 창 상한인 2*horizon_m으로
    // 늘려 다시 검사한다. 초기 뒷차축 상태·목표 중앙·전체 예측 시간과 모든
    // 안전 검사를 유지한다. 유효한 짧은 연결을 미루거나 출력 속도를 가산하지 않는다.
    bool static_center_blocked=false;
    double static_clearance_progress=0;
    // #CURVE_SEGMENT_CONTINUITY
    // 긴 C2 조각이 S자 도로를 벗어나면 짧은 공간 창만 남아 그 끝을 위한
    // 감속이 반복된다. 기존 두 길이를 먼저 유지하고, 기하가 성립하지 않은
    // 창만 같은 C2 연결기의 반·사분 길이로 이어 전방 도로를 표현한다.
    // 이미 기하가 있는 창의 속도/충돌 실패를 중복 계산하지 않는다. 뒷차축
    // 초기 상태·전체 계획 길이·600표본 한도와 원래 차체/도로/시간 검사를
    // 유지한다. 새 형상도 실제 객체 예측과 가감속·jerk 검사 후에만 선택한다.
    std::set<double> long_geometry_ready;
    const auto center_candidate=[&](double horizon,double segment_extent) -> std::vector<PathPoint> {
      const int segments=static_cast<int>(std::ceil(horizon/segment_extent));
      const std::vector<std::pair<double,double>> goals(segments,{horizon/segments,0});
      auto smooth=cartesianPieces(c,ego,goals);
      // 새 곡선이 시간 계획에서 실패해도 기존 짧은 창과 DP 후보를 계속
      // 검사해야 한다. 탐색 순서/정적 차단 표시는 원래 기하에서만 갱신한다.
      const bool original_geometry=!smooth.empty();
      if(!original_geometry) smooth=cartesianPieces(c,ego,goals,true);
      if(smooth.empty()) return {};
      if(original_geometry && segment_extent>=p_.horizon_m) long_geometry_ready.insert(horizon);
      const ArcPath arc(smooth);
      double alternate_clearance_progress=0;
      if(std::isfinite(staticStopArc(c,arc,original_geometry ? &static_clearance_progress : &alternate_clearance_progress))) {
        if(original_geometry) static_center_blocked=true;
        return {};
      }
      const double stop_arc=stopArc(ego,reference,arc,limits);
      if(stop_arc<0 && limits.release_time_sec>0) return {};
      auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map);
      if(!path.empty() && path.back().speed_mps>0 &&
         checkPlannedPath(ego,reference_map,predictions,limits,path)) return path;
      return {};
    };
    for(double segment_extent:{p_.horizon_m,2*p_.horizon_m,p_.horizon_m/2,p_.horizon_m/4})
    for(double horizon:{planning_horizon_m,.75*planning_horizon_m,
                        .5*planning_horizon_m,.375*planning_horizon_m}) {
      if(static_center_blocked) break;
      if(segment_extent<p_.horizon_m && long_geometry_ready.count(horizon)) continue;
      if(horizon<=p_.horizon_m || horizon/p_.horizon_m>500) continue;
      auto path=center_candidate(horizon,segment_extent);
      if(path.empty()) continue;
      // #VERIFIED_BRAKING_CANDIDATE_COMPARISON
      // map 도로의 같은 전체 계산 범위에서도 짧은 C2 연결은 불필요한 곡률과
      // 급감속을 만들 수 있다. 첫 속도 계획 시간 격자 안에 제동이 있으면 기존 두 배 길이
      // 연결 한 개만 같은 초기 상태로 비교한다. 먼 미래의 말단 감속만 있을 때는
      // 비교를 생략해 정상 순항의 계산량을 유지한다. 새 공간 창이나 다른 차선을
      // 추가하지 않고, 정적 차단·전체 차체·도로·동적 충돌·정지선·속도·가감속·
      // jerk 검사를 통과한 후보 중 기존 시간 궤적 비용이 낮은 결과를 선택한다.
      // 비교 후보가 실패하거나 비용이 같으면 원래 검증된 후보를 유지한다.
      // ROI/검출 입력이 바뀌어도 같은 map 객체·도로 계약과 검사를 유지해야 한다.
      if(segment_extent==p_.horizon_m && std::any_of(path.begin(),path.end(),
          [&](const PathPoint& point){
            return point.time_sec<=p_.speed_time_step_sec && point.acceleration_mps2<0;
          })) {
        auto alternative=center_candidate(horizon,2*p_.horizon_m);
        const double ceiling=std::min({p_.max_speed_mps,limits.maximum_speed_mps,limits.preferred_speed_mps});
        if(!alternative.empty() && trajectoryCost(c,alternative,ceiling)<trajectoryCost(c,path,ceiling))
          path=std::move(alternative);
      }
      return path;
    }
    const double cap=std::min(p_.max_speed_mps,limits.maximum_speed_mps);
    // #STATIC_CREEP_SEARCH_COMPLETENESS
    // 현재 정지한 차량에게 첫 시간 격자 전체의 대기 후 작은 전진을 예약하면
    // 재계획 때마다 대기가 연장될 수 있다. 그 후보는 안전한 정지 선택지로
    // 보존하고 다른 기존 창에서 즉시 전진 가능한 검증된 경로를 더 찾는다.
    // 이미 이동 중인 차량의 정상 제동·정지 계획은 기존처럼 선택한다.
    // cap=0 요청과 도로·충돌·동역학 검사를 유지하며, 다른 해가 없으면
    // 보존한 대기를 반환한다. 가속 offset이나 대기/정지 검사 해제는 없다.
    const auto has_initial_progress=[&](const std::vector<PathPoint>& path) {
      if(path.empty() || path.back().progress_m<=1e-6) return false;
      return ego.speed_mps>0 || std::any_of(path.begin(),path.end(),[&](const PathPoint& q) {
        return q.time_sec<=p_.speed_time_step_sec && q.progress_m>1e-6;
      });
    };
    // #STATIC_AVOIDANCE_ANCHOR
    // 짧은 중앙 창이 장애물 전에 끝나면 정적 충돌이 없어 보이지만, 그 창의
    // 제동 경로가 회피 탐색을 선점한다. 긴 중앙 형상을 막은 가장 가까운 정적
    // 객체의 팽창 OBB 뒤끝을 map 참조선에 투영하고, 뒷범퍼가 벗어나는 지점에
    // 회피 끝점을 고정한다. 객체 ID나 특정 지도 좌표에 의존하지 않는다.
    // 처음부터 옆으로 이동하면 아직 열리지 않은 차선을 침범할 수 있으므로,
    // 기존 C2 중앙 접근 뒤에 C2 횡이동을 잇는다. 접속 위치·방향·곡률은 연속이며
    // 기존 horizon_m의 유한 공간 격자와 DP 횡 격자만 검사한다.
    // 실제 도로 합집합·합법 변경 방향·차체 전체·정적/동적 예측·정지선·곡률·
    // 횡가속도·가감속·jerk 검사에 모두 통과한 후보만 선택한다. 해가 없으면
    // 기존 DP/QP와 제동 계획을 계속하며, 장애물 여유나 허용 도로를 넓히지 않는다.
    if(static_center_blocked && cap>0 && static_clearance_progress>=p_.min_horizon_m &&
       static_clearance_progress<=planning_horizon_m &&
       origin.s+static_clearance_progress<=reference.length()) {
      const auto end=cartesianEndpoint(reference,origin.s+static_clearance_progress);
      const auto offsets=roadGoalOffsets(c,end);
      // #STATIC_AVOIDANCE_CONTINUATION
      // 회피 끝을 객체 뒤끝으로 한정하면 공간 창 끝의 정지 가능성 제약이
      // 통과 가능한 도로에서도 감속을 강제한다. 같은 C2 연결기를 사용해
      // 옆 차선 회피 뒤 중앙 복귀와 전방 도로를 기존 계획 길이 안에서 잇는다.
      // 회피/복귀 구간은 정적 충돌이 없어야 하며, 그 뒤의 다른 장애물은
      // planSpeed가 기존 정지 거리·동적 예측·최종 경로 검사로 처리한다.
      // 처음 다섯 이분 길이 사이도 기존 1/8 공간 격자로 검사해 합법적
      // 차선이 열리는 짧은 구간을 빠뜨리지 않는다. 새 한도/가속 보상은 없다.
      std::vector<double> spans;
      for(int attempt=0;attempt<5;++attempt) {
        const double span=std::min(p_.horizon_m,static_clearance_progress)/std::pow(2.0,attempt);
        if(span<p_.min_horizon_m) break;
        spans.push_back(span);
        if(attempt<4) for(int q=1;q<4;++q) {
          const double intermediate=span*(1-q/8.0);
          if(intermediate>=p_.min_horizon_m) spans.push_back(intermediate);
        }
      }
      const double extent=std::min(planning_horizon_m,reference.length()-origin.s-
          std::hypot(p_.vehicle.front_m,p_.vehicle.width_m/2)-p_.road_margin_m-1e-6);
      for(double span:spans) {
        double approach=static_clearance_progress-span;
        if(approach<p_.min_horizon_m) approach=0;
        std::vector<std::pair<double,double>> pieces;
        if(approach>0) {
          const int count=static_cast<int>(std::ceil(approach/p_.horizon_m));
          pieces.assign(count,{approach/count,0});
        }
        for(double lateral:offsets) {
          auto goals=pieces;
          goals.push_back({static_clearance_progress-approach,lateral});
          double covered=static_clearance_progress,completed=covered;
          if(extent-covered>=p_.min_horizon_m) {
            const double returning=std::min(span,extent-covered);
            goals.push_back({returning,0});covered+=returning;completed=covered;
            while(extent-covered>=p_.min_horizon_m) {
              const double next=std::min(p_.horizon_m,extent-covered);
              goals.push_back({next,0});covered+=next;
            }
          }
          const auto shape=cartesianPieces(c,ego,goals);
          if(shape.empty()) continue;
          const ArcPath arc(shape);
          const auto end=std::find_if(shape.begin(),shape.end(),[&](const PathPoint& q) {
            return q.progress_m>completed+1e-6;
          });
          const std::vector<PathPoint> crossing_shape(shape.begin(),end);
          const ArcPath crossing(crossing_shape);
          if(std::isfinite(staticStopArc(c,crossing))) continue;
          const double stop_arc=stopArc(ego,reference,arc,limits);
          if(stop_arc<0 && limits.release_time_sec>0) continue;
          auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map);
          if(!path.empty() && path.back().speed_mps>0 && has_initial_progress(path) &&
             checkPlannedPath(ego,reference_map,predictions,limits,path)) return path;
        }
      }
    }
    std::vector<std::vector<PathPoint>> proposals;
    proposals.push_back(generatePath(ego,reference_map,predictions,
                       {{0,0},{planning_horizon_m,duration}},previous,true,planning_horizon_m));
    if(std::abs(origin.l)>1e-6) {
      std::vector<PathPoint> hold;
      const double extent=std::min(planning_horizon_m,reference.length()-origin.s-
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
    if(!predictions.current.empty()) {
      // #REACHABLE_AVOIDANCE_TIMING
      // 고속 가정 두 개만 검사하면 회피 곡선의 횡가속도 검사에서 전부 탈락해
      // 실제 저속으로 가능한 DP/QP 경로도 누락된다. 원래 두 후보를 우선하고,
      // 설정된 최대 곡률을 주행할 수 있는 속도의 후보 하나를 추가한다.
      // 이 시각표는 형상 탐색용이며 출력 속도에는 사용하지 않는다. 실제 속도는
      // 측정 초기 속도·가속도에서 다시 계산하고 전체 시간별 충돌 검사를 통과해야 한다.
      std::vector<double> guesses{cap,std::max(ego.speed_mps,cap/2),
          std::min(cap,std::sqrt(p_.max_lateral_accel_mps2/p_.max_curvature_inv_m))};
      guesses.erase(std::unique(guesses.begin(),guesses.end()),guesses.end());
      // 5초 동안의 최대 전진 거리는 v0*T + Amax*T²/2 이하다. 도로 전체와
      // 객체는 유지하고 형상 후보의 DP/QP 창만 그 상한 + 기존 dp 한 칸으로
      // 제한한다. 실제 도달 가능한 경로를 버리지 않고 불필요한 먼 격자를 줄인다.
      const double proposal_extent=std::min(planning_horizon_m,
          ego.speed_mps*duration+p_.max_acceleration_mps2*duration*duration/2+p_.dp_step_m);
      // #STATIC_PROPOSAL_REUSE
      // 모든 예측 시점에서 실제 box 형상·위치·방향·불확실성이 정확히 같을 때만
      // 동일 공간 창의 중복 시간 가정을 생략한다. 움직이는 객체나 오차 반경의
      // 변화가 하나라도 있으면 기존 세 가정을 모두 검사한다. 이 후보의 가정
      // 시간은 matchInitialCurve에서 버리고 최종 시간 계획·충돌 검사를 다시 하므로
      // 정적인 동일 형상을 두 번 계산하지 않아도 선택 결과는 같다.
      bool constant_boxes=true;
      for(std::size_t j=0;j<predictions.current.size();++j) {
        const auto& original=predictions.current[j];
        if(original.velocity_mps.x!=0 || original.velocity_mps.y!=0)constant_boxes=false;
        for(const auto& slice:predictions.future) {
          const auto& sample=slice[j];
          if(sample.pose.position.x!=original.pose.position.x ||
             sample.pose.position.y!=original.pose.position.y ||
             sample.pose.yaw_rad!=original.pose.yaw_rad ||
             sample.length_m!=original.length_m || sample.width_m!=original.width_m ||
             sample.radius_error_m!=original.radius_error_m ||
             sample.interpolation_acceleration_mps2!=original.interpolation_acceleration_mps2 ||
             sample.velocity_mps.x!=0 || sample.velocity_mps.y!=0)constant_boxes=false;
        }
      }
      std::vector<double> tried_extents;
      for(double speed : guesses) {
        if(speed<=0) continue;
        const std::vector<ProgressTime> timing{{0,0},{proposal_extent,proposal_extent/speed}};
        const double extent=std::min(proposal_extent,progressAt(timing,duration));
        if(constant_boxes && std::find(tried_extents.begin(),tried_extents.end(),extent)!=tried_extents.end())continue;
        tried_extents.push_back(extent);
        proposals.push_back(generatePath(ego,reference_map,predictions,
            timing,previous,false,proposal_extent,true));
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
      auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map);
      if(path.empty() || !checkPlannedPath(ego,reference_map,predictions,limits,path)) continue;
      // #VERIFIED_CENTER_PRIORITY
      // 중앙 복귀가 실제 시간 계획과 모든 안전 검사를 통과해 계속 전진할 수
      // 있으면, 더 빠르다는 이유만으로 이전 횡 offset의 직진 후보를 고르지 않는다.
      // 명목 경로가 장애물 때문에 예측 구간 끝에 정지해야 하는 경우는 기존
      // 회피 후보와 비용 비교를 계속한다. 이 우선순위는 빈 경로나 미검증 형상을
      // 발행하지 않으며 도로·방향·충돌·곡률·속도·가감속·jerk 검사를 모두 유지한다.
      if(i==0 && path.back().speed_mps>0) return path;
      const double cost=trajectoryCost(c,path,std::min(cap,limits.preferred_speed_mps));
      if(cost<best) { best=cost; result=std::move(path); }
    }
    // #RECEDING_GEOMETRIC_WINDOW
    // 먼 구간의 형상이 현재의 DP/QP 창 전체를 막아도, 지금 실행할 수 있는
    // 가까운 경로가 있는지 기존 계산기로 확인한다. 가짜 객체를 넣지 않는다.
    // 전체 도로 경계와 실제 예측을 유지하며, 속도 계산은 남은 예측 시간 전체를
    // 다룬다. 창 끝에서 제동 가능한지와 곡률·충돌·가감속·jerk를 재검증한다.
    // #STATIONARY_WINDOW_SEARCH
    // 정지한 차량 앞에 장애물이 있으면 첫 창이 제자리 대기만 반환할 수 있다.
    // 안전한 대기도 다음 공간 창·횡 후보가 불가능하다는 증거는 아니다. 대기를
    // 보존한 채 기존의 유한한 창을 계속 검사하고, 실제 전진과 모든 시간별
    // 안전 검사가 성립한 후보가 있을 때만 바꾼다. 목표 속도에 하한을 더하거나
    // 정지 요청을 무시하지 않으며 maximum_speed_mps=0이면 기존 대기를 유지한다.
    if(result.empty() || (cap>0 && !has_initial_progress(result))) {
      std::vector<double> windows;
      const double base=std::min(p_.horizon_m,planning_horizon_m/2);
      // #MERGE_WINDOW_CONTINUITY
      // 주도로 진입 실측은 25m 창도 유효하지만 그 끝 때문에 불필요하게 감속했다.
      // 뒤쪽 실패 복구에서만 검사하던 중간 C2 길이를 같은 창 탐색 앞으로 옮긴다.
      // 기존 25~37.5m의 1.25m 격자를 유지해 50m까지 먼저 검사하고, 원래 짧은
      // 창도 그대로 남긴다. 40m 후보가 통과하는데 25m 성공 직후 탐색을 끝내는
      // 누락을 없앤다. 측정 초기 상태·원래 도로·차체·충돌·정지선·곡률·횡가속도·
      // 가감속·jerk 검사를 모두 통과한 경로만 선택하며 출력 속도를 보정하지 않는다.
      const double upper=std::min(2*base,planning_horizon_m);
      for(int attempt=0;attempt<20 && upper>base;++attempt)
        windows.push_back(upper-(upper-base)*attempt/20);
      const std::size_t coarse_begin=windows.size();
      double horizon=base;
      for(int attempt=0;attempt<5 && horizon>=p_.min_horizon_m;++attempt,horizon*=.5)
        windows.push_back(horizon);
      // #WINDOW_SEARCH_COMPLETENESS
      // 이분 간격 사이에서도 끝 자세·곡률과 제동 거리가 함께 성립할 수 있다.
      // 원래 다섯 창을 우선하고, 모두 실패하면 인접 창 사이의 사분점만 추가한다.
      // 최대 17개로 계산량을 제한하며 도로·충돌·곡률·기울기·속도·jerk 검사와
      // 초기 상태는 바꾸지 않는다. 특정 실패가 단조롭다고 가정해 창을 버리지 않는다.
      const std::size_t coarse_count=windows.size();
      for(std::size_t i=coarse_begin+1;i<coarse_count;++i)
        for(int quarter=1;quarter<4;++quarter)
          windows.push_back(windows[i-1]+(windows[i]-windows[i-1])*quarter/4.0);
      // 중앙 경로가 실제 정적 객체에 막혔다면 기존의 짧은 창부터 검사한다.
      // 같은 제동 후보를 20번 재계산하기 전에 가까운 회피/접근을 확인한다.
      // 중간 길이 격자도 뒤에 모두 남겨 계산 누락이나 안전 한도 변경은 없다.
      // #ROAD_REENTRY_WINDOW
      // 경계를 걸친 경우는 먼 순항 창보다 차체 길이 근처의 기존 복귀 창을
      // 먼저 평가한다. 같은 창·C2·속도 탐색과 최종 검사를 모두 보존하며,
      // 첫 창이 실패하면 남은 창도 검사한다. 정상 도로 주행 순서는 유지한다.
      if(c.native_region && c.native_region->recovery_original) {
        const double body_span=std::hypot(p_.vehicle.front_m+p_.vehicle.rear_m,
                                          p_.vehicle.width_m);
        std::stable_sort(windows.begin(),windows.end(),[&](double a,double b) {
          return std::abs(a-body_span)<std::abs(b-body_span);
        });
      } else if(static_center_blocked)
        std::rotate(windows.begin(),windows.begin()+coarse_begin,windows.end());
      for(double horizon:windows) {
        // #BOUNDED_WINDOW_GEOMETRY_ORDER
        // 이미 존재하는 C2 중앙 후보를 먼저 검사한다. 좁은 창에서 Frenet
        // DP/QP가 반복 실패한 뒤 같은 C2로 돌아오던 수십 회 계산을 줄인다.
        // C2 형상이 없으면 원래 DP/QP·횡 후보를 모두 유지한다. 형상이 있어도
        // 동일한 전체 시간 계획·도로/충돌/동역학 최종 검사를 통과해야 반환한다.
        // 계산 지연을 숨기기 위해 경로 시각·제어 입력 만료 한도를 바꾸지 않는다.
        auto matched=cartesianRecovery(c,ego,horizon,0);
        if(matched.empty()) {
          const auto shape=generatePath(ego,reference_map,predictions,
              {{0,0},{horizon,duration}},previous,true,horizon);
          matched=shape.empty() ? std::vector<PathPoint>{} : matchInitialCurve(c,ego,shape);
        }
        if(matched.empty() && origin.s+horizon<=reference.length()) {
          const auto end=cartesianEndpoint(reference,origin.s+horizon);
          const double lower=end.lower+p_.vehicle.width_m/2+p_.road_margin_m;
          const double upper=end.upper-p_.vehicle.width_m/2-p_.road_margin_m;
          if(lower<=upper) {
            // #MINIMUM_FOOTPRINT_OFFSET: 기존 DP의 차체 전체 허용 interval로
            // 필요한 최소 횡 보정 끝점을 먼저 시도한다. 이전 인코스 offset을
            // 유지하는 후보보다 중앙에 가까우며 전체 경계·곡률 검사는 그대로다.
            for(const auto& interval:roadLateralIntervals(c,end,end.yaw,lower,upper)) {
              const double centered=Geometry::clamp(0.0,interval.first,interval.second);
              matched=cartesianRecovery(c,ego,horizon,centered);
              if(matched.empty()) {
                const double reachable=(centered+Geometry::clamp(origin.l,interval.first,interval.second))/2;
                matched=cartesianRecovery(c,ego,horizon,reachable);
              }
              if(!matched.empty()) break;
            }
          }
        }
        if(matched.empty()) matched=cartesianRecovery(c,ego,horizon,origin.l);
        if(matched.empty()) continue;
        const ArcPath arc(matched);
        const double stop_arc=stopArc(ego,reference,arc,limits);
        if(stop_arc<0 && limits.release_time_sec>0) continue;
        auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map);
        // #SINGLE_CENTER_WINDOW_SEARCH
        // 이 창은 위에서 중앙 C2 형상을 먼저 검사했다. 형상이 없었으면
        // 같은 입력의 중앙 C2 재호출도 비고, 형상이 있었다면 같은 시간 계획을
        // 반복해도 결과가 같다. 이전 순서에서 필요했던 후행 재시도를 제거한다.
        // 다른 창·DP 횡 후보·전체 도로/충돌/동역학 최종 검사는 그대로 유지한다.
        if(path.empty() || !checkPlannedPath(ego,reference_map,predictions,limits,path)) continue;
        if(cap==0 || has_initial_progress(path)) {
          result=std::move(path);
          break;
        }
        if(result.empty()) result=std::move(path);
      }
      // #DP_GOAL_RECOVERY
      // 중앙·최소 차체 보정·이전 offset 후보가 모든 창에서 실패했을 때만,
      // 기존 DP의 lateral_step_m 격자 끝점도 map XY의 C2 후보로 검사한다.
      // 곡선 중간의 앞 범퍼 여유 때문에 끝점에서 0 offset이 가능해도 경로
      // 전체에는 작은 횡 보정이 필요할 수 있다. 허용 끝점은 원래 차체 전체
      // interval 안으로 제한하며, 창별 격자 수는 기존 max_lateral_samples다.
      // 절댓값이 작은 offset부터 시도하고 전체 시간 계획·안전 검사를 통과해야
      // 반환한다. 도로 polygon·ROI·차체 기준점과 안전 계수는 변경하지 않는다.
      if((result.empty() || (cap>0 && !has_initial_progress(result))) && roadClear(c,ego.pose,ego.pose)) {
        for(double horizon:windows) {
          if(origin.s+horizon>reference.length()) continue;
          const auto end=cartesianEndpoint(reference,origin.s+horizon);
          for(double l:roadGoalOffsets(c,end)) {
            const auto matched=cartesianRecovery(c,ego,horizon,l);
            if(matched.empty()) continue;
            const ArcPath arc(matched);
            const double stop_arc=stopArc(ego,reference,arc,limits);
            if(stop_arc<0 && limits.release_time_sec>0) continue;
            auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map);
            if(path.empty() || !checkPlannedPath(ego,reference_map,predictions,limits,path)) continue;
            if(cap==0 || has_initial_progress(path)) {
              result=std::move(path);
              break;
            }
            if(result.empty()) result=std::move(path);
          }
          if(!result.empty() && (cap==0 || has_initial_progress(result))) break;
        }
      }
    }
    return result;
  }

  // Rechecks the cubic-speed / arc-interpolated representation above. This is
  // NOT checkTimedPath's legacy linear-in-time interpolation contract.
  bool checkPlannedPath(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                        const Predictions& predictions, const PlanningConstraints& requested_limits,
                        const std::vector<PathPoint>& path) const {
    requireSceneDomain(ego,reference_map,predictions);
    requirePlanningDomain(ego,requested_limits);
    const auto limits=lateralRecoveryLimits(ego,requested_limits);
    if (path.size()<2) return false;
    if(ego.speed_mps*ego.speed_mps*std::abs(ego.curvature_inv_m)>p_.max_lateral_accel_mps2+1e-6 &&
       path.back().speed_mps>std::min(p_.max_speed_mps,limits.maximum_speed_mps)+1e-8)return false;
    for (std::size_t i=0;i<path.size();++i) {
      const auto& q=path[i];
      if (!Geometry::finite(q.pose) || !std::isfinite(q.curvature_inv_m) ||
          !std::isfinite(q.speed_mps) || !std::isfinite(q.acceleration_mps2) ||
          !std::isfinite(q.progress_m) || !std::isfinite(q.time_sec) ||
          (i==0 && (q.progress_m!=0 || q.time_sec!=0)) ||
          (i>0 && (q.time_sec<=path[i-1].time_sec || q.progress_m<path[i-1].progress_m)))
        throw std::invalid_argument("invalid planned trajectory");
    }
    const double duration=predictions.step_sec*kPredictionSteps-(ego.stamp_sec-predictions.reference_stamp_sec);
    if (ego.stamp_sec<predictions.reference_stamp_sec || path.back().time_sec>duration+1e-12 ||
        Geometry::norm(Geometry::sub(path.front().pose.position,ego.pose.position))>1e-6 ||
        std::abs(Geometry::wrap(path.front().pose.yaw_rad-ego.pose.yaw_rad))>1e-6 ||
        std::abs(path.front().speed_mps-ego.speed_mps)>1e-8 ||
        std::abs(path.front().acceleration_mps2-ego.acceleration_mps2)>1e-8 ||
        std::abs(path.front().curvature_inv_m-ego.curvature_inv_m)>1e-8) return false;
    const Reference reference(reference_map);
    const auto original_geometry=roadGeometry(reference_map,reference);
    const NativeRegionScope reentry_scope(active_native_region_,roadReentryRegion(ego,original_geometry));
    const auto geometry=roadGeometry(reference_map,reference);
    const auto& boundary=geometry->boundary;
    const std::vector<ProgressTime> unused;
    const Context c{reference,predictions,unused,boundary,0,ego.stamp_sec-predictions.reference_stamp_sec,false,&geometry->index,geometry->native_region.get()};
    if(!reentryProgressClear(c,path,true))return false;
    const auto recovery=ceilingRecovery(ego,limits);
    if (!roadClear(c,path.front().pose,path.front().pose) || collision(c,path.front(),path.front())) return false;
    const auto line=stopLine(ego,reference,limits);
    for (std::size_t i=1;i<path.size();++i) {
      const auto& a=path[i-1]; const auto& b=path[i];
      const double distance=Geometry::norm(Geometry::sub(b.pose.position,a.pose.position));
      const double angle=std::abs(Geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad));
      if (!std::isfinite(distance) || (distance==0 && (angle>1e-10 || a.progress_m!=b.progress_m)) ||
          (distance>0 && (b.progress_m==a.progress_m ||
             Geometry::dot(Geometry::sub(b.pose.position,a.pose.position),Geometry::unit(a.pose.yaw_rad))<=0 ||
             Geometry::dot(Geometry::sub(b.pose.position,a.pose.position),Geometry::unit(b.pose.yaw_rad))<=0))) return false;
      const SpeedEdge edge{0,a.speed_mps,a.acceleration_mps2,b.speed_mps,b.acceleration_mps2,
                           a.time_sec,b.time_sec-a.time_sec};
      const double curvature=std::max({std::abs(a.curvature_inv_m),std::abs(b.curvature_inv_m),
                                      distance>0 ? angle/distance : 0});
      if (!speedAdmissible(edge,ego,limits,curvature,recovery) ||
          std::abs(edge.distance(edge.h)-distance)>2e-6 ||
          !roadClear(c,a.pose,b.pose)) return false;
      const std::vector<PathPoint> pair{a,b};
      const ArcPath arc(pair);
      if (!collisionEdgeClear(c,arc,edge)) return false;
      if (std::isfinite(limits.stop_line_progress_m) && a.time_sec<limits.release_time_sec) {
        const double h=std::min(edge.h,limits.release_time_sec-a.time_sec);
        const auto until=arc.at(edge.distance(h));
        if (!behindLine(line,a.pose,until.pose)) return false;
      }
    }
    return sampledGeometryClear(path,true);
  }
  // Retain source compatibility for legacy braced schedules, including {}.
  std::vector<PathPoint> plan(const EgoState& ego, const std::vector<ReferencePoint>& reference,
                                const Predictions& predictions, std::initializer_list<ProgressTime> schedule,
                                const std::vector<PathPoint>& previous = {}) const {
    return plan(ego,reference,predictions,std::vector<ProgressTime>(schedule),previous);
  }

  // Empty means this finite search produced no usable point sequence. It is
  // neither a brake command nor proof that every possible maneuver is blocked.
  std::vector<PathPoint> plan(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
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


  // Geometric proposals may ignore guessed timing, but never road/shape limits.
  // Only final speed planning against the real predictions can approve motion.
  std::vector<PathPoint> generatePath(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                     const Predictions& predictions_map, const std::vector<ProgressTime>& schedule,
                     const std::vector<PathPoint>& previous_path_map, bool geometric_only,
                     double requested_horizon=0,bool retime_proposal=false) const {
    requireSceneDomain(ego,reference_map,predictions_map);
    requireForwardSchedule(schedule);
    for (const auto& point : previous_path_map)
      if (!Geometry::finite(point.pose)) throw std::invalid_argument("nonfinite previous path");
    const Reference reference(reference_map);
    const auto origin=reference.project(ego.pose.position,0.0,reference.length());
    if (origin.distance > p_.max_initial_distance_m)
      return {};
    const auto initial=reference.at(origin.s);
    const double heading_error=Geometry::wrap(ego.pose.yaw_rad-initial.yaw);
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
    double horizon=std::min({maximum_horizon,reference.length()-origin.s-front_reserve-p_.road_margin_m-1e-6,
                                  geometric_only ? maximum_horizon : progressAt(schedule,available_time)});
    // Arc station minus a bumper radius alone is insufficient on a curve:
    // the finite road polygon ends in a straight cap. Reserve the padded
    // footprint's SUPPORT along that cap's normal, without moving the road
    // or the rear-axle output. Lateral/rotated paths still require full checks.
    const auto end=reference.at(reference.length(),true);
    const Vec2 end_direction=Geometry::unit(end.yaw);
    const auto behind_end=[&](double progress) {
      const auto r=reference.at(origin.s+progress);
      const auto before=reference.at(origin.s+std::max(0.0,progress-p_.output_step_m));
      const auto box=Geometry::sweptEgoEnvelope({before.position,before.yaw},{r.position,r.yaw},
                                               p_.vehicle,p_.road_margin_m);
      return Geometry::dot(Geometry::sub(box.pose.position,end.position),end_direction)+
             Geometry::extent(box,end_direction)<-std::max(1e-5,2*p_.qp_tolerance);
    };
    const double remaining=reference.length()-origin.s-horizon;
    // A distant terminal supporting plane can cut through another winding
    // section of the road. Only apply it in the terminal station neighborhood.
    if(horizon>0 && remaining<=front_reserve+p_.road_margin_m+1e-5 && !behind_end(horizon)) {
      double lo=std::max(0.0,horizon-front_reserve),hi=horizon;
      if(!behind_end(lo)) return {};
      for(int iteration=0;iteration<40;++iteration) {
        const double mid=(lo+hi)/2;
        if(behind_end(mid)) lo=mid; else hi=mid;
      }
      horizon=lo;
    }
    if (horizon+1e-9<std::max(p_.min_horizon_m,2.0*p_.output_step_m)) return {};
    const auto geometry=roadGeometry(reference_map,reference);
    const auto& boundary=geometry->boundary;
    const Context c{reference,predictions_map,schedule,boundary,origin.s,prediction_offset,geometric_only,&geometry->index,geometry->native_region.get(),retime_proposal};
    const double initial_metric=initial.metric*(1.0-initial.curvature*origin.l);
    if (initial_metric<=0.05)
      return {};
    const double initial_dl=initial_metric*std::tan(heading_error);

    // A clear centerline with a smooth initial recovery is the nominal
    // solution, not a weighted compromise with an old lateral bias. Keep the rear-axle
    // contract and all footprint/timing checks; a blocked centerline still
    // goes through the existing lateral DP/QP. Integrated timing is checked
    // separately by planSpeed after matching the measured initial curvature.
    {
      std::vector<PathPoint> nominal;
      const int count=std::max(2,static_cast<int>(std::ceil(horizon/p_.output_step_m)));
      // Zero-slope quintic recovery peaks at 1.875*|offset|/extent.
      // Leave room under the same sampled lateral-slope bound as the QP;
      // a nonzero initial heading still needs the explicit check below.
      // #LANE_CENTER_RECOVERY
      // 옆 차선 중앙까지 1~2m를 되돌리는 경우 고정 5m 창은 곡률 한도를 넘을
      // 수 있어, 안전한 직진 유지 후보만 남고 중앙 복귀가 계속 누락된다.
      // 0 기울기 quintic의 최대 |l''|은 6*|offset|/L² 미만이므로 기존 최대
      // 곡률로 후보의 복귀 길이를 확보한다. 이는 map 좌표 참조선에 대한 형상
      // 후보 길이이며 초기 heading·도로 곡률·실제 차체·충돌·시간 계획의 최종
      // 검사와 안전 계수를 대체하지 않는다. 뒷차축 위치와 기준점도 유지한다.
      const double first_recovery=std::min(horizon,std::max({p_.dp_step_m,
          2*std::abs(origin.l)/p_.max_lateral_slope+p_.output_step_m,
          std::sqrt(6*std::abs(origin.l)/p_.max_curvature_inv_m)+p_.output_step_m}));
      // 곡률만 맞아도 비대칭 앞 범퍼가 목표 차선 바깥에 닿을 수 있다.
      // 같은 quintic 복귀 형상을 유한한 창 안에서 길게 다시 검사한다. 모든
      // 후보에 원래 차체·도로·충돌 검사를 적용하며 공간 한도를 넓히지 않는다.
      for(double recovery=first_recovery;;recovery=std::min(horizon,2*recovery)) {
      nominal.clear();
      for(int i=0;i<=count;++i) {
        const double progress=horizon*i/count;
        const auto r=reference.at(origin.s+progress);
        const double l=progress<recovery ? quintic(origin.l,0,initial_dl,0,recovery,progress/recovery) : 0;
        nominal.push_back({{Geometry::add(r.position,Geometry::mul(Geometry::normal(r.yaw),l)),r.yaw},
                           progress,timeAt(schedule,progress),r.curvature});
      }
      setGeometry(nominal);
      nominal.front().pose=ego.pose;
      nominal=withScheduleKnots(nominal,schedule);
      if(lateralSlopeClear(c,nominal) && checkPath(c,nominal) &&
         (geometric_only || retime_proposal || sampledGeometryClear(nominal,false))) return nominal;
      if(recovery>=horizon)break;
      }
    }

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
      offsets.push_back(Geometry::clamp(0.0,lower,upper));
      offsets.push_back(Geometry::clamp(origin.l,lower,upper));
      // Keep the existing avoidance grid, but also represent the smallest
      // rear-axle correction that fits the asymmetric Ioniq5 footprint.
      // A whole-body constraint, not a fixed inward shift or a body-center
      // change to the output coordinate contract, determines this candidate.
      for(const auto& interval:roadLateralIntervals(c,road,road.yaw,lower,upper)) {
        const double centered=Geometry::clamp(0.0,interval.first,interval.second);
        offsets.push_back(centered);
        offsets.push_back((centered+Geometry::clamp(origin.l,interval.first,interval.second))/2);
      }
      std::sort(offsets.begin(),offsets.end());
      offsets.erase(std::unique(offsets.begin(),offsets.end(),[](double a,double b){return std::abs(a-b)<1e-8;}),offsets.end());
      const double prior=previousL(reference,origin.s+i*dp_ds,previous_path_map);
      for (double l : offsets) {
       for (double dl : {-p_.dp_slope_sample,0.0,p_.dp_slope_sample}) {
        if (i==layers && dl!=0.0) continue;
        Node best; best.l=l; best.dl=dl;
        const auto& before=lattice[static_cast<std::size_t>(i-1)];
        std::vector<std::pair<double,std::size_t>> parents;
        for (std::size_t j=0;j<before.size();++j) {
          if (!std::isfinite(before[j].cost)) continue;
          const double delta=l-before[j].l;
          if (std::abs(delta)/dp_ds>p_.max_lateral_slope) continue;
          const double cost=before[j].cost+p_.reference_weight*l*l*dp_ds+
              p_.slope_weight*(delta*delta/dp_ds+dl*dl)+p_.previous_weight*(l-prior)*(l-prior)*dp_ds;
          if(std::isfinite(cost))parents.emplace_back(cost,j);
        }
        // #DP_COST_PRUNING
        // 같은 끝점·끝 기울기의 연결 비용은 형상·도로·충돌 검사와 독립적이다.
        // 기존 비용이 낮은 부모부터 검사하면 첫 유효 연결이 원래의 최솟값이다.
        // stable_sort로 동률의 기존 부모 순서를 보존한다. 실제 선택할 연결에는
        // 원래 검사를 모두 적용하고, 결과를 이길 수 없는 연결의 검사만 생략한다.
        // 차선 허용 범위·차체·충돌·시간 안전 한도와 DP 비용 식은 변경하지 않는다.
        std::stable_sort(parents.begin(),parents.end(),
            [](const std::pair<double,std::size_t>& a,const std::pair<double,std::size_t>& b) {
              return a.first<b.first;
            });
        for(const auto& parent:parents) {
          const auto j=parent.second;
          const auto edge=makeEdge(c,(i-1)*dp_ds,before[j].l,i*dp_ds,l,before[j].dl,dl);
          if (!checkPath(c,edge)) continue;
          best.cost=parent.first; best.parent=static_cast<int>(j);
          break;
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
      guide_path.push_back({{Geometry::add(r.position,Geometry::mul(Geometry::normal(r.yaw),l)),r.yaw},s,timeAt(schedule,s),0.0});
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
      const Vec2 normal=Geometry::normal(road.yaw);
      const auto own=Geometry::egoBox(guide_path[static_cast<std::size_t>(i)].pose,p_.vehicle);
      // Linearize the full footprint around the DP guide's heading. Preserve
      // its connected road component, just as the obstacle constraints below
      // preserve its avoidance side. The final rotated/swept path is still
      // checked after QP; this local constraint is not a safety certificate.
      const auto intervals=roadLateralIntervals(c,road,guide_path[static_cast<std::size_t>(i)].pose.yaw_rad,
                                               lower[i],upper[i]);
      const auto interval=std::find_if(intervals.begin(),intervals.end(),[&](const std::pair<double,double>& range) {
        return guide[i]>=range.first && guide[i]<=range.second;
      });
      if(interval==intervals.end()) return {};
      lower[i]=interval->first; upper[i]=interval->second;
      for (std::size_t j=0;!geometric_only && j<predictions_map.current.size();++j) {
        const auto other=Geometry::objectBox(samplePrediction(predictions_map,j,
            prediction_offset+guide_path[static_cast<std::size_t>(i)].time_sec),p_.obstacle_margin_m);
        const auto d=Geometry::sub(other.pose.position,own.pose.position);
        double best_width=-1.0, best_lo=lower[i], best_hi=upper[i];
        for (Vec2 axis : {Geometry::unit(own.pose.yaw_rad),Geometry::normal(own.pose.yaw_rad),
                          Geometry::unit(other.pose.yaw_rad),Geometry::normal(other.pose.yaw_rad)}) {
          const double delta=Geometry::dot(d,axis);
          const double slack=std::abs(delta)-Geometry::extent(own,axis)-Geometry::extent(other,axis)-1e-6;
          if (slack<=0.0) continue;
          const double coefficient=(delta>=0.0 ? 1.0 : -1.0)*Geometry::dot(normal,axis);
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
      result.push_back({{Geometry::add(r.position,Geometry::mul(Geometry::normal(r.yaw),solution[i])),r.yaw},i*ds,timeAt(schedule,i*ds),0.0});
    }
    setGeometry(result);
    // Initial pose/yaw are execution boundary conditions, not a reference reset.
    if (Geometry::norm(Geometry::sub(result.front().pose.position,ego.pose.position))>1e-3) return {};
    if (std::abs(Geometry::wrap(result.front().pose.yaw_rad-ego.pose.yaw_rad))>0.05) return {};
    result.front().pose=ego.pose;
    result=withScheduleKnots(result,schedule);
    if (checkPath(c,result)) return result;
    // A fixed-heading QP bound cannot certify its newly rotated footprint.
    // Keep the CURRENT DP proposal as another numerical candidate when the
    // converged refinement is unusable, but recheck its complete geometry and
    // timing. Never substitute an old path or relax any collision/road limit.
    // Solver nonconvergence still returns empty above, not an approved result.
    guide_path=withScheduleKnots(guide_path,schedule);
    if(lateralSlopeClear(c,guide_path) && checkPath(c,guide_path) &&
       (geometric_only || retime_proposal || sampledGeometryClear(guide_path,false)))
      return guide_path;
    return {};
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
      if (!Geometry::finite(point.pose) || !std::isfinite(point.curvature_inv_m) ||
          !std::isfinite(point.progress_m) || !std::isfinite(point.time_sec) ||
          (i==0 && (point.progress_m!=0.0 || point.time_sec!=0.0)) ||
          (i>0 && (point.progress_m<path_map[i-1].progress_m || point.time_sec<=path_map[i-1].time_sec)))
        throw std::invalid_argument("invalid timed forward path");
      if (i>0 && ((point.progress_m==path_map[i-1].progress_m)!=
                 (Geometry::norm(Geometry::sub(point.pose.position,path_map[i-1].pose.position))==0.0)))
        throw std::invalid_argument("stationary position and stationary progress must agree");
    }
    if (path_map.size()<2) return false;
    if (Geometry::norm(Geometry::sub(path_map.front().pose.position,ego.pose.position))>1e-6 ||
        std::abs(Geometry::wrap(path_map.front().pose.yaw_rad-ego.pose.yaw_rad))>1e-6) return false;
    const double offset=ego.stamp_sec-predictions_map.reference_stamp_sec;
    // Unlike generation, validation must not silently shorten the requested path.
    // Account only for roundoff when inverting t(s) at the prediction endpoint,
    // not an operational grace period or permission to extrapolate predictions.
    const double horizon=predictions_map.step_sec*kPredictionSteps;
    const double roundoff=16.0*std::numeric_limits<double>::epsilon()*std::max(1.0,horizon);
    if (offset<0.0 || offset+path_map.back().time_sec>horizon+roundoff) return false;
    const Reference reference(reference_map);
    const auto geometry=roadGeometry(reference_map,reference);
    const auto& boundary=geometry->boundary;
    const std::vector<ProgressTime> unused_schedule;
    const Context c{reference,predictions_map,unused_schedule,boundary,0.0,offset,false,&geometry->index,geometry->native_region.get()};
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
        if (!points.empty() && Geometry::norm(Geometry::sub(p.position,points.back().position))<1e-6) {
          points.back().min_l_m=std::max(points.back().min_l_m,p.min_l_m);
          points.back().max_l_m=std::min(points.back().max_l_m,p.max_l_m);
        } else points.push_back(p);
      }
      if (points.size()<2) throw std::invalid_argument("reference has fewer than two distinct points");
      station.push_back(0.0);
      for (std::size_t i=1;i<points.size();++i) {
        const double next=station.back()+Geometry::norm(Geometry::sub(points[i].position,points[i-1].position));
        if (!std::isfinite(next)) throw std::invalid_argument("reference station arithmetic overflow");
        station.push_back(next);
      }
      // Interpolate the map knots with a C2 natural cubic, rather than using
      // averages of unit chord directions as Hermite derivatives. Those
      // shortened derivatives systematically pull sparse curves inward and
      // leave curvature jumps at joins. Preserve the original road polygon
      // separately below: improving the target must not expand lane permission.
      const std::size_t n=points.size();
      std::vector<Vec2> second(n),rhs(n);
      std::vector<double> upper(n,0.0);
      const auto direction=[&](std::size_t i) {
        return Geometry::mul(Geometry::sub(points[i+1].position,points[i].position),
                             1.0/(station[i+1]-station[i]));
      };
      for(std::size_t i=1;i+1<n;++i) {
        const double before=station[i]-station[i-1],after=station[i+1]-station[i];
        const double diagonal=2*(before+after)-before*upper[i-1];
        upper[i]=after/diagonal;
        rhs[i]=Geometry::mul(Geometry::sub(Geometry::mul(Geometry::sub(direction(i),direction(i-1)),6),
                                          Geometry::mul(rhs[i-1],before)),1.0/diagonal);
      }
      for(std::size_t i=n-1;i>1;--i)
        second[i-1]=Geometry::sub(rhs[i-1],Geometry::mul(second[i],upper[i-1]));
      tangents.resize(n);
      for(std::size_t i=0;i+1<n;++i)
        tangents[i]=Geometry::sub(direction(i),Geometry::mul(Geometry::add(Geometry::mul(second[i],2),second[i+1]),
                                  (station[i+1]-station[i])/6));
      tangents.back()=Geometry::add(direction(n-2),Geometry::mul(Geometry::add(second[n-2],Geometry::mul(second[n-1],2)),
                                       (station[n-1]-station[n-2])/6));
      // #PROJECTION_PRUNING
      // map 참조선의 C2 구간을 Bezier 제어점 hull의 AABB로 감싼다. 매 경로점
      // 투영에서 멀리 떨어진 모든 구간에 Newton 계산을 반복하는 비용을 줄인다.
      // 좌표 반올림 여유를 포함한 거리 하한이 이미 계산한 후보 거리보다 큰
      // 구간만 제외한다. 원래 spline·5회 투영 계산과 구간 순서/동률 우선순위는
      // 유지한다. 큰 map 좌표나 굽은 구간에서도 chord만으로 경계를 만들면
      // 실제 spline 돌출부를 누락하므로 네 제어점을 모두 포함해야 한다.
      segment_bounds.reserve(n-1);
      for(std::size_t i=0;i+1<n;++i) {
        Geometry::Bounds bounds;
        const double h=station[i+1]-station[i];
        for(const auto point:{points[i].position,points[i+1].position,
            Geometry::add(points[i].position,Geometry::mul(tangents[i],h/3)),
            Geometry::sub(points[i+1].position,Geometry::mul(tangents[i+1],h/3))})
          bounds.include(point);
        const double scale=std::max({1.0,std::abs(bounds.min_x),std::abs(bounds.min_y),
                                    std::abs(bounds.max_x),std::abs(bounds.max_y)});
        const double padding=128*std::numeric_limits<double>::epsilon()*scale;
        bounds.min_x-=padding;bounds.min_y-=padding;bounds.max_x+=padding;bounds.max_y+=padding;
        segment_bounds.push_back(bounds);
      }
    }
    double length() const { return station.back(); }
    // The allowed road is the resulting piecewise-linear closed polygon.
    // Retain EVERY knot: a narrow notch must not disappear between grid samples.
    std::vector<Vec2> boundary(double step) const {
      std::vector<Vec2> left,right;
      const auto append=[&](double s) {
        const auto r=at(s,true);
        const Vec2 normal=Geometry::normal(r.yaw);
        left.push_back(Geometry::add(r.position,Geometry::mul(normal,r.upper)));
        right.push_back(Geometry::add(r.position,Geometry::mul(normal,r.lower)));
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
          const auto a=Geometry::sub(polygon.back(),polygon[polygon.size()-2]);
          const auto b=Geometry::sub(point,polygon.back());
          if(Geometry::cross(a,b)!=0 || Geometry::dot(a,b)<=0) break;
          polygon.pop_back();
        }
        polygon.push_back(point);
      }
      return polygon;
    }
    RoadSample at(double s,bool road_boundary=false) const {
      s=Geometry::clamp(s,0.0,length());
      const std::size_t i=segment(s);
      const double u=(s-station[i])/(station[i+1]-station[i]);
      const double h=station[i+1]-station[i];
      const Vec2 direction=Geometry::mul(Geometry::sub(points[i+1].position,points[i].position),1.0/h);
      const Vec2 first=road_boundary ? boundaryTangent(i) : tangents[i];
      const Vec2 last=road_boundary ? boundaryTangent(i+1) : tangents[i+1];
      if (Geometry::norm(Geometry::sub(first,direction))<1e-12 &&
          Geometry::norm(Geometry::sub(last,direction))<1e-12)
        return {Geometry::lerp(points[i].position,points[i+1].position,u),std::atan2(direction.y,direction.x),
                Geometry::lerp(points[i].min_l_m,points[i+1].min_l_m,u),Geometry::lerp(points[i].max_l_m,points[i+1].max_l_m,u)};
      const Vec2 m0=Geometry::mul(first,h),m1=Geometry::mul(last,h);
      const Vec2 delta=Geometry::sub(points[i+1].position,points[i].position);
      const double u2=u*u,u3=u2*u;
      // Local differences avoid cancellation at large map/UTM coordinates.
      const Vec2 pos=Geometry::add(points[i].position,Geometry::add(Geometry::mul(delta,-2*u3+3*u2),
          Geometry::add(Geometry::mul(m0,u3-2*u2+u),Geometry::mul(m1,u3-u2))));
      const Vec2 d=Geometry::add(Geometry::mul(delta,-6*u2+6*u),
          Geometry::add(Geometry::mul(m0,3*u2-4*u+1),Geometry::mul(m1,3*u2-2*u)));
      const Vec2 dd=Geometry::add(Geometry::mul(delta,-12*u+6),
          Geometry::add(Geometry::mul(m0,6*u-4),Geometry::mul(m1,6*u-2)));
      const double speed=Geometry::norm(d);
      if (speed<1e-9) throw std::invalid_argument("reference tangent degeneracy/cusp");
      return {pos,std::atan2(d.y,d.x),
              Geometry::lerp(points[i].min_l_m,points[i+1].min_l_m,u),Geometry::lerp(points[i].max_l_m,points[i+1].max_l_m,u),
              speed/h,(d.x*dd.y-d.y*dd.x)/(speed*speed*speed)};
    }
    Projection project(Vec2 p,double lo,double hi) const {
      Projection best;
      const std::size_t begin=segment(Geometry::clamp(lo,0.0,length()));
      const std::size_t end=segment(Geometry::clamp(hi,0.0,length()));
      if(begin>end) return best;
      const auto lowerDistance=[&](std::size_t i) {
        const auto& box=segment_bounds[i];
        if(!std::isfinite(box.min_x) || !std::isfinite(box.min_y) ||
           !std::isfinite(box.max_x) || !std::isfinite(box.max_y)) return 0.0;
        return std::hypot(std::max({0.0,box.min_x-p.x,p.x-box.max_x}),
                          std::max({0.0,box.min_y-p.y,p.y-box.max_y}));
      };
      // Evaluate one promising segment only to obtain an upper distance bound.
      // The final scan remains in the original order, including all ties.
      std::size_t nearest=begin;
      double lower=lowerDistance(begin);
      for(std::size_t i=begin+1;i<=end;++i) {
        const double distance=lowerDistance(i);
        if(distance<lower) { lower=distance;nearest=i; }
      }
      const auto evaluate=[&](std::size_t i) {
        const auto d=Geometry::sub(points[i+1].position,points[i].position);
        const double len=station[i+1]-station[i];
        const double u=Geometry::clamp(Geometry::dot(Geometry::sub(p,points[i].position),d)/(len*len),0.0,1.0);
        double s=station[i]+u*len;
        for (int k=0;k<5;++k) {
          const auto r=at(s);
          const double next=Geometry::clamp(s+Geometry::dot(Geometry::sub(p,r.position),Geometry::unit(r.yaw)),station[i],station[i+1]);
          if (std::abs(next-s)<1e-8) break;
          s=next;
        }
        const auto r=at(s);
        const auto delta=Geometry::sub(p,r.position);
        return Projection{s,Geometry::dot(Geometry::normal(r.yaw),delta),Geometry::norm(delta)};
      };
      const auto seed=evaluate(nearest);
      double upper=seed.distance;
      for(std::size_t i=begin;i<=end;++i) {
        if(lowerDistance(i)>upper) continue;
        const auto candidate=i==nearest?seed:evaluate(i);
        if(candidate.distance<best.distance) { best=candidate;upper=std::min(upper,best.distance); }
      }
      return best;
    }
    bool ambiguous(Vec2 p,const Projection& best,double yaw) const {
      for (std::size_t i=0;i+1<points.size();++i) {
        if (std::abs(station[i]-best.s)<5.0) continue;
        const auto q=project(p,station[i],station[i]);
        const auto r=at(q.s);
        if (std::abs(q.s-best.s)>5.0 && q.distance<best.distance+0.05 && std::abs(Geometry::wrap(r.yaw-yaw))<0.5) return true;
      }
      return false;
    }
   private:
    std::vector<ReferencePoint> points;
    std::vector<double> station;
    std::vector<Vec2> tangents;
    std::vector<Geometry::Bounds> segment_bounds;
    // Lane permission remains the prior map-knot/chord-Hermite polygon.
    // It is independent of the smoothed nominal target's interpolation.
    Vec2 boundaryTangent(std::size_t i) const {
      const auto direction=[&](std::size_t j) {
        return Geometry::mul(Geometry::sub(points[j+1].position,points[j].position),
                             1.0/(station[j+1]-station[j]));
      };
      if(i==0) return direction(0);
      if(i+1==points.size()) return direction(i-1);
      return Geometry::mul(Geometry::add(direction(i-1),direction(i)),.5);
    }
    std::size_t segment(double s) const {
      auto it=std::upper_bound(station.begin(),station.end(),s);
      return std::min(static_cast<std::size_t>(std::max<std::ptrdiff_t>(0,it-station.begin()-1)),points.size()-2);
    }
  };

  // #ROAD_GEOMETRY_REUSE
  // 차량 위치와 객체가 변해도 같은 route의 전체 허용 도로 경계는 같다.
  // 원본 꼭짓점/좌우 경계를 정확히 비교해 변경되면 새 다각형과 BVH를 만든다.
  // ROI나 경계 간격을 줄이지 않고, 생성 후보와 최종 검증이 같은 불변 경계를
  // 참조한다. 소유권은 Core 안에 있으며 다음 route로 바뀌어도 사용 중인 경계의
  // 수명이 끝나지 않는다. 파라미터는 Core 생성 시 고정된 기존 값을 사용한다.
  struct RoadGeometry {
    std::vector<ReferencePoint> input;
    std::vector<Vec2> boundary;
    Geometry::BoundaryIndex index;
    std::shared_ptr<const NativeRoadRegion> native_region;
    RoadGeometry(const std::vector<ReferencePoint>& points,const Reference& reference,double step,
                 std::shared_ptr<const NativeRoadRegion> native)
        : input(points),boundary(reference.boundary(step)),index(boundary),native_region(std::move(native)) {}
  };
  lanelet::routing::RoutingGraphConstPtr routing_graph_;
  mutable std::shared_ptr<const RoadGeometry> road_geometry_;
  mutable std::set<std::uint32_t> pedestrian_caution_ids_;
  std::shared_ptr<const RoadGeometry> roadGeometry(const std::vector<ReferencePoint>& points,
                                                const Reference& reference) const {
    if(!road_geometry_ || road_geometry_->native_region!=active_native_region_ ||
       road_geometry_->input.size()!=points.size() ||
       !std::equal(points.begin(),points.end(),road_geometry_->input.begin(),
          [](const ReferencePoint& a,const ReferencePoint& b) {
            return a.position.x==b.position.x && a.position.y==b.position.y &&
                   a.min_l_m==b.min_l_m && a.max_l_m==b.max_l_m;
          })) road_geometry_=std::make_shared<const RoadGeometry>(points,reference,p_.road_boundary_step_m,active_native_region_);
    return road_geometry_;
  }
  struct ObjectBounds {
    Geometry::Bounds centers;
    double half_length=0,half_width=0,rotation_delta=0;
  };
  static std::vector<ObjectBounds> predictionBounds(const Predictions& predictions) {
    std::vector<ObjectBounds> result(predictions.current.size());
    for(std::size_t j=0;j<result.size();++j) {
      double acceleration=0,previous_yaw=predictions.current[j].pose.yaw_rad;
      const auto include=[&](const PredictedObject& o) {
        auto& bound=result[j]; bound.centers.include(o.pose.position);
        const double rounding=128*std::numeric_limits<double>::epsilon()*
            std::max({1.0,std::abs(previous_yaw),std::abs(o.pose.yaw_rad)});
        bound.rotation_delta=std::max(bound.rotation_delta,std::min(Geometry::kPi,
            std::abs(Geometry::wrap(o.pose.yaw_rad-previous_yaw))+rounding+1e-10));
        previous_yaw=o.pose.yaw_rad;
        bound.half_length=std::max(bound.half_length,o.length_m/2+o.radius_error_m);
        bound.half_width=std::max(bound.half_width,o.width_m/2+o.radius_error_m);
        acceleration=std::max(acceleration,o.interpolation_acceleration_mps2);
      };
      include(predictions.current[j]);
      for(const auto& future:predictions.future) include(future[j]);
      const double interpolation_pad=acceleration*predictions.step_sec*predictions.step_sec/8;
      result[j].half_length+=interpolation_pad; result[j].half_width+=interpolation_pad;
    }
    return result;
  }
  struct FollowingObject {
    Geometry::Box box;
    Vec2 heading;
  };
  struct Context {
    mutable std::map<double,std::vector<FollowingObject>> following_objects;
    const Reference& reference;
    const Predictions& predictions;
    const std::vector<ProgressTime>& schedule;
    const std::vector<Vec2>& boundary;
    double start_s;
    double prediction_offset;
    bool geometric_only;
    bool proposal_timing;
    std::shared_ptr<const Geometry::BoundaryIndex> owned_index;
    const Geometry::BoundaryIndex& boundary_index;
    std::vector<ObjectBounds> object_bounds;
    const NativeRoadRegion* native_region;
    Context(const Reference& ref,const Predictions& pred,const std::vector<ProgressTime>& timing,
            const std::vector<Vec2>& polygon,double start,double offset,bool geometric=false,
            const Geometry::BoundaryIndex* index=nullptr,const NativeRoadRegion* native=nullptr,
            bool retime_proposal=false)
        : reference(ref),predictions(pred),schedule(timing),boundary(polygon),start_s(start),
          prediction_offset(offset),geometric_only(geometric),proposal_timing(retime_proposal),
          owned_index(index?nullptr:std::make_shared<const Geometry::BoundaryIndex>(polygon)),
          boundary_index(index?*index:*owned_index),object_bounds(predictionBounds(pred)),native_region(native) {}
  };

  // #ROAD_REENTRY
  // 차체가 경계를 걸친 상태에서는 원래 포함 검사로 초기점부터 모든 후보가
  // 탈락한다. 지도와 허용 차선은 그대로 두고 이 호출의 기존 C2 후보에만
  // 짧은 복귀 통로를 제공한다. 처음 차체의 진행 방향 sweep(기존 horizon_m)
  // 과 원래 도로의 합집합이며, 지도 내부 구멍과 차선 변경 방향은 보존한다.
  // 통로 자체는 주행 승인이 아니다. 최종 시간 경로에서 원래 도로 밖 차체
  // 면적이 늘지 않고 horizon_m 안에 완전히 복귀하는지 다시 검사한다.
  // 복귀 후에는 원래 연속 차체 경계 검사를 적용한다. 충돌·곡률·속도·가감속
  // 검사는 기존 계산 그대로다. 완전히 도로와 떨어진 차체는 이 대상으로
  // 삼지 않는다. LiDAR/GT 구분 없이 같은 map 도로·객체 계약을 사용한다.
  std::shared_ptr<const NativeRoadRegion> roadReentryRegion(
      const EgoState& ego,const std::shared_ptr<const RoadGeometry>& geometry) const {
    auto original=geometry->native_region;
    if(!original || original->recovery_original)return original;
    const auto initial=Geometry::sweptEgoEnvelope(ego.pose,ego.pose,p_.vehicle,p_.road_margin_m);
    if(original ? original->containsBox(initial) : geometry->index.containsBox(initial))return original;
    const Reference reference(geometry->input);
    const auto anchor=reference.project(ego.pose.position,0,reference.length());
    // 옆 경계 복귀와 지도 시작/끝의 누락은 구분한다. 실제 전후 도로가
    // 없는 곳에 차체 길이를 더한 가상 종방향 도로를 만들지 않는다.
    if(anchor.s<p_.vehicle.rear_m+p_.road_margin_m ||
       reference.length()-anchor.s<p_.vehicle.front_m+p_.road_margin_m)return original;
    const double outside=original->outsideArea(initial);
    if(outside>=4*initial.half_length*initial.half_width-1e-7)return original;
    boost::geometry::model::multi_point<RoadPoint> corners;
    // 초기 회전 때 뒷범퍼가 그리는 sweep의 수치 여유는 기존 road_margin_m만
    // 사용한다. 밖으로 더 나가는 후보는 아래 면적 검사를 통과하지 못한다.
    const auto footprint=Geometry::sweptEgoEnvelope(ego.pose,ego.pose,p_.vehicle,2*p_.road_margin_m);
    const auto direction=Geometry::mul(Geometry::unit(ego.pose.yaw_rad),p_.horizon_m);
    for(const auto& corner:Geometry::corners(footprint)) {
      corners.emplace_back(corner.x,corner.y);
      corners.emplace_back(corner.x+direction.x,corner.y+direction.y);
    }
    RoadPolygon corridor;boost::geometry::convex_hull(corners,corridor);
    auto region=std::make_shared<NativeRoadRegion>();
    boost::geometry::union_(original->polygons,corridor,region->polygons);
    // 후보 차체는 위 합집합 안에 있으므로, 도로 밖 면적은 통로에서 원래
    // 도로를 뺀 작은 영역과의 교집합으로 정확히 같다. 전역 지도 차집합을
    // 시간 후보마다 반복하지 않고 한 번 계산해 계산 지연을 줄인다.
    boost::geometry::difference(corridor,original->polygons,region->recovery_outside);
    for(const auto& polygon:original->polygons)for(const auto& ring:polygon.inners()) {
      RoadPolygon hole;hole.outer()=ring;boost::geometry::correct(hole);
      RoadPolygons remaining;boost::geometry::difference(region->polygons,hole,remaining);
      region->polygons=std::move(remaining);
    }
    region->one_way_boundaries=original->one_way_boundaries;
    region->recovery_original=original;
    region->margin_only_recovery=original->containsBox(Geometry::egoBox(ego.pose,p_.vehicle));
    region->initial_outside_area_m2=region->margin_only_recovery ? 0 : region->reentryOutsideArea(initial);
    region->prepare();return region;
  }

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
  void pruneSpeedLayer(std::vector<SpeedState>& next) const {
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
    // #SPEED_SEARCH_PROGRESS_RESOLUTION
    // 속도 상한 20m/s와 시간 격자 0.5초를 곱한 10m band는 서로 다른
    // 감속 도착 상태를 합쳐 완만한 제동 경로를 조기에 버린다. 기존 최소
    // 공간 간격 0.5m(속도 탐색의 0.25m 위치 cell 두 개)를 유지한다.
    // 같은 예측 시각의 진행 거리만 구분하며 가속도 부호·정확한 대기 상태,
    // 대표 상태 예산 1/4과 전체 beam 크기는 유지한다. 도로/ROI·차체·충돌·
    // 속도·횡가속도·가감속·jerk 검사와 최종 출력 재검사는 그대로 적용한다.
    const double width=.5;
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
  // #MEASURED_LATERAL_RECOVERY
  // 입력의 실측 v·a·곡률은 이미 발생한 뒷차축 상태다. 횡가속도가 한도를
  // 넘었다고 전부 거부하면 안전한 감속 경로도 사라져 제어 연결이 끊긴다.
  // 초기 |곡률|에서 한도를 만족하는 속도를 구하고 기존 ceilingRecovery의
  // 최대 jerk·가감속으로 계산한 가장 빠른 S-curve를 고정 복귀 상한으로 쓴다.
  // 한도 초과 중에는 초기보다 더 큰 곡률을 허용하지 않는다. 계획 전체가
  // 이 포락선 이하인지 다항식 극값으로 검사하고, 복귀 뒤에는 원래 횡가속도
  // 한도가 성립한다. 정상 초기 상태에는 적용하지 않으며 실측값 clipping,
  // 속도 offset, 제동/조향 변경, 도로·충돌 여유 완화는 수행하지 않는다.
  // GT/LiDAR 구분 없이 map 객체와 동일한 전체 차체 검사를 계속 사용한다.
  PlanningConstraints lateralRecoveryLimits(const EgoState& ego,PlanningConstraints limits) const {
    const double curvature=std::abs(ego.curvature_inv_m);
    if(ego.speed_mps*ego.speed_mps*curvature>p_.max_lateral_accel_mps2+1e-6)
      limits.maximum_speed_mps=std::min(limits.maximum_speed_mps,
          std::sqrt(p_.max_lateral_accel_mps2/curvature));
    return limits;
  }
  bool recoveringLateralExcess(const EgoState& ego,double curvature,
                              const CeilingRecovery& recovery) const {
    const double initial=std::abs(ego.curvature_inv_m);
    return ego.speed_mps*ego.speed_mps*initial>p_.max_lateral_accel_mps2+1e-6 &&
        curvature<=initial+1e-8 &&
        recovery.cap<=std::sqrt(p_.max_lateral_accel_mps2/initial)+1e-8;
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
    Geometry::Bounds bounds;
    double heading_per_meter=0;
    double yaw_scale=2*Geometry::kPi;
    explicit ArcPath(const std::vector<PathPoint>& input):points(input),station(1,0) {
      for(std::size_t i=1;i<input.size();++i)
        station.push_back(station.back()+Geometry::norm(Geometry::sub(input[i].pose.position,input[i-1].pose.position)));
      for(const auto& point:input) { bounds.include(point.pose.position); yaw_scale=std::max(yaw_scale,std::abs(point.pose.yaw_rad)); }
      for(std::size_t i=1;i<input.size();++i) {
        const double distance=station[i]-station[i-1];
        const double angle=std::abs(Geometry::wrap(input[i].pose.yaw_rad-input[i-1].pose.yaw_rad));
        if(distance>0) heading_per_meter=std::max(heading_per_meter,angle/distance);
        else if(angle>0) heading_per_meter=std::numeric_limits<double>::infinity();
      }
      position_curvature.resize(input.size(),0);
      for(std::size_t i=1;i+1<input.size();++i)
        position_curvature[i]=std::abs(Geometry::curvature(input[i-1].pose.position,input[i].pose.position,
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
          const auto d0=Geometry::sub(b.pose.position,a.pose.position),d1=Geometry::sub(c.pose.position,b.pose.position);
          const double n0=Geometry::norm(d0),n1=Geometry::norm(d1);
          if(n0>0 && n1>0 && Geometry::dot(d0,d1)>0 && std::abs(Geometry::cross(d0,d1))<=1e-12*n0*n1 &&
             std::abs(Geometry::wrap(a.pose.yaw_rad-b.pose.yaw_rad))<=1e-12 &&
             std::abs(Geometry::wrap(b.pose.yaw_rad-c.pose.yaw_rad))<=1e-12 &&
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
      return {{Geometry::lerp(a.pose.position,b.pose.position,u),Geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u)},
              Geometry::lerp(a.progress_m,b.progress_m,u),0,
              Geometry::lerp(a.curvature_inv_m,b.curvature_inv_m,u)};
    }
    double curvature(double lo,double hi) const {
      double k=std::max(std::abs(at(lo).curvature_inv_m),std::abs(at(hi).curvature_inv_m));
      auto it=std::upper_bound(station.begin(),station.end(),lo);
      for(;it!=station.end() && *(it-1)<=hi;++it) {
        const std::size_t i=static_cast<std::size_t>(it-station.begin());
        k=std::max({k,std::abs(points[i-1].curvature_inv_m),std::abs(points[i].curvature_inv_m),
                    position_curvature[i-1],position_curvature[i],
                    std::abs(Geometry::wrap(points[i].pose.yaw_rad-points[i-1].pose.yaw_rad))/(*it-*(it-1))});
      }
      return k;
    }
  };

  void requirePlanningDomain(const EgoState& ego,const PlanningConstraints& limits) const {
    for(double x : {ego.speed_mps,ego.acceleration_mps2,ego.curvature_inv_m})
      if(!std::isfinite(x)) throw std::invalid_argument("nonfinite initial motion");
    if(ego.speed_mps<0) throw std::invalid_argument("reverse motion is outside forward planning domain");
    for(double x : {limits.maximum_speed_mps,limits.stop_line_progress_m,limits.release_time_sec,limits.preferred_speed_mps})
      if(std::isnan(x) || x<0) throw std::invalid_argument("negative/nonfinite planning constraint");
  }

  bool sampledGeometryClear(const std::vector<PathPoint>& path,bool planned) const {
    EgoState initial;
    if(planned && !path.empty()) {
      initial.speed_mps=path.front().speed_mps;
      initial.acceleration_mps2=path.front().acceleration_mps2;
      initial.curvature_inv_m=path.front().curvature_inv_m;
    }
    const auto recovery=ceilingRecovery(initial,lateralRecoveryLimits(initial,PlanningConstraints{}));
    std::vector<std::size_t> support;
    for(std::size_t i=1;i<path.size();++i) {
      const auto d=Geometry::sub(path[i].pose.position,path[i-1].pose.position);
      const double length=Geometry::norm(d);
      if(length>1e-9) {
        const double direction=std::atan2(d.y,d.x);
        // Arc interpolation subdivides the same geometric chord; its yaw is
        // the underlying curve tangent, not the piecewise-linear chord.
        // Allow only the angle error of one configured spatial sample.
        const double tolerance=p_.max_curvature_inv_m*p_.output_step_m+1e-5;
        if(std::abs(Geometry::wrap(direction-path[i-1].pose.yaw_rad))>tolerance ||
           std::abs(Geometry::wrap(direction-path[i].pose.yaw_rad))>tolerance) return false;
      }
    }
    const ArcPath arc(path);
    // Retain every actual turn. Uniform resampling alone can alias geometry
    // whose spacing is finer than output_step_m. Only collinear subdivisions
    // (including waits) are removed; their timing remains in the original path.
    for(std::size_t i=0;i<path.size();++i) {
      if(!support.empty() && arc.station[i]-arc.station[support.back()]<=1e-9) continue;
      while(support.size()>1) {
        const auto a=Geometry::sub(path[support.back()].pose.position,
                                   path[support[support.size()-2]].pose.position);
        const auto b=Geometry::sub(path[i].pose.position,path[support.back()].pose.position);
        if(Geometry::dot(a,b)<=0 || std::abs(std::atan2(Geometry::cross(a,b),Geometry::dot(a,b)))>1e-8) break;
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
            if(std::pow(edge.speed(t),2)*k>p_.max_lateral_accel_mps2+1e-6 &&
               !(recoveringLateralExcess(initial,k,recovery) && belowRecovery(edge,recovery)))return false;
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
      double k=std::abs(Geometry::curvature(a,b,c));
      const auto left=Geometry::sub(b,a),right=Geometry::sub(c,b);
      const double l=Geometry::norm(left),r=Geometry::norm(right);
      // A final time endpoint can truncate an interpolated spatial chord.
      // Positions alone cannot distinguish that case from an actual short
      // corner: only use the preceding support when both the supplied curve
      // curvature and its independent yaw change support that interpretation.
      // Interior turns, however finely spaced, always keep local curvature.
      if(i+2==support.size() && r<l && r<p_.output_step_m) {
        const double turn=std::atan2(Geometry::cross(left,right),Geometry::dot(left,right));
        const double sign=turn<0 ? -1 : 1;
        const double endpoint_k=std::abs(turn)/l;
        const double declared_k=std::max({sign*path[support[i-1]].curvature_inv_m,
                                        sign*path[support[i]].curvature_inv_m,
                                        sign*path[support[i+1]].curvature_inv_m});
        const double yaw_k=sign*Geometry::wrap(path[support[i]].pose.yaw_rad-
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
      const double k=std::abs(Geometry::curvature(arc.at(s-step).pose.position,arc.at(s).pose.position,
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
    const auto envelope=Geometry::sweptEgoEnvelope(a,b,p_.vehicle,p_.road_margin_m);
    const auto normal=Geometry::unit(line.yaw);
    return Geometry::dot(Geometry::sub(envelope.pose.position,line.position),normal)+
           Geometry::extent(envelope,normal)<-1e-8;
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
    const double dl=f*std::tan(Geometry::wrap(yaw-r.yaw));
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
    return {{Geometry::add(r.position,Geometry::mul(Geometry::normal(r.yaw),l)),
             Geometry::wrap(r.yaw+std::atan2(dl,f))},progress,progress,k};
  }
  // #CARTESIAN_CURVATURE_RECOVERY
  // 조밀한 지도 중심선의 작은 흔들림을 그대로 따라가면 Frenet 기준 곡률이
  // 실제 차량 한도를 넘을 수 있다. 동일한 초기/끝 자세를 map XY의 C2 quintic으로
  // 보간하는 추가 기하 후보를 만든다. 뒷차축 위치·yaw·초기 곡률은 정확히 유지한다.
  // 끝점은 기존 reference의 횡 offset이다. 끝 곡률은 dp_step_m 간격의 세
  // map 위치로 계산하여 작은 중심선 흔들림의 미분 증폭을 피하고, 상수 횡 offset
  // 곡률식 k/(1-k*l)을 적용한다. 후보 끝 곡률에도 기존 한도를 적용한다.
  // 지도·도로 경계는 수정하지 않는다. 전체 차체, 진행 방향, 횡변화율과 곡률을
  // 기존 검사로 승인한 후보만 기존 속도·충돌·jerk 계산에 전달한다.
  RoadSample cartesianEndpoint(const Reference& reference,double station) const {
    auto end=reference.at(station);
    const double window=std::min(p_.dp_step_m,reference.length()/2);
    const double center=Geometry::clamp(station,window,reference.length()-window);
    const auto before=reference.at(center-window).position;
    const auto middle=reference.at(center).position;
    const auto after=reference.at(center+window).position;
    // 목표 끝점의 방향도 같은 세 공간 샘플로 계산한다. 비균일/흔들리는
    // map knot의 순간 tangent를 따라 조향이 바뀌지 않도록 하며 위치는 보존한다.
    const auto tangent=Geometry::add(Geometry::mul(Geometry::sub(after,before),.5),
        Geometry::mul(Geometry::sub(Geometry::sub(after,middle),Geometry::sub(middle,before)),
                      (station-center)/window));
    if(Geometry::norm(tangent)>1e-9) end.yaw=std::atan2(tangent.y,tangent.x);
    end.curvature=Geometry::curvature(before,middle,after);
    return end;
  }
  std::vector<PathPoint> cartesianRecovery(const Context& c,const EgoState& ego,
                                         double extent,double terminal_l,
                                         bool zero_initial_curvature_slope=false) const {
    const auto origin=c.reference.project(ego.pose.position,0,c.reference.length());
    if(extent<=0 || origin.s+extent>c.reference.length()) return {};
    const auto end=cartesianEndpoint(c.reference,origin.s+extent);
    const auto goal=Geometry::add(end.position,Geometry::mul(Geometry::normal(end.yaw),terminal_l));
    const double reference_curvature=end.curvature;
    const double offset_metric=1-reference_curvature*terminal_l;
    if(offset_metric<=.05) return {};
    const double terminal_curvature=Geometry::clamp(reference_curvature/offset_metric,
                                                  -p_.max_curvature_inv_m,p_.max_curvature_inv_m);
    // #INITIAL_CURVATURE_SLOPE
    // 긴 중앙 C2 연결의 원래 기하가 도로 검사를 통과하지 못할 때만,
    // 같은 map 뒷차축 시작/끝 위치·yaw·곡률을 잇는 별도 기하를 검사한다.
    // P'(0)=h0*tangent, P''(0)=k0*h0²*normal이므로 실제 초기 곡률은
    // 실측 k0 그대로다. quintic의 normal 방향 3차 계수를 0으로 만드는
    // 양의 h0 해를 사용해 시작점의 d(curvature)/ds를 0으로 맞춘다.
    // 끝점의 미분 조건은 원래 extent를 유지하며 속도/가속도를 덮지 않는다.
    // 해가 없으면 기존 후보 탐색을 계속한다. 새 기하는 같은 차체·도로·
    // 횡기울기 검사와 planSpeed의 전체 충돌·가감속·jerk 검사를 거친다.
    // GT ID나 특정 지도 위치를 쓰지 않는다. LiDAR/ROI를 통합할 때도
    // map 객체·예측 시각과 원래 허용 도로 계약을 그대로 공급해야 한다.
    double initial_extent=extent;
    if(zero_initial_curvature_slope) {
      const double turn=Geometry::wrap(end.yaw-ego.pose.yaw_rad);
      const double lateral=Geometry::dot(Geometry::sub(goal,ego.pose.position),Geometry::normal(ego.pose.yaw_rad));
      const double numerator=10*lateral-4*extent*std::sin(turn)+
          .5*terminal_curvature*extent*extent*std::cos(turn);
      const double denominator=1.5*ego.curvature_inv_m;
      if(denominator==0) return {};
      const double square=numerator/denominator;
      if(!std::isfinite(square) || square<=0) return {};
      initial_extent=std::sqrt(square);
      if(initial_extent==extent) return {};
    }
    const Vec2 c1=Geometry::mul(Geometry::unit(ego.pose.yaw_rad),initial_extent);
    const Vec2 c2=Geometry::mul(Geometry::normal(ego.pose.yaw_rad),ego.curvature_inv_m*initial_extent*initial_extent/2);
    const Vec2 delta=Geometry::sub(Geometry::sub(goal,ego.pose.position),Geometry::add(c1,c2));
    const Vec2 velocity=Geometry::sub(Geometry::mul(Geometry::unit(end.yaw),extent),
                                       Geometry::add(c1,Geometry::mul(c2,2)));
    const Vec2 acceleration=Geometry::sub(
        Geometry::mul(Geometry::normal(end.yaw),terminal_curvature*extent*extent),Geometry::mul(c2,2));
    const Vec2 c3=Geometry::add(Geometry::sub(Geometry::mul(delta,10),Geometry::mul(velocity,4)),Geometry::mul(acceleration,.5));
    const Vec2 c4=Geometry::sub(Geometry::add(Geometry::mul(delta,-15),Geometry::mul(velocity,7)),acceleration);
    const Vec2 c5=Geometry::add(Geometry::sub(Geometry::mul(delta,6),Geometry::mul(velocity,3)),Geometry::mul(acceleration,.5));
    const int count=std::max(2,static_cast<int>(std::ceil(extent/p_.output_step_m)));
    std::vector<PathPoint> out;
    for(int i=0;i<=count;++i) {
      const double u=static_cast<double>(i)/count;
      const auto nested=Geometry::add(c1,Geometry::mul(Geometry::add(c2,Geometry::mul(
          Geometry::add(c3,Geometry::mul(Geometry::add(c4,Geometry::mul(c5,u)),u)),u)),u));
      const auto position=Geometry::add(ego.pose.position,Geometry::mul(nested,u));
      const auto first=Geometry::add(c1,Geometry::mul(Geometry::add(Geometry::mul(c2,2),Geometry::mul(
          Geometry::add(Geometry::mul(c3,3),Geometry::mul(Geometry::add(Geometry::mul(c4,4),Geometry::mul(c5,5*u)),u)),u)),u));
      const auto second=Geometry::add(Geometry::mul(c2,2),Geometry::mul(Geometry::add(
          Geometry::mul(c3,6),Geometry::mul(Geometry::add(Geometry::mul(c4,12),Geometry::mul(c5,20*u)),u)),u));
      const double norm=Geometry::norm(first);
      if(norm<=1e-9) return {};
      const double progress=i==0 ? 0 : c.reference.project(position,origin.s,origin.s+extent).s-origin.s;
      if(!out.empty() && progress<=out.back().progress_m) return {};
      out.push_back({{position,std::atan2(first.y,first.x)},progress,progress,
                     Geometry::cross(first,second)/(norm*norm*norm)});
    }
    out.front().pose=ego.pose; out.front().curvature_inv_m=ego.curvature_inv_m;
    const Context geometric{c.reference,c.predictions,c.schedule,c.boundary,origin.s,c.prediction_offset,true,&c.boundary_index,c.native_region,c.proposal_timing};
    if(!lateralSlopeClear(geometric,out) || !checkPath(geometric,out)) return {};
    return out;
  }

  std::vector<PathPoint> cartesianPieces(const Context& c,const EgoState& ego,
                                        const std::vector<std::pair<double,double>>& goals,
                                        bool repair_initial_slope=false) const {
    double intervals=0;
    for(const auto& goal:goals) intervals+=std::max(2.0,std::ceil(goal.first/p_.output_step_m));
    if(intervals>600) return {};
    std::vector<PathPoint> shape;
    EgoState next=ego;
    for(const auto& goal:goals) {
      auto part=cartesianRecovery(c,next,goal.first,goal.second);
      if(part.empty() && repair_initial_slope)
        part=cartesianRecovery(c,next,goal.first,goal.second,true);
      if(part.empty()) return {};
      const double progress=shape.empty()?0:shape.back().progress_m;
      for(std::size_t i=shape.empty()?0:1;i<part.size();++i) {
        auto point=part[i];point.progress_m+=progress;point.time_sec=point.progress_m;
        shape.push_back(point);
      }
      next.pose=part.back().pose;next.curvature_inv_m=part.back().curvature_inv_m;
    }
    return shape;
  }

  std::vector<double> roadGoalOffsets(const Context& c,const RoadSample& end) const {
    const double lower=end.lower+p_.vehicle.width_m/2+p_.road_margin_m;
    const double upper=end.upper-p_.vehicle.width_m/2-p_.road_margin_m;
    if(lower>upper) return {};
    const auto intervals=roadLateralIntervals(c,end,end.yaw,lower,upper);
    std::vector<double> offsets;
    for(double l=std::ceil(lower/p_.lateral_step_m)*p_.lateral_step_m;
        l<=upper+1e-9;l+=p_.lateral_step_m) {
      offsets.push_back(l);
      if(offsets.size()>p_.max_lateral_samples) return {};
    }
    std::stable_sort(offsets.begin(),offsets.end(),[](double a,double b) {
      return std::abs(a)<std::abs(b);
    });
    offsets.erase(std::remove_if(offsets.begin(),offsets.end(),[&](double l) {
      return std::abs(l)<1e-8 || !std::any_of(intervals.begin(),intervals.end(),
          [&](const std::pair<double,double>& interval) {return l>=interval.first && l<=interval.second;});
    }),offsets.end());
    return offsets;
  }

  std::vector<PathPoint> matchInitialCurve(const Context& c,const EgoState& ego,
                                         const std::vector<PathPoint>& shape) const {
    const auto origin=c.reference.project(ego.pose.position,0,c.reference.length());
    const double h=std::min(10.0,shape.back().progress_m);
    auto end=std::lower_bound(shape.begin(),shape.end(),h,[](const PathPoint& q,double s){return q.progress_m<s;});
    if(end==shape.end()) end=shape.end()-1;
    const double extent=end->progress_m;
    const auto road=c.reference.at(origin.s+extent);
    const double l1=Geometry::dot(Geometry::sub(end->pose.position,road.position),Geometry::normal(road.yaw));
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
    const Context geometric{c.reference,c.predictions,c.schedule,c.boundary,origin.s,c.prediction_offset,true,&c.boundary_index,c.native_region,c.proposal_timing};
    if(!checkPath(geometric,out)) return {};
    return out;
  }
  double followingSpeedTarget(const Context& c,const Pose2& pose,double time,double cap) const {
    // #LEAD_VEHICLE_HEADWAY
    // 고정 0.25m 충돌 여유만으로 속도를 최적화하면 느린 앞차의 범퍼까지
    // 접근하는 계획이 선택된다. map 경로상의 각 예정 자세에서 같은 방향의
    // 차량 예측을 조회하고, 실제 차체의 횡 투영이 겹치는 앞차만 추종한다.
    // 뒷차축→앞범퍼와 상대 OBB의 전방 투영을 빼 실제 범퍼 간격을 구한다.
    // 목표속도는 (간격-정지여유)/추종시간 이하로 낮춘다. 옆 차선·뒤차·
    // 교차 차량은 이 추종 목표에서 제외하며 원래 시간별 충돌 검사가 담당한다.
    // 이는 기존 속도 비용의 목표값이지 새 물리 속도 제약이 아니다. 따라서
    // 현재 속도를 순간 삭제하지 않고 원래 가감속/jerk 한도에서 감속한다.
    // headway는 추종 여유이며 급제동·모델 오차에 대한 안전 보증은 아니다.
    // ROI는 앞차를 임의 제거하면 안 되며 bbox와 예측 시각 계약을 유지한다.
    // #FOLLOWING_PREDICTION_REUSE
    // 같은 Context 안의 속도 후보들은 동일한 예측 배열·시각을 사용한다.
    // 후보마다 모든 객체의 같은 보간·OBB·방향 계산을 반복하지 않고 시각별
    // 불변 결과만 재사용한다. 객체 순서와 원래 추종/충돌 계산식은 유지한다.
    // 자차 자세와 cap은 후보마다 다르므로 목표속도 자체는 저장하지 않는다.
    // 캐시는 이번 계산의 Context와 함께 폐기해 다음 관측/ROI 장면에 섞이지 않는다.
    auto found=c.following_objects.find(time);
    if(found==c.following_objects.end()) {
      std::vector<FollowingObject> objects;
      for(std::size_t j=0;j<c.predictions.current.size();++j) {
        if(c.predictions.current[j].kind!=ObjectKind::Vehicle) continue;
        const auto object=samplePrediction(c.predictions,j,c.prediction_offset+time);
        objects.push_back({Geometry::objectBox(object,p_.obstacle_margin_m),
                           Geometry::unit(object.pose.yaw_rad)});
      }
      found=c.following_objects.emplace(time,std::move(objects)).first;
    }
    const auto forward=Geometry::unit(pose.yaw_rad),left=Geometry::normal(pose.yaw_rad);
    for(const auto& object:found->second) {
      if(Geometry::dot(forward,object.heading)<=.5) continue;
      const auto delta=Geometry::sub(object.box.pose.position,pose.position);
      const double longitudinal=Geometry::dot(delta,forward);
      if(longitudinal<=0) continue;
      const auto& box=object.box;
      if(std::abs(Geometry::dot(delta,left))>
         p_.vehicle.width_m/2+Geometry::extent(box,left)) continue;
      const double gap=longitudinal-p_.vehicle.front_m-Geometry::extent(box,forward);
      cap=std::min(cap,std::max(0.0,(gap-p_.following_min_gap_m)/p_.following_time_headway_sec));
    }
    return cap;
  }
  double edgeCost(const Context& c,const ArcPath& arc,const SpeedEdge& edge,double origin_s,double cap) const {
    // Four-point Gauss-Legendre is exact for the polynomial dynamics terms.
    // Reference and moving-lead targets use the same numerical quadrature in
    // speed search and final candidate ranking.
    const double nodes[]{.06943184420297371,.33000947820757187,.66999052179242813,.93056815579702629};
    const double weights[]{.17392742256872693,.32607257743127307,.32607257743127307,.17392742256872693};
    double cost=-.2*edge.distance(edge.h);
    for(int i=0;i<4;++i) {
      const double t=nodes[i]*edge.h;
      const auto q=arc.at(edge.s0+edge.distance(t));
      const auto r=c.reference.at(origin_s+q.progress_m);
      const double l=Geometry::dot(Geometry::sub(q.pose.position,r.position),Geometry::normal(r.yaw));
      const double following_target=followingSpeedTarget(c,q.pose,edge.t0+t,cap);
      double target=following_target;
      // #CURVE_TRACKING_RESERVE
      // R53은 계획 1.91m/s²와 작은 속도/곡률 추종 오차가 합쳐 실측
      // 2.03m/s²가 되었고, 실제 초기 조건이 기존 2.0 한도를 벗어났다.
      // 설정한 선호 횡가속도를 곡률별 속도 비용 목표로 사용해 실행 여유를 둔다.
      // 목표보다 빠른 실측 초기 상태는 보존한다. 이것은 완화된 제약이 아니며
      // 실제 도로·충돌·속도·곡률·횡가속도·가감속·jerk 최종 검사는 그대로다.
      // ROI/좌표·제어 게인과 무관하며 0 설정은 기존 비용 목표를 유지한다.
      // #REPRESENTED_CURVATURE_COST
      // 최종 검사는 명시 곡률과 위치/방향 보간이 만드는 실제 ArcPath 곡률을
      // 함께 제한한다. 비용도 같은 현재 구간 상한을 써서 명시 곡률만 작을 때
      // 선호 여유를 잃지 않게 한다. 앞으로 보는 별도 창·계수·속도 가산은 없으며
      // 실측 초기 상태, 최종 hard 한도와 전체 충돌/도로 검사는 유지한다.
      // clamp는 부동소수점 적분 잔차를 곡률 조회의 [0,length] 안으로만 제한한다.
      if(p_.preferred_lateral_accel_mps2>0) {
        const double station=Geometry::clamp(edge.s0+edge.distance(t),0.0,arc.length());
        const double goal_curvature=arc.curvature(station,station);
        if(goal_curvature>1e-12)
          target=std::min(target,std::sqrt(p_.preferred_lateral_accel_mps2/goal_curvature));
      }
      // #STOPPED_LEAD_PROGRESS_COST
      // 앞차 때문에 낮아진 목표속도에는 기존 전진 거리 보상을 상쇄한다.
      // 그렇지 않으면 목표가 0이어도 -0.2*v 보상이 0.03m/s 잔류 속도를
      // 유지시키고, 매 재계획마다 정지를 뒤로 미뤄 앞차에 계속 접근한다.
      // 실제 초기 v/a는 보존하고 원래 jerk/제동 한도에서 정지한다.
      // 앞차가 멀어져 추종 목표가 올라가면 다시 출발할 수 있으며,
      // 추종 제한이 없는 구간의 원래 비용은 그대로 유지한다.
      const double following_progress_compensation=target<cap ? .2*edge.speed(t) : 0;
      // #CURVE_TARGET_BASELINE
      // 위치별 목표의 제곱을 그대로 비교하면 낮은 목표 구간에 기다리는
      // 경로가 더 싸질 수 있다. 기존 앞차 추종 비용을 기준으로 곡선 보정
      // 2*v*(following_target-target)만 더한다. 같은 위치에서 비용의 최소는
      // 여전히 target이며, 정지(v=0) 비용은 곡선 목표와 무관해 대기를
      // 보상하지 않는다. 선호 목표가 0이면 기존 비용과 정확히 같고 기존
      // 앞차 추종·충돌 검사와 모든 물리 제약은 유지한다.
      const double curve_cost=2*edge.speed(t)*(following_target-target);
      cost+=edge.h*weights[i]*(following_progress_compensation+
        p_.speed_tracking_weight*(std::pow(edge.speed(t)-following_target,2)+curve_cost)+
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
         v>ceiling+1e-8 || a>p_.max_acceleration_mps2+1e-8 ||
         a<-p_.max_deceleration_mps2-1e-8 || std::abs(j)>p_.max_longitudinal_jerk_mps3+1e-8 ||
         (v*v*curvature>p_.max_lateral_accel_mps2+1e-6 &&
          !recoveringLateralExcess(ego,curvature,recovery))) return false;
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

  std::vector<PathPoint> sampleEdge(const ArcPath& arc,const SpeedEdge& edge,
                                    bool for_output=false) const {
    std::vector<double> times{0,edge.h};
    const double end=edge.s0+edge.distance(edge.h);
    // #OUTPUT_KNOT_ROUNDTRIP
    // map 공간 knot와 속도 phase 끝이 약 0.0001초 차이면 출력 v/a를 다시
    // Hermite 식에 넣을 때 반올림만으로 jerk 4가 4.00000007처럼 계산된다.
    // 최종 출력에만 phase 끝 1ms 안의 공간 knot를 생략해 이 퇴화를 피한다.
    // 4ms보다 짧은 phase는 그 길이의 1/4까지만 적용하며 phase 끝 자체와
    // 실측 초기 v/a는 유지한다. 탐색의 충돌·횡가속도 표본은 원래 그대로다.
    // 새 출력 chord는 기존 전체 도로·swept 충돌·곡률·가감속·jerk와 거리
    // 재검사를 통과해야 한다. 한도·오차 허용값·경로 시각은 바꾸지 않는다.
    // 이후 ROI/차선 경계가 바뀌어도 이 최종 재검사를 생략하면 안 된다.
    // #EXACT_DYADIC_KNOT_TIME
    // 보통의 0.5초 속도 격자에서 공간 knot마다 45회 이분 탐색하면 합류
    // 계산이 입력 유효 시간보다 길어진다. h가 2의 거듭제곱이고 속도가
    // 전체 구간에서 0.5m/s보다 클 때만 Newton으로 탐색 위치를 추측한다.
    // 이 추측 자체는 결과로 쓰지 않는다. 원래 45회 이분 탐색의 동일한
    // 2^45 정수 격자에서 원래 distance 비교로 양 끝을 확인·확장한 뒤
    // 경계를 찾아 같은 중점을 반환한다. 따라서 허용 오차·표본 시각·차체
    // sweep·충돌 검사를 바꾸지 않는다. 정지 근처와 비 dyadic 시간 구간은
    // 원래 45회 계산을 그대로 사용하며 지도/GT/ROI 값과 무관하다.
    int exponent=0;
    const auto extrema=edge.extrema();
    const bool exact_grid=std::frexp(edge.h,&exponent)==.5 &&
        std::all_of(extrema.begin(),extrema.end(),[&](double t){return edge.speed(t)>.5;});
    const auto time_at_knot=[&](double target) {
      if(!exact_grid) {
        double lo=0,hi=edge.h;
        for(int k=0;k<45;++k) {
          const double mid=(lo+hi)/2;
          if(edge.s0+edge.distance(mid)<target)lo=mid;else hi=mid;
        }
        return (lo+hi)/2;
      }
      const std::uint64_t count=std::uint64_t{1}<<45;
      const double tick=std::ldexp(edge.h,-45);
      double guess=edge.h*(target-edge.s0)/edge.distance(edge.h);
      for(int k=0;k<5;++k) {
        const double next=guess+(target-(edge.s0+edge.distance(guess)))/edge.speed(guess);
        if(!(next>0 && next<edge.h) || next==guess)break;
        guess=next;
      }
      auto lo=static_cast<std::uint64_t>(std::max(0.0,std::min(double(count),guess/tick)));
      auto hi=lo;
      const auto before=[&](std::uint64_t index){return edge.s0+edge.distance(index*tick)<target;};
      for(std::uint64_t step=1;!before(lo);step*=2)lo=lo>step?lo-step:0;
      for(std::uint64_t step=1;before(hi);step*=2)hi=std::min(count,hi+step);
      while(hi-lo>1) {
        const auto mid=lo+(hi-lo)/2;
        if(before(mid))lo=mid;else hi=mid;
      }
      return (lo+hi)*tick/2;
    };
    auto knot=std::upper_bound(arc.interpolation_knots.begin(),arc.interpolation_knots.end(),edge.s0+1e-10);
    for(;knot!=arc.interpolation_knots.end() && *knot<end-1e-10;++knot) {
      const double knot_time_sec=time_at_knot(*knot);
      const double minimum_span_sec=std::min(.001,edge.h/4);
      if(!for_output || (knot_time_sec>=minimum_span_sec && edge.h-knot_time_sec>=minimum_span_sec))
        times.push_back(knot_time_sec);
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
  // #COLLISION_BROAD_PHASE
  // 객체를 삭제하거나 가까운 ROI로 자르지 않는다. 예측 전체의 중심 범위와
  // bbox/오차, 기존 swept SAT의 최대 회전 팽창까지 감싼 바깥 범위만 비교한다.
  // 완전히 떨어진 경우에만 후보의 고비용 시간 샘플링·SAT를 생략한다.
  // 범위가 겹치면 보행자·교차 차량·회전 객체 모두 기존 samplePrediction과
  // sweptOverlap으로 검사한다. 좌표가 크거나 계산이 불확실하면 정밀 경로로 간다.
  double egoCollisionRadius(double motion_pad,double angle) const {
    const double offset=std::abs(p_.vehicle.front_m-p_.vehicle.rear_m)/2;
    const double half_length=(p_.vehicle.front_m+p_.vehicle.rear_m)/2;
    const double half_width=p_.vehicle.width_m/2;
    const double center_arc_pad=offset*angle*angle/8;
    const double rotation_factor=1+std::sqrt(2.0)*angle/2;
    return offset+(std::hypot(half_length,half_width)+
        std::sqrt(2.0)*(center_arc_pad+motion_pad))*rotation_factor;
  }
  bool potentialObjectCollision(const ObjectBounds& object,const Geometry::Bounds& rear_axle,
                                double ego_radius) const {
    const double rotation_factor=1+std::sqrt(2.0)*object.rotation_delta/2;
    const double object_radius=std::hypot(object.half_length+p_.obstacle_margin_m,
                                         object.half_width+p_.obstacle_margin_m)*rotation_factor;
    return !rear_axle.separated(object.centers,ego_radius+object_radius);
  }
  bool potentialCollision(const Context& c,const Geometry::Bounds& rear_axle,double motion_pad) const {
    const double radius=egoCollisionRadius(motion_pad,Geometry::kPi);
    for(const auto& object:c.object_bounds)
      if(potentialObjectCollision(object,rear_axle,radius))return true;
    return false;
  }
  std::vector<std::size_t> collisionObjectIndices(const Context& c,const Geometry::Bounds& rear_axle,
                                                 double motion_pad,double angle,
                                                 double time0=std::numeric_limits<double>::quiet_NaN(),
                                                 double time1=std::numeric_limits<double>::quiet_NaN(),
                                                 double ego_yaw=std::numeric_limits<double>::quiet_NaN()) const {
    std::vector<std::size_t> indices;const double radius=egoCollisionRadius(motion_pad,angle);
    for(std::size_t j=0;j<c.object_bounds.size();++j) {
      if(!potentialObjectCollision(c.object_bounds[j],rear_axle,radius))continue;
      // #TEMPORAL_COLLISION_BOUNDS
      // 5초 예측 전체에서 차선에 도달하는 뒤차를 모든 짧은 속도 edge에서
      // 정밀 검사하면 시간 표본 생성이 반복되어 제어 입력이 만료된다.
      // map 객체의 중심은 기존 samplePrediction과 같은 knot 사이 선형
      // 보간이므로 검사 시간의 양 끝과 그 사이 모든 예측 knot를 감싼다.
      // bbox 크기·불확실성·가속도 보간 여유·회전 팽창은 전체 예측의 기존
      // 상한을 그대로 사용한다. 시각 범위가 불확실하면 전체 범위로 검사한다.
      // 이 바깥 범위까지 분리된 객체만 생략하고, 겹치는 뒤차·횡단 객체는
      // 원래 45회 시간 역산·swept SAT를 수행한다. ROI나 안전 한도는 바꾸지 않는다.
      auto bounded=c.object_bounds[j];
      const double horizon=c.predictions.step_sec*kPredictionSteps;
      if(std::isfinite(time0) && std::isfinite(time1) && time0>=0 &&
         time1>=time0 && time1<=horizon) {
        const auto center=[&](double time) {
          const double scaled=time/c.predictions.step_sec;
          const auto lo=std::min(static_cast<std::size_t>(scaled),kPredictionSteps-1);
          const auto& a=(lo==0?c.predictions.current:c.predictions.future[lo-1])[j];
          const auto& b=c.predictions.future[lo][j];
          return Geometry::lerp(a.pose.position,b.pose.position,scaled-lo);
        };
        bounded.centers=Geometry::Bounds{};
        bounded.centers.include(center(time0));bounded.centers.include(center(time1));
        for(std::size_t k=1;k<kPredictionSteps;++k) {
          const double time=k*c.predictions.step_sec;
          if(time>time0 && time<time1)bounded.centers.include(c.predictions.future[k-1][j].pose.position);
        }
      }
      // #AXIS_ALIGNED_SWEEP_BOUNDS
      // 평행 차선 차량은 외접원끼리 겹쳐도 실제 폭 방향으로 분리될 수 있다.
      // map X/Y별 반길이를 사용하되 기존 swept SAT의 회전 팽창까지 감싼다.
      // yaw 변화에 따른 축 투영 차이는 외접 반지름×각도 이하이고, SAT가
      // 각 반길이에 더하는 회전 여유의 축 투영은 반지름×각도/√2 이하다.
      // 자차는 비대칭 앞뒤 길이의 큰 값, 중심 회전·속도 보간 여유를 포함한다.
      // 객체는 전체 예측의 최대 크기·오차와 모든 knot의 누적 회전 상한을
      // 유지한다. 두 축 중 완전히 분리될 때만 생략하고 정밀 검사식은 그대로다.
      if(std::isfinite(ego_yaw)) {
        const double length=std::max(p_.vehicle.front_m,p_.vehicle.rear_m),width=p_.vehicle.width_m/2;
        const double r=std::hypot(length,width);
        const double delta=std::abs(p_.vehicle.front_m-p_.vehicle.rear_m)/2;
        const double inflation=motion_pad+delta*angle*angle/8;
        const double padding=r*angle*(1+1/std::sqrt(2.0))+inflation*(std::sqrt(2.0)+angle);
        const double cx=std::abs(std::cos(ego_yaw)),sy=std::abs(std::sin(ego_yaw));
        Geometry::Bounds ego=rear_axle;
        const double ex=length*cx+width*sy+padding,ey=length*sy+width*cx+padding;
        ego.min_x-=ex;ego.max_x+=ex;ego.min_y-=ey;ego.max_y+=ey;
        const double object_length=bounded.half_length+p_.obstacle_margin_m;
        const double object_width=bounded.half_width+p_.obstacle_margin_m;
        const double rotation=std::min(Geometry::kPi,kPredictionSteps*bounded.rotation_delta);
        const double object_padding=std::hypot(object_length,object_width)*
            (rotation+bounded.rotation_delta/std::sqrt(2.0));
        const double yaw=c.predictions.current[j].pose.yaw_rad;
        const double ox=object_length*std::abs(std::cos(yaw))+object_width*std::abs(std::sin(yaw))+object_padding;
        const double oy=object_length*std::abs(std::sin(yaw))+object_width*std::abs(std::cos(yaw))+object_padding;
        auto object=bounded.centers;
        object.min_x-=ox;object.max_x+=ox;object.min_y-=oy;object.max_y+=oy;
        if(ego.separated(object,0))continue;
      }
      if(potentialObjectCollision(bounded,rear_axle,radius))indices.push_back(j);
    }
    return indices;
  }

  // #REENTRY_SPEED_PREFIX_PRUNE
  // map 도로 복귀의 기존 바깥 면적·연속 차체 경계 조건을 같은 시간 표본에서 검사한다.
  // 최종 검사에만 두면 이미 실패한 초기 이동을 가진 속도 beam 수천 개를 끝까지
  // 계산해 경로가 수 초 늦게 도착한다. 속도 edge의 실제 출력 표본을 먼저 검사해
  // 뒤의 어떤 속도 선택으로도 고칠 수 없는 prefix만 제외한다. 도로/객체/좌표와
  // 한도는 유지하며, 초기 대기와 아직 복귀 중인 prefix는 허용한다. require_return은
  // 완성한 경로에만 적용해 기존 완전 복귀 조건과 전체 최종 검사를 그대로 둔다.
  // LiDAR/GT 여부에 관계없이 같은 map 차체·도로 계약을 사용한다.
  bool reentryProgressClear(const Context& c,const std::vector<PathPoint>& path,
                            bool require_return,bool physical_proven=false) const {
    if(c.native_region && c.native_region->recovery_original) {
      const auto& original=*c.native_region->recovery_original;
      // 복귀하지 못한 말단은 비싼 시간별 충돌 검사 전에 같은 조건으로 거부한다.
      if(require_return && path.back().progress_m>1e-8 && !original.containsBox(
          Geometry::sweptEgoEnvelope(path.back().pose,path.back().pose,p_.vehicle,p_.road_margin_m)))return false;
      // #PHYSICAL_ROAD_MARGIN_RECOVERY
      // 실제 차체가 도로 안인데 5cm 계획 여유만 이미 소모된 상태를 구분한다.
      // 실측 자세/곡률을 보존하면 첫 이동에서 가상 여유 침범은 잠시 커질 수
      // 있다. 이때는 실제 차체의 도로 밖 면적 0과 각 구간의 연속 sweep을
      // 원래 map polygon으로 검사한다. 정상 주행의 여유는 그대로며, 완성한
      // 경로는 기존 horizon_m 안에서 전체 여유를 되찾고 다시 잃지 않아야 한다.
      // 처음부터 실제 차체가 밖이면 기존 침범 면적 단조 감소 조건을 유지한다.
      // 객체/동역학/차선 방향 검사는 생략하지 않으며 GT 전용 정보가 없다.
      const bool margin_only=c.native_region->margin_only_recovery;
      double previous=c.native_region->initial_outside_area_m2;
      bool returned=false;
      for(std::size_t i=0;i<path.size();++i) {
        const auto full_margin=Geometry::sweptEgoEnvelope(path[i].pose,path[i].pose,p_.vehicle,p_.road_margin_m);
        const auto box=margin_only ? Geometry::egoBox(path[i].pose,p_.vehicle) : full_margin;
        const double outside=margin_only && physical_proven ? 0 : c.native_region->reentryOutsideArea(box);
        if(outside>previous+1e-7)return false;
        previous=outside;
        if(margin_only && !physical_proven && i>0 && !roadClearWithMargin(c,path[i-1].pose,path[i].pose,0,0,&original))return false;
        if(returned && i>0 && !roadClearWithMargin(c,path[i-1].pose,path[i].pose,p_.road_margin_m,0,&original))return false;
        if(original.containsBox(full_margin))returned=true;
        if(!returned && path[i].progress_m>p_.horizon_m)return false;
      }
      if(require_return && !returned && path.back().progress_m>1e-8)return false;
    }
    return true;
  }

  bool edgeClear(const Context& c,const ArcPath& arc,const SpeedEdge& edge,bool physical_proven=false) const {
    if(c.native_region && c.native_region->recovery_original &&
       !reentryProgressClear(c,sampleEdge(arc,edge,true),false,physical_proven))return false;
    return collisionEdgeClear(c,arc,edge);
  }
  // 최종 경로의 복귀 검사는 원래 출력 표본에서 이미 수행했다. 여기서는
  // 기존 시간별 충돌 검사만 수행해 추가 재표본으로 허용 조건을 바꾸지 않는다.
  bool collisionEdgeClear(const Context& c,const ArcPath& arc,const SpeedEdge& edge) const {
    if(c.predictions.current.empty()) return true;
    double max_acceleration=0,min_speed=std::numeric_limits<double>::infinity();
    for(double t:edge.extrema()) {
      max_acceleration=std::max(max_acceleration,std::abs(edge.acceleration(t)));
      min_speed=std::min(min_speed,edge.speed(t));
    }
    const double radius=std::hypot(std::max(p_.vehicle.front_m,p_.vehicle.rear_m),p_.vehicle.width_m/2);
    // 각 시간 구간의 각도/거리 비는 선형 arc의 상한과 반올림 오차로 감싼다.
    // 기존 d>1e-12 분기와 같은 최소 분모를 사용한다. 큰 좌표의 오차는
    // 범위를 넓혀 공간 선별의 이득만 줄이며, 충돌 검사 범위를 줄이지 않는다.
    const double scale=std::max({1.0,std::abs(arc.bounds.min_x),std::abs(arc.bounds.min_y),
                                std::abs(arc.bounds.max_x),std::abs(arc.bounds.max_y)});
    const double error=128*std::numeric_limits<double>::epsilon();
    const double heading_bound=arc.heading_per_meter+
        (arc.heading_per_meter*error*scale+error*arc.yaw_scale)/1e-12;
    const double motion_bound=max_acceleration*edge.h*edge.h/8*(1+radius*heading_bound);
    // #SPEED_EDGE_BOUNDS
    // 짧은 속도 edge와 관계없는 전체 경로를 사용하면 멀리 있는 장애물 때문에도
    // 매 후보마다 정밀 SAT가 반복된다. 비음수 속도 edge의 실제 arc 시작·끝과
    // 그 사이 모든 꺾임점을 포함한다. 선형 arc 보간은 이 AABB 밖으로 나가지
    // 않으며 곡선 중간 돌출도 빠지지 않는다. 부호가 바뀌는 비단조 구간은 기존
    // 전체 범위를 유지한다. 객체 예측, 회전 팽창 및 정밀 시간 샘플은 유지한다.
    Geometry::Bounds edge_bounds=arc.bounds;
    if(min_speed>=0) {
      edge_bounds=Geometry::Bounds{};
      const double end=edge.s0+edge.distance(edge.h);
      edge_bounds.include(arc.at(edge.s0).pose.position);
      edge_bounds.include(arc.at(end).pose.position);
      auto knot=std::upper_bound(arc.station.begin(),arc.station.end(),edge.s0);
      for(;knot!=arc.station.end() && *knot<end;++knot)
        edge_bounds.include(arc.points[static_cast<std::size_t>(knot-arc.station.begin())].pose.position);
    }
    // 같은 보수적 선별을 객체별로 한 번 적용해, 이미 분리된 먼 객체의 SAT를
    // 모든 시간 표본에서 반복하지 않는다. 원본 객체 배열과 순서는 보존한다.
    // #ROTATION_BOUNDS
    // swept SAT가 각 표본에 부여하는 회전 팽창도 선별 범위에 포함한다. 매번
    // 최악의 180도 회전을 가정하는 대신 해당 arc의 최대 yaw/거리 비와 이동
    // 거리로 감싸고, 객체는 예측 knot 사이 최대 회전으로 감싼다. knot에서
    // 시간 구간을 나누는 원래 정밀 검사와 반올림 여유는 그대로 유지한다.
    const double angle=std::min(Geometry::kPi,arc.heading_per_meter*
        std::abs(edge.distance(edge.h))+error*arc.yaw_scale+1e-10);
    const auto indices=collisionObjectIndices(c,edge_bounds,motion_bound,min_speed>=0?angle:Geometry::kPi,
        c.prediction_offset+edge.t0,c.prediction_offset+edge.t0+edge.h,arc.at(edge.s0).pose.yaw_rad);
    if(indices.empty())return true;
    return edgeClearNarrow(c,arc,edge,&indices);
  }
  bool edgeClearNarrow(const Context& c,const ArcPath& arc,const SpeedEdge& edge,
                       const std::vector<std::size_t>* indices=nullptr) const {
    if(c.predictions.current.empty()) return true;
    double max_acceleration=0;
    for(double t:edge.extrema()) max_acceleration=std::max(max_acceleration,std::abs(edge.acceleration(t)));
    const double radius=std::hypot(std::max(p_.vehicle.front_m,p_.vehicle.rear_m),p_.vehicle.width_m/2);
    const auto path=sampleEdge(arc,edge);
    for(std::size_t i=1;i<path.size();++i) {
      const auto& a=path[i-1]; const auto& b=path[i];
      const double dt=b.time_sec-a.time_sec;
      const double d=Geometry::norm(Geometry::sub(b.pose.position,a.pose.position));
      const double angle=std::abs(Geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad));
      // |a| dt²/8 bounds cubic-speed position deviation from a linear time
      // chord. Inflate for both translation and yaw's arc/time discrepancy.
      const double pad=max_acceleration*dt*dt/8*(1+(d>1e-12 ? radius*angle/d : 0));
      if(collisionNarrow(c,a,b,pad,indices)) return false;
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
  double staticStopArc(const Context& c,const ArcPath& arc,
                       double* clearance_progress=nullptr) const {
    if(clearance_progress) *clearance_progress=0;
    double stop=std::numeric_limits<double>::infinity();
    for(std::size_t j=0;j<c.predictions.current.size();++j) {
      const auto& o=c.predictions.current[j];
      bool stationary=Geometry::norm(o.velocity_mps)<1e-10;
      for(const auto& slice:c.predictions.future)
        stationary=stationary && Geometry::norm(Geometry::sub(slice[j].pose.position,o.pose.position))<1e-10 &&
                   std::abs(Geometry::wrap(slice[j].pose.yaw_rad-o.pose.yaw_rad))<1e-10;
      if(!stationary) continue;
      // Include the largest supplied prediction uncertainty, not just t=0.
      auto inflated=o;
      for(const auto& slice:c.predictions.future)
        inflated.radius_error_m=std::max(inflated.radius_error_m,slice[j].radius_error_m);
      const auto other=Geometry::objectBox(inflated,p_.obstacle_margin_m);
      const auto overlaps=[&](Pose2 a,Pose2 b) {
        const auto envelope=Geometry::sweptEgoEnvelope(a,b,p_.vehicle,0);
        return Geometry::sweptOverlap(envelope,envelope,other,other);
      };
      if(overlaps(arc.points.front().pose,arc.points.front().pose)) return -1;
      for(std::size_t i=1;i<arc.points.size() && arc.station[i-1]<stop;++i) {
        if(!overlaps(arc.points[i-1].pose,arc.points[i].pose)) continue;
        double lo=arc.station[i-1],hi=arc.station[i];
        for(int k=0;k<40;++k) {
          const double mid=(lo+hi)/2;
          if(overlaps(arc.points[i-1].pose,arc.at(mid).pose)) hi=mid; else lo=mid;
        }
        if(lo-1e-5<stop) {
          stop=lo-1e-5;
          if(clearance_progress) {
            const double origin=c.reference.project(arc.points.front().pose.position,0,c.reference.length()).s;
            double station=origin;
            for(const auto& corner:Geometry::corners(other))
              station=std::max(station,c.reference.project(corner,origin,
                  std::min(c.reference.length(),origin+p_.trajectory_horizon_m)).s);
            *clearance_progress=station-origin+p_.vehicle.rear_m+p_.road_margin_m;
          }
        }
        break;
      }
    }
    return stop;
  }

  bool lateralAccelerationClear(const ArcPath& arc,const SpeedEdge& edge) const {
    return lateralAccelerationClear(arc,edge,EgoState{},CeilingRecovery{});
  }
  bool lateralAccelerationClear(const ArcPath& arc,const SpeedEdge& edge,
                                const EgoState& ego,const CeilingRecovery& recovery) const {
    // #TIME_ALIGNED_LATERAL_ACCELERATION
    // 0.5초 edge 앞의 높은 속도와 뒤의 큰 곡률을 곱하면, 그 사이 감속한
    // 정상 곡선 진입까지 거부할 수 있다. 전체 edge 상한이 실패할 때만
    // 기존 출력의 기하 knot·시간 분할로 같은 시간의 속도와 공간 곡률을 묶는다.
    // 각 작은 구간에서도 속도 다항식의 모든 극값과 arc 곡률 상한을 사용해
    // 연속 구간 전체를 보수적으로 검사한다. 표본점에서만 검사하지 않는다.
    // 측정 초기 상태, 횡가속도 2m/s²와 기존 최종 경로 재검사는 유지한다.
    const auto samples=sampleEdge(arc,edge);
    const auto extrema=edge.extrema();
    // #LATERAL_BOUND_REFINEMENT
    // 감속과 곡률 증가가 겹치면 출력 knot 사이에서도 서로 다른 시각의
    // 최대값 곱이 실제 횡가속도를 과대평가한다. 상한이 한도에 걸린 구간만
    // 최대 6회 이분한다. 각 자식 구간의 속도 다항식 극값과 원래 arc 곡률
    // 상한으로 전체 시간을 다시 증명하며, 점 검사만으로 통과시키지 않는다.
    // 실제 끝점 위반 또는 분할 예산 안에 입증하지 못한 구간은 계속 거부한다.
    // map/뒷차축 기준·경로 표본·충돌/도로 검사·계획 한도는 바꾸지 않는다.
    const auto clear=[&](const auto& self,double lo,double hi,int depth)->bool {
      double speed=std::max(std::abs(edge.speed(lo)),std::abs(edge.speed(hi)));
      for(double t:extrema)
        if(t>lo && t<hi) speed=std::max(speed,std::abs(edge.speed(t)));
      const double start=edge.s0+edge.distance(lo),end=edge.s0+edge.distance(hi);
      const double curvature=arc.curvature(start,end);
      if(!std::isfinite(speed) || !std::isfinite(curvature)) return false;
      const double limit=p_.max_lateral_accel_mps2+1e-6;
      if(speed*speed*curvature<=limit) return true;
      if(recoveringLateralExcess(ego,curvature,recovery) && belowRecovery(edge,recovery))return true;
      if(depth>=6 ||
         std::pow(edge.speed(lo),2)*std::abs(arc.at(start).curvature_inv_m)>limit ||
         std::pow(edge.speed(hi),2)*std::abs(arc.at(end).curvature_inv_m)>limit) return false;
      const double mid=(lo+hi)/2;
      return self(self,lo,mid,depth+1) && self(self,mid,hi,depth+1);
    };
    for(std::size_t i=1;i<samples.size();++i) {
      if(!clear(clear,samples[i-1].time_sec-edge.t0,samples[i].time_sec-edge.t0,0)) return false;
    }
    return true;
  }

  std::vector<PathPoint> planSpeed(const Context& c,const ArcPath& arc,const EgoState& ego,
                                 const PlanningConstraints& limits,double duration,double stop_arc,
                                 const std::vector<ReferencePoint>& reference_map) const {
    auto start=arc.points.front(); start.time_sec=0;
    if(!roadClear(c,start.pose,start.pose) || collision(c,start,start)) return {};
    const double cap=std::min(p_.max_speed_mps,limits.maximum_speed_mps);
    const double origin_s=c.reference.project(start.pose.position,0,c.reference.length()).s;
    const auto recovery=ceilingRecovery(ego,limits);
    const double persistent_stop=std::min(staticStopArc(c,arc),
                                         limits.release_time_sec>=duration ? stop_arc :
                                         std::numeric_limits<double>::infinity());
    const double available_space=std::min(persistent_stop,arc.length());
    // #MEASURED_SPEED_RECOVERY
    // 상한을 조금 넘은 실측 초기 속도를 버리면 경로가 비고 제어 연결이 끊긴다.
    // 초기 속도와 양의 초기 가속도에서 피할 수 없는 peak까지만 후보 범위를
    // 유지한다. speedAdmissible과 기존 ceilingRecovery가 최대 jerk로 가속을
    // 줄이고 상한으로 복귀하도록 전체 다항식을 검사한다. 상한 위의 신규
    // 가속·속도 목표를 허용하지 않으며 도로·충돌·가감속 한도는 유지한다.
    const double recovery_peak=std::max(p_.max_speed_mps,ego.speed_mps+
        std::pow(std::max(0.0,ego.acceleration_mps2),2)/(2*p_.max_longitudinal_jerk_mps3));
    if(available_space<0 || brakingDistance(ego.speed_mps,ego.acceleration_mps2)>available_space) return {};
    // #REACHABLE_TERMINAL_STOP
    // 최고 속도에 즉시 도달한다고 가정하면 현재 속도로 5초 안에 닿을 수
    // 없는 먼 장애물도 종료 정지를 강제해 앞의 합법적 회피 후보를 버린다.
    // 실측 속도에서 최대 가속도를 즉시 적용한 거리조차 못 미치는 경우만
    // 종료 정지 강제에서 제외한다. jerk·속도/곡률 한도는 이 도달 상한을
    // 더 줄이므로 과소평가하지 않는다. 모든 상태의 s+brakingDistance 검사와
    // 실제 장애물 앞 정지 여유, 시간별 충돌 및 최종 검사는 그대로 유지한다.
    const double reachable_upper=ego.speed_mps*duration+
        .5*p_.max_acceleration_mps2*duration*duration;
    const bool finish_stopped=cap==0 || persistent_stop<reachable_upper;
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
    // #REENTRY_GEOMETRY_PROOF
    // 속도 후보가 달라도 ArcPath의 구간별 선형 위치/yaw 보간은 같다.
    // 여유만 소모된 복귀에서 원래 도로가 전체 차체 sweep을 포함함을 먼저
    // 증명하면 모든 부분 이동도 물리 도로 안에 있다. 입증하지 못하면 기존
    // 후보별 검사를 그대로 수행한다. 증명은 이 planSpeed 호출에만 유효하며
    // 원래 최종 전체 경로 검사와 객체/시간/동역학 검사는 유지한다.
    bool physical_proven=false;
    if(c.native_region && c.native_region->margin_only_recovery) {
      const auto& original=*c.native_region->recovery_original;
      physical_proven=original.containsBox(Geometry::egoBox(arc.points.front().pose,p_.vehicle));
      for(std::size_t i=1;i<arc.points.size() && physical_proven;++i)
        physical_proven=roadClearWithMargin(c,arc.points[i-1].pose,arc.points[i].pose,0,0,&original);
    }
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
        std::vector<double> accelerations{a_lo,a_hi,Geometry::clamp(0.0,a_lo,a_hi),
          Geometry::clamp(-p_.max_deceleration_mps2/2,a_lo,a_hi),
          Geometry::clamp(p_.max_acceleration_mps2/2,a_lo,a_hi)};
        for(double j : {-1.0,-.5,0.0,.5,1.0})
          accelerations.push_back(Geometry::clamp(q.a+j*jerk*dt,a_lo,a_hi));
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
          const double hi=std::min(recovery_peak,q.v+std::min((4*q.a+2*a)*dt+jerk*dt*dt,
                                                                 (2*q.a+4*a)*dt+jerk*dt*dt)/6);
          if(lo>hi+1e-12) continue;
          const double upper=std::max(lo,hi);
          std::vector<double> velocities{lo,upper};
          for(double v : {q.v+(q.a+a)*dt/2,0.0,cap,q.v,curve_cap})
            velocities.push_back(Geometry::clamp(v,lo,upper));
          std::sort(velocities.begin(),velocities.end());
          velocities.erase(std::unique(velocities.begin(),velocities.end()),velocities.end());
          for(double v:velocities) {
            if(v<0 || v>recovery_peak) continue;
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
            if((!lateral_clear && !lateralAccelerationClear(arc,edge,ego,recovery)) ||
               !edgeClear(c,arc,edge,physical_proven)) continue;
            const double cost=q.cost+edgeCost(c,arc,edge,origin_s,std::min(cap,limits.preferred_speed_mps));
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

      if(layer+1==clock.size() &&
         ego.speed_mps*ego.speed_mps*std::abs(ego.curvature_inv_m)>p_.max_lateral_accel_mps2+1e-6)
        next.erase(std::remove_if(next.begin(),next.end(),[&](const State& q){return q.v>cap+1e-8;}),next.end());
      if(next.empty()) return {};
      pruneSpeedLayer(next);
      layers.push_back(std::move(next));
    }
    // #RETAINED_SPEED_VALIDATION
    // map 기하를 시간별 경로점으로 다시 표본화하면 위치 기반 곡률과 Hermite
    // 재구성 검사가 원래 arc 검사보다 엄격해질 수 있다. 최저 비용 후보 하나가
    // 최종 검사에 실패했다고 같은 beam의 다른 속도 계획까지 버리지 않는다.
    // 이미 계산·보존한 후보만 기존 비용 순서로 복원하고, 같은 초기 상태와
    // 도로·충돌·곡률·횡가속도·가감속·jerk 전체 검사를 통과한 첫 결과를 쓴다.
    // beam 크기나 안전 한도는 늘리지 않으며, 전부 실패하면 빈 경로를 반환한다.
    for(std::size_t terminal=0;terminal<layers.back().size();++terminal) {
      int selected=static_cast<int>(terminal);
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
        const auto sampled=sampleEdge(arc,edge,true);
        out.insert(out.end(),sampled.begin()+(out.empty() ? 0 : 1),sampled.end());
      }
      out.front().pose=ego.pose; out.front().curvature_inv_m=ego.curvature_inv_m;
      if(checkPlannedPath(ego,reference_map,c.predictions,limits,out)) return out;
    }
    return {};
  }
  void validateParameters() const {
    for (double x : {p_.vehicle.front_m,p_.vehicle.rear_m,p_.vehicle.width_m,p_.horizon_m,p_.dp_step_m,
                    p_.lateral_step_m,p_.dp_slope_sample,p_.output_step_m,p_.corridor_half_width_m,p_.max_curvature_inv_m,
                    p_.max_lateral_accel_mps2,p_.max_speed_mps,p_.max_lateral_slope,p_.max_initial_heading_error_rad,
                    p_.max_initial_distance_m,p_.qp_rho,p_.qp_tolerance,p_.min_horizon_m,p_.road_boundary_step_m,
                    p_.max_acceleration_mps2,p_.max_deceleration_mps2,p_.max_longitudinal_jerk_mps3,
                    p_.speed_time_step_sec,p_.trajectory_output_step_sec,p_.trajectory_horizon_m,
                    p_.following_time_headway_sec})
      if (!std::isfinite(x) || x<=0.0) throw std::invalid_argument("nonpositive/nonfinite path parameter");
    for (double x : {p_.obstacle_margin_m,p_.road_margin_m,p_.reference_weight,p_.dp_tracking_weight,
                    p_.slope_weight,p_.bending_weight,p_.jerk_weight,p_.previous_weight,
                    p_.speed_tracking_weight,p_.acceleration_weight,p_.longitudinal_jerk_weight,
                    p_.following_min_gap_m,p_.preferred_lateral_accel_mps2,
                    p_.pedestrian_caution_speed_mps,p_.pedestrian_caution_distance_m,p_.pedestrian_near_preferred_speed_mps})
      if (!std::isfinite(x) || x<0.0) throw std::invalid_argument("negative/nonfinite path weight or margin");
    if (p_.preferred_lateral_accel_mps2>p_.max_lateral_accel_mps2 ||
        p_.horizon_m/p_.output_step_m>500 || p_.horizon_m/p_.dp_step_m>100 ||
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
    if (!Geometry::finite(ego.pose) || !std::isfinite(ego.stamp_sec) || ego.stamp_sec<0.0 ||
        ref.size()<2 ||
        !std::isfinite(pred.reference_stamp_sec) || pred.reference_stamp_sec<0.0 ||
        !std::isfinite(pred.step_sec) || pred.step_sec<=0.0 || pred.step_sec>2.0)
      throw std::invalid_argument("invalid calculation epoch, horizon or array dimensions");
    for (const auto& r : ref)
      if (!Geometry::finite(r.position) || !std::isfinite(r.min_l_m) || !std::isfinite(r.max_l_m) || r.min_l_m>=r.max_l_m)
        throw std::invalid_argument("invalid reference boundary");
    const auto validate_slice=[&](const std::vector<PredictedObject>& slice) {
      if (slice.size()!=pred.current.size()) throw std::invalid_argument("prediction slice loses objects");
      for (std::size_t j=0;j<slice.size();++j) {
        const auto& o=slice[j];
        if (o.id!=pred.current[j].id || !Geometry::finite(o.pose) || !Geometry::finite(o.velocity_mps) ||
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
    return Geometry::lerp(a.time_sec,it->time_sec,(s-a.progress_m)/(it->progress_m-a.progress_m));
  }
  static double progressAt(const std::vector<ProgressTime>& schedule,double time) {
    const auto it=std::lower_bound(schedule.begin(),schedule.end(),time,
                                  [](const ProgressTime& a,double b){return a.time_sec<b;});
    if (it==schedule.begin()) return it->progress_m;
    if (it==schedule.end()) return schedule.back().progress_m;
    const auto& a=*(it-1);
    return Geometry::lerp(a.progress_m,it->progress_m,(time-a.time_sec)/(it->time_sec-a.time_sec));
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
        out.push_back({{Geometry::lerp(a.pose.position,b.pose.position,u),
                        Geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u)},
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
      auto d=Geometry::sub(after,before);
      // A one-sided chord lags a curve's endpoint tangent by half a sample.
      // That rotates the long front overhang toward the road boundary even
      // when the rear axle fits. Use a one-sided quadratic derivative in
      // reference progress; preserve nonuniform station spacing as well.
      if(points.size()>2 && (i==0 || i+1==points.size())) {
        const std::size_t first=i==0 ? 0 : points.size()-3;
        const double h0=points[first+1].progress_m-points[first].progress_m;
        const double h1=points[first+2].progress_m-points[first+1].progress_m;
        if(h0>1e-12 && h1>1e-12) {
          const Vec2 a=Geometry::mul(Geometry::sub(points[first+1].pose.position,points[first].pose.position),1/h0);
          const Vec2 b=Geometry::mul(Geometry::sub(points[first+2].pose.position,points[first+1].pose.position),1/h1);
          d=i==0 ? Geometry::sub(Geometry::mul(a,(2*h0+h1)/(h0+h1)),Geometry::mul(b,h0/(h0+h1))) :
                   Geometry::sub(Geometry::mul(b,(2*h1+h0)/(h0+h1)),Geometry::mul(a,h1/(h0+h1)));
        }
      }
      points[i].pose.yaw_rad=std::atan2(d.y,d.x);
      if (i>0 && i+1<points.size()) points[i].curvature_inv_m=Geometry::curvature(before,points[i].pose.position,after);
    }
    if (points.size()>2) { points.front().curvature_inv_m=points[1].curvature_inv_m; points.back().curvature_inv_m=points[points.size()-2].curvature_inv_m; }
  }
  std::vector<PathPoint> makeEdge(const Context& c,double s0,double l0,double s1,double l1,double dl,double end_dl) const {
    const int steps=std::max(2,static_cast<int>(std::ceil((s1-s0)/p_.output_step_m)));
    std::vector<PathPoint> out;
    for (int i=0;i<=steps;++i) {
      const double u=static_cast<double>(i)/steps, s=Geometry::lerp(s0,s1,u);
      const auto r=c.reference.at(c.start_s+s);
      const double l=quintic(l0,l1,dl,end_dl,s1-s0,u);
      out.push_back({{Geometry::add(r.position,Geometry::mul(Geometry::normal(r.yaw),l)),r.yaw},s,timeAt(c.schedule,s),0.0});
    }
    setGeometry(out);
    const auto first=c.reference.at(c.start_s+s0),last=c.reference.at(c.start_s+s1);
    out.front().pose.yaw_rad=Geometry::wrap(first.yaw+std::atan2(dl,first.metric*(1-first.curvature*l0)));
    out.back().pose.yaw_rad=Geometry::wrap(last.yaw+std::atan2(end_dl,last.metric*(1-last.curvature*l1)));
    return withScheduleKnots(out,c.schedule);
  }
  double previousL(const Reference& ref,double s,const std::vector<PathPoint>& path) const {
    if (path.empty()) return 0.0;
    const auto target=ref.at(s);
    double best=std::numeric_limits<double>::infinity(), l=0.0;
    for (const auto& point : path) {
      const auto d=Geometry::sub(point.pose.position,target.position);
      const double longitudinal=std::abs(Geometry::dot(d,Geometry::unit(target.yaw)));
      if (longitudinal<best && longitudinal<1.0 && std::abs(Geometry::wrap(point.pose.yaw_rad-target.yaw))<0.8) {
        best=longitudinal; l=Geometry::dot(d,Geometry::normal(target.yaw));
      }
    }
    return l;
  }
  std::vector<std::pair<double,double>> roadLateralIntervals(const Context& c,const RoadSample& road,
                                     double heading,double lower,double upper) const {
    auto box=Geometry::egoBox({road.position,heading},p_.vehicle);
    box.half_length+=p_.road_margin_m; box.half_width+=p_.road_margin_m;
    const Vec2 direction=Geometry::normal(road.yaw);
    const double separation=std::max(1e-6,2*p_.qp_tolerance);
    std::vector<std::pair<double,double>> intervals{{lower,upper}};
    // SAT between each road segment and the translated rectangle gives an
    // exact forbidden interval for a FIXED heading. Subtract all of them;
    // concave notches must not be reduced to four corner-distance tests.
    // #ROAD_INTERVAL_QUERY
    // lower~upper의 모든 횡이동 차체는 두 끝점의 네 모서리 AABB 안에 있다.
    // 전체 도로 경계를 유지하며 이 영역과 확실히 분리된 선분만 제외한다.
    // SAT의 폐구간 접촉·오목 경계·separation·구간 차집합 식은 기존 그대로다.
    Geometry::Bounds swept;
    for(const auto corner:Geometry::corners(box))
      for(double offset:{lower,upper})
        swept.include(Geometry::add(corner,Geometry::mul(direction,offset)));
    const auto subtract_boundary=[&](const std::vector<Vec2>& boundary,
                                      const Geometry::BoundaryIndex& index) {
      for(const auto i:index.queryBounds(swept)) {
        const std::size_t j=i==0?boundary.size()-1:i-1;
        const Vec2 edge=Geometry::sub(boundary[i],boundary[j]);
        const double length=Geometry::norm(edge);
        if(length<1e-12) continue;
        const Vec2 segment_normal{-edge.y/length,edge.x/length};
        double enter=-std::numeric_limits<double>::infinity(),leave=std::numeric_limits<double>::infinity();
        bool intersects=true;
        for(Vec2 axis:{Geometry::unit(heading),Geometry::normal(heading),segment_normal}) {
          const double a=Geometry::dot(Geometry::sub(boundary[j],box.pose.position),axis);
          const double b=Geometry::dot(Geometry::sub(boundary[i],box.pose.position),axis);
          const double lo=std::min(a,b),hi=std::max(a,b);
          const double radius=Geometry::extent(box,axis)+1e-9;
          const double coefficient=Geometry::dot(direction,axis);
          if(std::abs(coefficient)<1e-12) {
            if(lo>radius || hi<-radius) { intersects=false; break; }
          } else {
            double first=(lo-radius)/coefficient,last=(hi+radius)/coefficient;
            if(first>last) std::swap(first,last);
            enter=std::max(enter,first); leave=std::min(leave,last);
            if(enter>leave) { intersects=false; break; }
          }
        }
        if(!intersects) continue;
        std::vector<std::pair<double,double>> remaining;
        for(const auto& range:intervals) {
          if(leave<range.first || enter>range.second) { remaining.push_back(range); continue; }
          if(range.first<enter-separation) remaining.emplace_back(range.first,std::min(range.second,enter-separation));
          if(range.second>leave+separation) remaining.emplace_back(std::max(range.first,leave+separation),range.second);
        }
        intervals=std::move(remaining);
        if(intervals.empty()) return;
      }
    };
    // #REVISITED_INTERSECTION_ROAD_UNION
    // map route가 같은 교차로를 다시 지나면, 전체 중심선의 좌우를 한 다각형으로
    // 닫았을 때 겹친 도로가 홀짝 판정으로 지워진다. map 입력에서는 기존 합법
    // lanelet 합집합의 실제 외곽선과 구멍을 같은 SAT 구간 계산에 사용한다.
    // 각 ring을 별도로 처리해 떨어진 영역 사이에 가상 연결 선분을 만들지 않는다.
    // 독립 reference 입력은 기존 경계를 유지한다. 차체 전체·경계 접촉·구멍·
    // 일방향 차선 변경·충돌·곡률·속도 검사는 그대로 적용한다.
    if(c.native_region) {
      for(const auto& component:c.native_region->components) {
        subtract_boundary(component.outer->points,component.outer->index);
        if(intervals.empty()) return intervals;
        for(const auto& hole:component.holes) {
          subtract_boundary(hole->points,hole->index);
          if(intervals.empty()) return intervals;
        }
      }
    } else subtract_boundary(c.boundary,c.boundary_index);
    // No boundary intersection also describes rectangles entirely outside
    // the polygon. Admit only components whose interior is inside the road.
    intervals.erase(std::remove_if(intervals.begin(),intervals.end(),[&](const std::pair<double,double>& range) {
      auto translated=box;
      translated.pose.position=Geometry::add(box.pose.position,Geometry::mul(direction,(range.first+range.second)/2));
      return !roadContainsBox(c,translated);
    }),intervals.end());
    return intervals;
  }
  bool lateralSlopeClear(const Context& c,const std::vector<PathPoint>& path) const {
    double previous=0;
    for(std::size_t i=0;i<path.size();++i) {
      const auto r=c.reference.at(c.start_s+path[i].progress_m);
      const double offset=Geometry::dot(Geometry::sub(path[i].pose.position,r.position),Geometry::normal(r.yaw));
      if(i>0 && std::abs(offset-previous)>
          (p_.max_lateral_slope+2*p_.qp_tolerance)*(path[i].progress_m-path[i-1].progress_m)) return false;
      previous=offset;
    }
    return true;
  }
  bool roadContainsBox(const Context& c,const Geometry::Box& box) const {
    // 실제 map polygon이 있으면 중심선의 자기 교차로 생긴 가상 경계로
    // 차체를 다시 거부하지 않는다. 두 경로 모두 전체 차체 포함을 검사한다.
    return c.native_region ? c.native_region->containsBox(box) :
                             c.boundary_index.containsBox(box);
  }
  bool roadClearWithMargin(const Context& c,Pose2 first,Pose2 last,double margin_m,int depth=0,
                           const NativeRoadRegion* region=nullptr) const {
    // 도로 검사만 원래 polygon으로 수행한다. 객체 예측 경계를 담은 Context를
    // 다시 만들지 않으며, 충돌 검사는 기존 Context와 시간별 예측을 그대로 쓴다.
    const auto* native=region?region:c.native_region;
    const auto contains=[&](const Geometry::Box& box) {
      return native?native->containsBox(box):c.boundary_index.containsBox(box);
    };
    if(native && !native->allowsMotion(first.position,last.position))return false;
    if (contains(
          Geometry::sweptEgoEnvelope(first,last,p_.vehicle,margin_m))) return true;
    if (depth>=6 || (Geometry::norm(Geometry::sub(first.position,last.position))<1e-9 &&
                    std::abs(Geometry::wrap(first.yaw_rad-last.yaw_rad))<1e-9)) return false;
    if (!contains(
          Geometry::sweptEgoEnvelope(first,first,p_.vehicle,margin_m)) ||
        !contains(
          Geometry::sweptEgoEnvelope(last,last,p_.vehicle,margin_m))) return false;
    // Refine an over-conservative enclosure, never replace it with point tests.
    const Pose2 mid{Geometry::lerp(first.position,last.position,0.5),
                    Geometry::yawLerp(first.yaw_rad,last.yaw_rad,0.5)};
    return roadClearWithMargin(c,first,mid,margin_m,depth+1,native) &&
           roadClearWithMargin(c,mid,last,margin_m,depth+1,native);
  }
  bool roadClear(const Context& c,Pose2 first,Pose2 last,int depth=0) const {
    return roadClearWithMargin(c,first,last,p_.road_margin_m,depth);
  }
  bool collision(const Context& c,const PathPoint& a,const PathPoint& b,double motion_pad=0) const {
    Geometry::Bounds centers; centers.include(a.pose.position); centers.include(b.pose.position);
    const double angle=std::min(Geometry::kPi,
        std::abs(Geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad))+
        128*std::numeric_limits<double>::epsilon()*std::max({1.0,std::abs(a.pose.yaw_rad),std::abs(b.pose.yaw_rad)})+1e-10);
    const auto indices=collisionObjectIndices(c,centers,motion_pad,angle,
        c.prediction_offset+a.time_sec,c.prediction_offset+b.time_sec,a.pose.yaw_rad);
    if(indices.empty())return false;
    return collisionNarrow(c,a,b,motion_pad,&indices);
  }
  bool collisionNarrow(const Context& c,const PathPoint& a,const PathPoint& b,double motion_pad=0,
                       const std::vector<std::size_t>* indices=nullptr) const {
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
        return Geometry::egoBox({Geometry::lerp(a.pose.position,b.pose.position,u),Geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u)},p_.vehicle);
      };
      auto e0=ego_at(cuts[k-1]),e1=ego_at(cuts[k]);
      const double angle=std::abs(Geometry::wrap(e1.pose.yaw_rad-e0.pose.yaw_rad));
      const double offset=std::abs(p_.vehicle.front_m-p_.vehicle.rear_m)/2.0;
      const double center_arc_pad=offset*angle*angle/8.0;
      e0.half_length+=center_arc_pad+motion_pad; e0.half_width+=center_arc_pad+motion_pad;
      e1.half_length+=center_arc_pad+motion_pad; e1.half_width+=center_arc_pad+motion_pad;
      for (std::size_t index=0;index<(indices?indices->size():c.predictions.current.size());++index) {
        const std::size_t j=indices?(*indices)[index]:index;
        const auto o0=samplePrediction(c.predictions,j,cuts[k-1]);
        const auto o1=samplePrediction(c.predictions,j,cuts[k]);
        if (Geometry::sweptOverlap(e0,e1,Geometry::objectBox(o0,p_.obstacle_margin_m),Geometry::objectBox(o1,p_.obstacle_margin_m))) return true;
      }
    }
    return false;
  }
  bool checkPath(const Context& c,const std::vector<PathPoint>& path) const {
    for (std::size_t i=0;i<path.size();++i) {
      if (!Geometry::finite(path[i].pose) || !std::isfinite(path[i].curvature_inv_m) ||
          std::abs(path[i].curvature_inv_m)>p_.max_curvature_inv_m+1e-6) return false;
      if (i==0) {
        if (!roadClear(c,path[i].pose,path[i].pose) || (!c.geometric_only && collision(c,path[i],path[i]))) return false;
        continue;
      }
      const auto& a=path[i-1]; const auto& b=path[i];
      const double dt=b.time_sec-a.time_sec;
      const double distance=Geometry::norm(Geometry::sub(b.pose.position,a.pose.position));
      if (!std::isfinite(dt) || dt<=0.0 || !std::isfinite(distance)) return false;
      const double angle=std::abs(Geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad));
      // Waiting is allowed in a supplied final timing, but rotating a car in
      // place is not. Forward generation itself still uses increasing progress.
      if (distance==0.0 && (angle>1e-10 || a.progress_m!=b.progress_m)) return false;
      const double speed=distance/dt;
      const double curvature=std::max({std::abs(a.curvature_inv_m),std::abs(b.curvature_inv_m),
                                      distance>0.0 ? angle/distance : 0.0});
      if (distance>0.0 && (Geometry::dot(Geometry::sub(b.pose.position,a.pose.position),Geometry::unit(a.pose.yaw_rad))<=0.0 ||
                          Geometry::dot(Geometry::sub(b.pose.position,a.pose.position),Geometry::unit(b.pose.yaw_rad))<=0.0)) return false;
      // #GEOMETRY_TIMING_SEPARATION
      // 이 speed는 DP 형상을 고르는 가정 시간표의 값일 수 있다. integrated
      // 후보의 가정 속도로 횡가속도를 판정하면 멀리 있는 정적 장애물을
      // 저속으로 회피 가능한 형상까지 버린다. proposal에서는 형상·도로·실제
      // 예측 충돌을 계속 검사하고 시간 동역학은 최종 planSpeed와
      // checkPlannedPath가 실측 초기 상태에서 전부 재검증한다. 외부에서 준
      // 최종 schedule의 기존 검사와 발행 경로의 제한은 그대로 유지한다.
      if (curvature>p_.max_curvature_inv_m+1e-6 || (!c.geometric_only && !c.proposal_timing &&
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
    for (Eigen::Index j=0;j<z.size();++j) z[j]=Geometry::clamp(z[j],lower[j],upper[j]);
    for (int iteration=1;iteration<=p_.qp_max_iterations;++iteration) {
      x=factor.solve(-q+rho*a.transpose()*(z-u));
      if (!x.allFinite()) return false;
      const Eigen::VectorXd ax=a*x;
      const Eigen::VectorXd relaxed=1.6*ax-0.6*z;
      z=relaxed+u;
      for (Eigen::Index j=0;j<z.size();++j) z[j]=Geometry::clamp(z[j],lower[j],upper[j]);
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
          const double next=Geometry::clamp(rho*std::sqrt(ratio),0.1,1e4);
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
    // #QP_KKT_FACTORIZATION
    // 양의 정부호 Hessian H는 활성 제약을 추가·제거해도 변하지 않는다.
    // 매번 큰 [H C'; C 0] 행렬 전체를 분해하는 대신 H를 한 번 분해하고
    // C H^-1 C'의 작은 Schur 계로 같은 primal·dual 해를 구한다.
    // 비용·활성 제약 선택·반복 한도는 유지한다. 특이한 제약은 거부하며,
    // 원래 KKT 잔차·전체 부등식·dual 부호와 최종 차체/충돌 검사를 유지한다.
    const Eigen::LDLT<Eigen::MatrixXd> h_factor(h);
    if(h_factor.info()!=Eigen::Success || !h_factor.isPositive()) return false;
    const Eigen::VectorXd unconstrained=h_factor.solve(-q);
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
      Eigen::VectorXd candidate(n+m);
      candidate.head(n)=unconstrained;
      if(m>0) {
        const Eigen::MatrixXd constraints=kkt.bottomLeftCorner(m,n);
        const Eigen::MatrixXd projected=h_factor.solve(constraints.transpose());
        const Eigen::MatrixXd schur=constraints*projected;
        const Eigen::FullPivLU<Eigen::MatrixXd> factor(schur);
        if(!factor.isInvertible()) return false;
        candidate.tail(m)=factor.solve(constraints*unconstrained-rhs.tail(m));
        candidate.head(n)-=projected*candidate.tail(m);
      }
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
 public:
  Message process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                  const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                  const sensor_msgs::Imu& imu, const Route& route, const PredictedObjects& predictions,
                  const ros::Time& stamp) {
    const auto& orientation = ego.pose.pose.orientation;
    const Eigen::Quaterniond map_from_base(orientation.w, orientation.x, orientation.y, orientation.z);
    const auto& position = ego.pose.pose.position;
    EgoState state;
    state.pose.position = {position.x, position.y};
    state.pose.yaw_rad = std::atan2(2 * (orientation.w*orientation.z + orientation.x*orientation.y),
                                  1 - 2 * (orientation.y*orientation.y + orientation.z*orientation.z));
    state.stamp_sec = stamp.toSec();
    // #MEASURED_FORWARD_SPEED
    // 저속 정지·출발 시 EKF 선속도 추정이 음수로 흔들려도 실제 차량은 전진할 수 있다.
    // 전진 전용 속도 계획의 초기 조건은 같은 Pipeline이 전달한 차량 상태의
    // 부호 있는 전방 속도(m/s)를 사용한다. 절댓값이나 양의 offset을 적용하지 않아
    // 실제 후진 입력은 기존 계산의 전진 도메인 조건에서 그대로 거부된다.
    // 위치·자세와 그 시각 전파는 ego의 EKF 추정 및 body twist 계약을 유지한다.
    state.speed_mps = competition.velocity_x_mps;
    const double stationary_specific_force_x_mps2 =
      (map_from_base.conjugate() * Eigen::Vector3d(0, 0, 9.81)).x();
    state.acceleration_mps2 = imu.linear_acceleration.x - stationary_specific_force_x_mps2;
    const bool brake_held=competition.brake>=.9 && competition.accel==0 &&
                          std::abs(competition.signed_velocity_mps)<=1e-3;
    const double kStandstillSpeedMps = brake_held ? 1e-3 : 1e-4;
    const double kStandstillAccelerationMps2 = brake_held ? .05 : 1e-3;
    const double yaw_rate_radps = ego.twist.twist.angular.z;
    // #STANDSTILL_ROUNDING
    // 정지 로그에 선속도 −7.45e−6m/s와 각속도 3.15e−5rad/s가 함께 남는다.
    // 정지에서는 yaw/v가 정의되지 않으므로 곡률 한도로 각속도 잔차를 판정하면
    // 이 입력이 실제 후진으로 오인된다. 선속도와 각속도의 작은 반올림 범위를
    // 각 단위에서 독립 판정한다. 이 범위를 넘는 후진·회전과 가감속은 보존한다.
    // 완전 제동·가속 페달 0·보조 속도도 1 mm/s 이내인 실제 정착 로그에는
    // v=0.000175 m/s, yaw rate=-0.0000524 rad/s, a=-0.0263 m/s²가 남았다.
    // 이때만 1 mm/s, 0.001 rad/s, 0.05 m/s² 안의 잔차를 영점 처리한다.
    // 일반 주행에는 원래 작은 허용값을 유지하며 .002 m/s 전진, 실제 후진과
    // .1 m/s² 출발 가속도는 보존한다. 목표 속도에 오프셋을 더하지 않는다.
    const double kStandstillYawRateRadps = brake_held ? 1e-3 : 1e-4;
    if (std::abs(state.speed_mps) <= kStandstillSpeedMps &&
        std::abs(yaw_rate_radps) <= kStandstillYawRateRadps) {
      state.speed_mps = 0;
      if (std::abs(state.acceleration_mps2) <= kStandstillAccelerationMps2)
        state.acceleration_mps2 = 0;
    }
    state.curvature_inv_m = state.speed_mps == 0 ? 0 : yaw_rate_radps / state.speed_mps;

    // #SNAPSHOT_TIME_ALIGNMENT
    // UDP 객체와 IMU/ego는 서로 다른 주기로 수신된다. Pipeline의 계산 기준
    // 시각(stamp)까지 측정 body twist로 뒷차축 자세를 등속 선회 전파한다.
    // 객체·예측의 원본 시각을 유지해 예측 구간의 offset을 계산한다.
    // ego 원본 메시지나 기준점을 바꾸지 않으며 외부 현재 시각은 조회하지 않는다.
    const double dt=state.stamp_sec-ego.header.stamp.toSec();
    const double angle=yaw_rate_radps*dt;
    const double forward=ego.twist.twist.linear.x, lateral=ego.twist.twist.linear.y;
    double a=dt,b=0;
    if(std::abs(yaw_rate_radps)>1e-9) {
      a=std::sin(angle)/yaw_rate_radps;
      const double half_sine=std::sin(angle/2);
      b=2*half_sine*half_sine/yaw_rate_radps;
    }
    const Vec2 displacement{forward*a-lateral*b,forward*b+lateral*a};
    state.pose.position=Geometry::add(state.pose.position,Geometry::rotate(displacement,state.pose.yaw_rad));
    state.pose.yaw_rad=Geometry::wrap(state.pose.yaw_rad+angle);

    Predictions native_predictions;
    native_predictions.reference_stamp_sec = objects.header.stamp.toSec();
    const auto convert = [&](const interfaces::Objects& sample, std::vector<PredictedObject>& output) {
      output.reserve(sample.objects.size());
      for (const auto& object : sample.objects) {
        const auto& q = object.bbox_pose.orientation;
        PredictedObject converted;
        converted.id = object.id;
        converted.kind = object.label == "vehicle" ? ObjectKind::Vehicle :
                         object.label == "pedestrian" ? ObjectKind::Pedestrian : ObjectKind::Unknown;
        converted.pose.position = {object.position.x, object.position.y};
        converted.pose.yaw_rad = std::atan2(2 * (q.w*q.z + q.x*q.y),
                                         1 - 2 * (q.y*q.y + q.z*q.z));
        converted.velocity_mps = {object.velocity.x, object.velocity.y};
        converted.length_m = object.bbox_size.x;
        converted.width_m = object.bbox_size.y;
        converted.source_stamp_sec = objects.header.stamp.toSec();
        output.push_back(converted);
      }
    };
    convert(objects, native_predictions.current);
    for (std::size_t i = 0; i < predictions.size(); ++i)
      convert(predictions[i], native_predictions.future[i]);

    const auto body_route=rearFootprintRoute(route);
    const auto native_reference=nativeReference(body_route);
    const auto& reference=native_reference->points;
    const auto& centerline=native_reference->heights;
    const NativeRegionScope region_scope(active_native_region_,nativeRoadRegion(body_route));
    // #EXACT_PLANNING_INPUT_STAMPS
    // 실제 조립된 원본 입력 시각을 계산 시작 전에 DEBUG로 남긴다. 수신 시각에
    // 가까운 메시지를 고르는 사후 추정과 구분해 느린 호출을 정확히 재생한다.
    // 모든 stamp·전파 후 뒷차축 상태는 읽기만 하며 경로·입력 수명은 바꾸지 않는다.
    // DEBUG 비활성 시 문자열을 만들지 않으며 외부 시계·파일·토픽을 사용하지 않는다.
    ROS_DEBUG_STREAM_NAMED("planning_input_stamps", std::setprecision(17)
        << "plan_ns=" << stamp.toNSec() << " ego_ns=" << ego.header.stamp.toNSec()
        << " status_ns=" << competition.header.stamp.toNSec()
        << " objects_ns=" << objects.header.stamp.toNSec() << " imu_ns=" << imu.header.stamp.toNSec()
        << " tf_ns=" << (tf.transforms.empty() ? 0 : tf.transforms.front().header.stamp.toNSec())
        << " x=" << state.pose.position.x << " y=" << state.pose.position.y
        << " yaw=" << state.pose.yaw_rad << " speed=" << state.speed_mps
        << " acceleration=" << state.acceleration_mps2 << " curvature=" << state.curvature_inv_m);
    PlanningConstraints mission_limits;
    if(p_.normal_speed_limit_mps>0) {
      const bool exempt=p_.speed_exemption_link_ids.count(competition.link_id)!=0;
      if(!exempt) {
        approaching_normal_speed_limit_=false;
        mission_limits.maximum_speed_mps=p_.normal_speed_limit_mps;
      } else {
        // #REGULATION_SPEED_TRANSITION
        // 공식 예외 구간은 실제 Link ID로 구분한다. 고주로의 100km/h는
        // 기존 팀 운용 목표이며 대회 규정상 상한이라는 뜻은 아니다.
        // 예외 뒤의 첫 일반 Link 시작까지 map 참조선 거리를 구하고, 실측
        // v/a에서 기존 jerk 제한 S-curve로 60km/h까지 줄이는 거리와 설정한
        // 전달 응답 시간을 미리 확보한다. 시작한 감속은 예외 이탈까지 유지해
        // 매 재계획에서 가속/감속 목표가 번갈아 바뀌지 않게 한다.
        // 원본 속도·가속도·경로 시각과 차체·도로·충돌·jerk 검사는 유지한다.
        // GT 객체가 아니라 기존 Competition Status/route 계약만 사용한다.
        if(!approaching_normal_speed_limit_) {
          const Reference native_road(reference);
          const auto origin=native_road.project(state.pose.position,0,native_road.length());
          bool passed_exempt=false;
          for(const auto& lane:*body_route) {
            if(p_.speed_exemption_link_ids.count(lane.attributeOr("mgeo:link_id",std::string("")))) {
              passed_exempt=true;continue;
            }
            if(!passed_exempt) continue;
            const auto start=lane.centerline3d().front();
            const auto boundary=native_road.project({start.x(),start.y()},origin.s,native_road.length());
            const double distance=std::max(0.0,boundary.s-origin.s-p_.vehicle.front_m);
            PlanningConstraints normal;normal.maximum_speed_mps=p_.normal_speed_limit_mps;
            const auto recovery=ceilingRecovery(state,normal);
            double slowing_distance=0;
            for(std::size_t i=0;i<recovery.count;++i) {
              const auto& phase=recovery.phase[i];const double h=phase.h;
              slowing_distance+=phase.v*h+phase.a*h*h/2+phase.j*h*h*h/6;
            }
            approaching_normal_speed_limit_=distance<=slowing_distance+
                std::max(0.0,state.speed_mps)*p_.speed_limit_response_sec;
            break;
          }
        }
        mission_limits.maximum_speed_mps=approaching_normal_speed_limit_ ?
            p_.normal_speed_limit_mps : p_.exception_target_speed_mps;
      }
    }
    previous_path_ = plan(state, reference, native_predictions, mission_limits, previous_path_);
    // #PLANNING_DIAGNOSTICS: 실패 시 계산에 실제 사용한 초기 조건만 기록한다.
    // 경로·제약을 보정하거나 제어 토픽을 발행하지 않으며 빈 경로의 원인을 추적한다.
    if (previous_path_.empty()) {
      ROS_WARN_STREAM_THROTTLE(1.0, "core_path_planner empty trajectory: speed=" << state.speed_mps
          << " acceleration=" << state.acceleration_mps2 << " curvature=" << state.curvature_inv_m
          << " measured_brake=" << competition.brake << " measured_accel=" << competition.accel);
      // #EMPTY_INITIAL_STATE_DIAGNOSTIC
      // 빈 경로에 실제로 사용한 전파 후 뒷차축 상태와 초기 차체 포함 여부를
      // 같은 map 도로 합집합으로 기록한다. 추정 자세·추종 오차·5cm 여유의
      // 영향을 native 관측과 분리하기 위한 DEBUG 전용 기록이다. 계산 상태나
      // 도로·충돌 한도는 바꾸지 않으며 logger가 꺼지면 추가 검사를 실행하지 않는다.
      ROS_DEBUG_STREAM_NAMED("empty_initial_state", std::setprecision(17)
          << "stamp=" << state.stamp_sec << " x=" << state.pose.position.x
          << " y=" << state.pose.position.y << " yaw=" << state.pose.yaw_rad
          << " speed=" << state.speed_mps << " acceleration=" << state.acceleration_mps2
          << " curvature=" << state.curvature_inv_m << " ego_dt=" << dt
          << " physical_inside=" << (active_native_region_ ? static_cast<int>(active_native_region_->containsBox(
               Geometry::egoBox(state.pose,p_.vehicle))) : -1)
          << " margin_inside=" << (active_native_region_ ? static_cast<int>(active_native_region_->containsBox(
               Geometry::sweptEgoEnvelope(state.pose,state.pose,p_.vehicle,p_.road_margin_m))) : -1));
    }
    Message message;
    for (const auto& point : previous_path_) {
      interfaces::PathPoint output;
      output.pose.pose.position.x = point.pose.position.x;
      output.pose.pose.position.y = point.pose.position.y;
      output.pose.pose.position.z = centerlineHeight(centerline, point.pose.position);
      output.pose.pose.orientation.z = std::sin(point.pose.yaw_rad/2);
      output.pose.pose.orientation.w = std::cos(point.pose.yaw_rad/2);
      // #PLANNED_TIME: 기준 stamp와 미래 예정 시각을 분리해 대기·재출발을 보존한다.
      output.time_from_start = ros::Duration(point.time_sec);
      output.velocity = point.speed_mps;
      output.acceleration = point.acceleration_mps2;
      message.points.push_back(output);
    }
    return message;
  }
 private:
  static double boundaryOffset(const lanelet::BasicLineString3d& line, Vec2 point, Vec2 normal) {
    double closest = std::numeric_limits<double>::infinity();
    double offset = 0;
    for (std::size_t i = 1; i < line.size(); ++i) {
      const Vec2 a{line[i - 1].x(), line[i - 1].y()}, b{line[i].x(), line[i].y()};
      const auto delta = Geometry::sub(b, a);
      const double squared = Geometry::dot(delta, delta);
      const double u = squared == 0 ? 0 : Geometry::clamp(Geometry::dot(Geometry::sub(point, a), delta)/squared, 0, 1);
      const auto projection = Geometry::add(a, Geometry::mul(delta, u));
      const auto difference = Geometry::sub(projection, point);
      const double distance = Geometry::dot(difference, difference);
      if (distance < closest) { closest = distance; offset = Geometry::dot(difference, normal); }
    }
    return offset;
  }
  struct NativeReference {
    std::vector<LaneKey> route_ids;
    std::vector<ReferencePoint> points;
    std::vector<Eigen::Vector3d> heights;
  };
  std::shared_ptr<const NativeReference> native_reference_cache_;
  static Eigen::Vector3d nearestCenterlinePoint(const lanelet::BasicLineString3d& line,
                                               const Eigen::Vector3d& point) {
    Eigen::Vector3d best=line.front();double distance=std::numeric_limits<double>::infinity();
    for(std::size_t i=1;i<line.size();++i) {
      const Eigen::Vector3d delta=line[i]-line[i-1];
      const double square=delta.head<2>().squaredNorm();
      const double u=square>0?Geometry::clamp((point-line[i-1]).head<2>().dot(delta.head<2>())/square,0,1):0;
      const Eigen::Vector3d candidate=line[i-1]+u*delta;
      const double d=(point-candidate).head<2>().squaredNorm();
      if(d<distance) { distance=d;best=candidate; }
    }
    return best;
  }
  // #NATIVE_ROUTE_TRANSITION
  // LaneletPath의 병렬 lane-change edge는 앞 차선 끝에서 옆 차선 시작으로
  // 후진하라는 뜻이 아니다. 같은 종방향 구간의 합법적인 병렬 전환만 묶고,
  // source 중심선에서 target 중심선으로 기존 0→1 quintic 가중치를 적용한다.
  // 이는 DP/QP가 사용할 map 참조선이며 발행 경로를 직접 만들지 않는다.
  // 원래 차선과 허용 옆 차선의 실제 경계로 폭을 구하고, 실제 polygon 합집합과
  // 전체 차체·충돌·곡률·속도 검사를 계속 적용한다. 뒷차축 초기 자세는 바꾸지
  // 않는다. 정적 지도에서 route ID와 각 lanelet의 진행 방향이 같을 때만 재사용한다.
  // 양방향 차선의 invert()는 같은 ID라도 경계 좌우와 중심선 진행 순서가 다르다.
  std::shared_ptr<const NativeReference> nativeReference(const Route& route) {
    std::vector<LaneKey> ids;for(const auto& lane:*route)ids.push_back(laneKey(lane));
    if(native_reference_cache_ && native_reference_cache_->route_ids==ids)return native_reference_cache_;
    auto result=std::make_shared<NativeReference>();result->route_ids=std::move(ids);
    for(std::size_t begin=0;begin<route->size();) {
      std::size_t end=begin;
      while(end+1<route->size()) {
        const auto left=legalNeighbor((*route)[end],true),right=legalNeighbor((*route)[end],false);
        const auto key=laneKey((*route)[end+1]);
        if((left && laneKey(*left)==key)||(right && laneKey(*right)==key))++end;else break;
      }
      std::size_t connected_end=end;
      auto source=(*route)[begin].centerline3d().basicLineString();
      auto target=(*route)[end].centerline3d().basicLineString();
      const auto polyline_length=[](const lanelet::BasicLineString3d& line) {
        double length=0;
        for(std::size_t i=1;i<line.size();++i)length+=(line[i]-line[i-1]).head<2>().norm();
        return length;
      };
      const auto append=[](const lanelet::ConstLanelet& lane,lanelet::BasicLineString3d& line) {
        for(const auto& point:lane.centerline3d().basicLineString())
          if(line.empty() || (point-line.back()).head<2>().norm()>1e-9) line.push_back(point);
      };
      // #SHORT_LANE_CHANGE_APPROACH
      // 다음 변경 구간이 horizon_m보다 짧아도, 바로 앞 source와 target이
      // 같은 방향으로 종방향 연결되고 그 앞에서도 동일한 옆 차선 변경이
      // 합법이면 기존 접근 구간의 마지막 horizon_m에서 변경을 시작한다.
      // 두 번째 변경의 source를 앞 변경에 소비하지 않고 13m 급변도 피한다.
      // 시작점은 지도 구간 끝에 고정하며 ego 이동으로 미루지 않는다.
      // 기존 도로·ROI·차체·충돌·곡률·시간 한도를 유지한다.
      bool approach_transition=false;
      if(begin==end && routing_graph_ && begin+2<route->size()) {
        const auto& next_source=(*route)[begin+1];
        const auto& next_target=(*route)[begin+2];
        const auto left=legalNeighbor(next_source,true),right=legalNeighbor(next_source,false);
        const bool left_change=left && laneKey(*left)==laneKey(next_target);
        const bool right_change=right && laneKey(*right)==laneKey(next_target);
        const double next_length=polyline_length(next_source.centerline3d().basicLineString());
        if((left_change || right_change) && next_length<p_.horizon_m &&
            polyline_length(source)+next_length>=p_.horizon_m) {
          const auto prior_target=legalNeighbor((*route)[begin],left_change);
          const auto following=routing_graph_->following((*route)[begin],false);
          if(prior_target && std::any_of(following.begin(),following.end(),
              [&](const lanelet::ConstLanelet& lane){return laneKey(lane)==laneKey(next_source);})) {
            const auto targets=routing_graph_->following(*prior_target,false);
            if(std::any_of(targets.begin(),targets.end(),
                [&](const lanelet::ConstLanelet& lane){return laneKey(lane)==laneKey(next_target);})) {
              append(next_source,source);
              target=prior_target->centerline3d().basicLineString();append(next_target,target);
              end=connected_end=begin+2;approach_transition=true;
            }
          }
        }
      }
      // #SHORT_LANE_CHANGE_CONNECTOR
      // 수 cm의 lanelet도 합법적인 옆 차선 연결을 가질 수 있다. 그 중심선
      // 길이 안에서 3.5m 횡이동을 끝내면 참조선에 급회전이 생겨 모든 실제
      // 곡률 검사를 통과하지 못한다. source/target 모두 종방향으로 연결되고
      // 같은 방향의 합법 left/right 관계가 유지되는 다음 구간만 이어 붙인다.
      // map 고정 전환 시작점과 horizon_m의 quintic은 유지하며, 분기에서
      // source가 유일하지 않거나 실선 등으로 변경이 끝나면 연장을 중단한다.
      // route·지도·허용 영역은 변경하지 않는다. 다음 target 구간을 중복
      // 연결하지 않고, 실제 차체·방향·충돌·곡률·시간 검사를 다시 적용한다.
      if(begin!=end && routing_graph_) {
        auto source_lane=(*route)[approach_transition?begin+1:begin],target_lane=(*route)[end];
        const auto left=legalNeighbor(source_lane,true),right=legalNeighbor(source_lane,false);
        const bool left_change=left && laneKey(*left)==laneKey(target_lane);
        const bool right_change=right && laneKey(*right)==laneKey(target_lane);
        double source_length=polyline_length(source);
        while((left_change || right_change) && source_length<p_.horizon_m &&
              connected_end+1<route->size()) {
          const auto& next_target=(*route)[connected_end+1];
          const auto following_targets=routing_graph_->following(target_lane,false);
          if(std::none_of(following_targets.begin(),following_targets.end(),
              [&](const lanelet::ConstLanelet& lane){return laneKey(lane)==laneKey(next_target);})) break;
          // #CONSECUTIVE_LANE_CHANGE_CONNECTOR
          // 다음 target이 또 다른 합법적 옆 차선 변경의 source이면 앞 변경의
          // 연장에 소비하지 않는다. 12구간의 1.3cm source를 소비하면 다음
          // 3.5m 횡이동이 거의 같은 종방향 위치의 직각 연결로 바뀐다.
          // source를 다음 그룹에 남겨 같은 map 고정 quintic과 horizon_m으로
          // 바로 옆 차선 변경을 각각 계산한다. route·지도·허용 도로와 전체
          // 차체·곡률·충돌·시간 검사는 유지하며 ROI 통합도 같은 영역을 쓴다.
          if(connected_end+2<route->size()) {
            const auto next_key=laneKey((*route)[connected_end+2]);
            const auto next_left=legalNeighbor(next_target,true),next_right=legalNeighbor(next_target,false);
            if((next_left && laneKey(*next_left)==next_key)||
               (next_right && laneKey(*next_right)==next_key)) break;
          }
          lanelet::ConstLanelets matches;
          for(const auto& next_source:routing_graph_->following(source_lane,false)) {
            const auto neighbor=legalNeighbor(next_source,left_change);
            if(neighbor && laneKey(*neighbor)==laneKey(next_target)) matches.push_back(next_source);
          }
          if(matches.size()!=1) break;
          const auto old_size=source.size();
          append(matches.front(),source);append(next_target,target);
          for(std::size_t i=std::max(std::size_t(1),old_size);i<source.size();++i)
            source_length+=(source[i]-source[i-1]).head<2>().norm();
          source_lane=matches.front();target_lane=next_target;++connected_end;
        }
      }
      std::map<LaneKey,lanelet::ConstLanelet> corridor;
      for(std::size_t j=begin;j<=connected_end;++j) {
        const auto& lane=(*route)[j];corridor.emplace(laneKey(lane),lane);
        for(bool left:{false,true}) {
          const auto next=corridorNeighbor(lane,left);
          if(next)corridor.emplace(laneKey(*next),*next);
        }
      }
      std::vector<Eigen::Vector3d> line;
      if(begin==end)line.assign(source.begin(),source.end());
      else {
        double total=0,station=0;
        for(std::size_t i=1;i<source.size();++i)total+=(source[i]-source[i-1]).head<2>().norm();
        // #ADJACENT_TARGET_REFERENCE
        // route의 병렬 edge는 바로 옆 목표 차선으로 변경하라는 뜻이다.
        // 100~200m 원본 lanelet 전체를 quintic 전환 길이로 쓰면 장애물을
        // 피해 먼저 이동한 차량보다 참조선이 출발 차선 쪽에 남는다. 그 결과
        // 명목 중앙 복귀가 변경을 되돌리고, 전역 route가 목표 차선만 남기는
        // 순간 참조선도 크게 바뀐다. 기존 형상 계획 거리 horizon_m 안에서
        // 같은 map 고정 quintic을 완료해 다음 계산에도 같은 목표를 유지한다.
        // ego 위치로 전환 시작점을 매번 미루지 않으며 실제 경로는 기존
        // DP/QP와 초기 곡률 정합·시간 계획이 생성한다. native 도로 합집합,
        // 방향별 차선 변경 허용, 차체·충돌·곡률·가감속 검사는 유지한다.
        const double transition_extent=std::min(total,p_.horizon_m);
        const double transition_start=approach_transition?total-transition_extent:0;
        line.push_back(source.front());
        for(std::size_t i=1;i<source.size();++i) {
          const double segment=(source[i]-source[i-1]).head<2>().norm();
          const int count=std::max(1,static_cast<int>(std::ceil(segment/p_.dp_step_m)));
          for(int k=1;k<=count;++k) {
            const double fraction=double(k)/count;
            const Eigen::Vector3d point=source[i-1]+fraction*(source[i]-source[i-1]);
            const double u=Geometry::clamp((station+fraction*segment-transition_start)/
                                           std::max(1e-9,transition_extent),0,1);
            const double weight=u*u*u*(10+u*(-15+6*u));
            const Eigen::Vector3d target_point=i+1==source.size() && k==count?
                target.back():nearestCenterlinePoint(target,point);
            line.push_back(point+weight*(target_point-point));
          }
          station+=segment;
        }
      }
      for(std::size_t i=0;i<line.size();++i) {
        const Vec2 point{line[i].x(),line[i].y()};
        if(!result->points.empty() && Geometry::norm(Geometry::sub(point,result->points.back().position))<1e-9)continue;
        const Eigen::Vector3d direction=line[std::min(i+1,line.size()-1)]-line[i==0?0:i-1];
        const double length=std::hypot(direction.x(),direction.y());
        const Vec2 normal{-direction.y()/length,direction.x()/length};
        double lower=std::numeric_limits<double>::infinity(),upper=-lower;
        for(const auto& item:corridor) {
          lower=std::min(lower,boundaryOffset(item.second.rightBound3d().basicLineString(),point,normal));
          upper=std::max(upper,boundaryOffset(item.second.leftBound3d().basicLineString(),point,normal));
        }
        result->points.push_back({point,lower,upper});result->heights.push_back(line[i]);
      }
      begin=connected_end+1;
    }
    native_reference_cache_=result;
    return result;
  }
  static double centerlineHeight(const std::vector<Eigen::Vector3d>& line, Vec2 point) {
    double closest = std::numeric_limits<double>::infinity();
    double height = 0;
    for (std::size_t i = 1; i < line.size(); ++i) {
      const Vec2 a{line[i - 1].x(), line[i - 1].y()}, b{line[i].x(), line[i].y()};
      const auto delta = Geometry::sub(b, a);
      const double squared = Geometry::dot(delta, delta);
      const double u = squared == 0 ? 0 : Geometry::clamp(Geometry::dot(Geometry::sub(point, a), delta)/squared, 0, 1);
      const auto difference = Geometry::sub(Geometry::add(a, Geometry::mul(delta, u)), point);
      const double distance = Geometry::dot(difference, difference);
      if (distance < closest) { closest = distance; height = line[i - 1].z() + u*(line[i].z() - line[i - 1].z()); }
    }
    return height;
  }
  std::vector<PathPoint> previous_path_;
  bool approaching_normal_speed_limit_=false;
};
}  // namespace path_planner
