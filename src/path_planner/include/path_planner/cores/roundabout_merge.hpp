#pragma once
#include <geometry_msgs/Point.h>
#include <path_planner/libraries/types.hpp>
#include <Eigen/Geometry>
#include <ros/console.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace path_planner {
struct RoundaboutMergeParameters {
  double local_path_length_m = 40.0;
  double desired_speed_mps = 8.0;
  double max_acceleration_mps2 = 2.0;
  double max_deceleration_mps2 = 4.0;
  double max_jerk_mps3 = 4.0;
  double max_lateral_acceleration_mps2 = 2.0;
  double time_step_sec = 0.25;
  double command_step_sec = 0.2;
  int beam_width = 64;
  double collision_margin_m = 0.5;
  double prediction_uncertainty_mps = 0.2;
  double vehicle_front_m = 3.845008;
  double vehicle_rear_m = 0.790000;
  double vehicle_half_width_m = 1.080418;
  std::vector<geometry_msgs::Point> global_path;
};

class RoundaboutMerge {
 public:
  explicit RoundaboutMerge(RoundaboutMergeParameters parameters) : p_(std::move(parameters)) {}

  Message process(const nav_msgs::Odometry& ego,
                  const interfaces::CompetitionVehicleStatus& competition,
                  const interfaces::Objects& objects,
                  const PredictedObjects& predictions, const ros::Time& stamp) {
    const double observation_time = predictions.front().header.stamp.toSec();
    const double plan_time = stamp.toSec();
    rememberPose(ego, competition);
    auto planned_ego = ego;
    const auto planned_frame = observationFrame(plan_time);
    const Eigen::Quaterniond planned_rotation(planned_frame.rotation());
    planned_ego.pose.pose.position.x = planned_frame.translation().x();
    planned_ego.pose.pose.position.y = planned_frame.translation().y();
    planned_ego.pose.pose.position.z = planned_frame.translation().z();
    planned_ego.pose.pose.orientation.x = planned_rotation.x();
    planned_ego.pose.pose.orientation.y = planned_rotation.y();
    planned_ego.pose.pose.orientation.z = planned_rotation.z();
    planned_ego.pose.pose.orientation.w = planned_rotation.w();
    auto path = rddfPath(planned_ego);
    const auto& projected = path.points.front().pose.pose;
    tracking_padding_ = std::hypot(planned_ego.pose.pose.position.x - projected.position.x,
                                   planned_ego.pose.pose.position.y - projected.position.y) +
        std::hypot(p_.vehicle_front_m, p_.vehicle_half_width_m)*
        std::abs(angle(yaw(planned_ego.pose.pose.orientation) - yaw(projected.orientation)));
    std::vector<double> station(path.points.size(), 0.0);
    for (std::size_t i = 1; i < station.size(); ++i) {
      const auto& a = path.points[i - 1].pose.pose.position;
      const auto& b = path.points[i].pose.pose.position;
      station[i] = station[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
    }
    auto obstacles = tubes(predictions);
    for (const auto& object : objects.objects) {
      Tube current;
      current.boxes.fill(box(object));
      current.start = objects.header.stamp.toSec() - observation_time;
      current.end = current.start + 0.5;
      obstacles.push_back(current);
    }
    const double age = plan_time - observation_time;
    const double horizon = 5.0 - age;
    const double velocity = competition.velocity_x_mps;
    const double measured_time = competition.header.stamp.toSec();
    if (measured_time != last_speed_time_) {
      last_acceleration_ = 0.0;
      if (measured_time > last_speed_time_ && measured_time - last_speed_time_ < 0.5)
        last_acceleration_ = clamp((velocity - last_speed_) / (measured_time - last_speed_time_),
                                   -p_.max_deceleration_mps2, p_.max_acceleration_mps2);
    }
    last_speed_ = velocity;
    last_speed_time_ = measured_time;
    const double acceleration = last_acceleration_;
    const double initial_velocity = std::max(0.0, velocity + acceleration*(plan_time - measured_time));
    auto schedule = search(path, station, obstacles, age, horizon, initial_velocity, acceleration);
    if (schedule.empty()) {
      ROS_WARN_THROTTLE(1.0, "roundabout_merge: no feasible merge trajectory; requesting stop");
      for (auto& point : path.points) {
        point.velocity = 0.0;
        point.acceleration = -p_.max_deceleration_mps2;
      }
      return path;
    }
    const auto terminal = schedule.back();
    const double cap = std::max(p_.desired_speed_mps,
        initial_velocity + std::pow(std::max(0.0, acceleration), 2)/(2*p_.max_jerk_mps3));
    // Beyond prediction coverage, publish only a kinematic braking tail; replan before executing it.
    const auto tail = stoppingTail(terminal, terminal.t + stoppingTime(terminal.v, terminal.a),
                                   cap, path, station, {}, age);
    schedule.insert(schedule.end(), tail.begin() + 1, tail.end());
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto after = std::lower_bound(schedule.begin(), schedule.end(), station[i],
          [](const State& state, double s) { return state.s < s; });
      if (after == schedule.end()) {
        path.points[i].velocity = 0.0;
        path.points[i].acceleration = 0.0;
      } else if (after == schedule.begin()) {
        path.points[i].velocity = after->v;
        path.points[i].acceleration = after->a;
      } else {
        const auto& before = *(after - 1);
        const Edge edge{before, after->t - before.t, after->v, after->a};
        double lo = 0, hi = edge.h;
        for (int iteration = 0; iteration < 40; ++iteration) {
          const double middle = (lo + hi) / 2;
          if (edge.station(middle) < station[i]) lo = middle;
          else hi = middle;
        }
        path.points[i].velocity = std::max(0.0, edge.velocity(hi));
        path.points[i].acceleration = edge.acceleration(hi);
      }
    }
    // The nearest point carries the next actionable target, rather than measured v(0).
    const auto command = stateAt(schedule, std::min(p_.command_step_sec, horizon));
    path.points.front().velocity = command.v;
    path.points.front().acceleration = command.a;
    return path;
  }

 private:
  struct Pose {
    double x, y, yaw;
  };
  struct EgoPose {
    double time;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d velocity, angular_velocity;
  };
  struct Box {
    Pose pose;
    double half_length, half_width;
  };
  struct Tube {
    std::array<Box, 11> boxes;
    double start = 0.0, end = 5.0;
  };
  struct State {
    double t, s, v, a, cost;
    int parent;
  };
  struct Edge {
    State start;
    double h, v1, a1;
    double c2() const { return 3 * (v1 - start.v) / (h*h) - (2*start.a + a1) / h; }
    double c3() const { return 2 * (start.v - v1) / (h*h*h) + (start.a + a1) / (h*h); }
    double velocity(double t) const { return start.v + t * (start.a + t * (c2() + t*c3())); }
    double acceleration(double t) const { return start.a + t * (2*c2() + 3*t*c3()); }
    double jerk(double t) const { return 2*c2() + 6*t*c3(); }
    double station(double t) const {
      return start.s + t * (start.v + t * (start.a/2 + t * (c2()/3 + t*c3()/4)));
    }
  };

  static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
  static double angle(double x) { return std::atan2(std::sin(x), std::cos(x)); }
  static double yaw(const geometry_msgs::Quaternion& q) {
    return std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  }
  static geometry_msgs::Point interpolate(const geometry_msgs::Point& from,
                                          const geometry_msgs::Point& to, double ratio) {
    geometry_msgs::Point point;
    point.x = from.x + ratio*(to.x - from.x);
    point.y = from.y + ratio*(to.y - from.y);
    point.z = from.z + ratio*(to.z - from.z);
    return point;
  }
  static void appendPoint(Message& path, const geometry_msgs::Point& position) {
    interfaces::PathPoint point;
    point.pose.pose.position = position;
    point.pose.pose.orientation.w = 1;
    path.points.push_back(point);
  }
  Message rddfPath(const nav_msgs::Odometry& ego) const {
    const auto& position = ego.pose.pose.position;
    const auto& points = p_.global_path;
    std::size_t first_segment = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    geometry_msgs::Point projection;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const auto& from = points[i];
      const auto& to = points[i + 1];
      const double dx = to.x - from.x, dy = to.y - from.y;
      const double squared_length = dx*dx + dy*dy;
      if (squared_length == 0.0) continue;
      const double ratio = clamp(((position.x - from.x)*dx + (position.y - from.y)*dy) /
                                 squared_length, 0, 1);
      const auto candidate = interpolate(from, to, ratio);
      const double distance = std::pow(candidate.x - position.x, 2) +
                              std::pow(candidate.y - position.y, 2);
      if (distance < closest_distance) {
        closest_distance = distance;
        first_segment = i;
        projection = candidate;
      }
    }
    Message path;
    appendPoint(path, projection);
    double length_m = 0;
    for (std::size_t i = first_segment + 1;
         i < points.size() && length_m < p_.local_path_length_m; ++i) {
      const auto from = path.points.back().pose.pose.position;
      const auto& to = points[i];
      const double segment_m = std::hypot(to.x - from.x, to.y - from.y);
      if (segment_m == 0.0) continue;
      const double take_m = std::min(segment_m, p_.local_path_length_m - length_m);
      appendPoint(path, interpolate(from, to, take_m / segment_m));
      length_m += take_m;
    }
    for (std::size_t i = 0; i + 1 < path.points.size(); ++i) {
      const auto& a = path.points[i].pose.pose.position;
      const auto& b = path.points[i + 1].pose.pose.position;
      const double heading = std::atan2(b.y - a.y, b.x - a.x);
      path.points[i].pose.pose.orientation.z = std::sin(heading / 2);
      path.points[i].pose.pose.orientation.w = std::cos(heading / 2);
    }
    if (path.points.size() > 1)
      path.points.back().pose.pose.orientation = path.points[path.points.size() - 2].pose.pose.orientation;
    return path;
  }

  void rememberPose(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& status) {
    const double time = ego.header.stamp.toSec();
    if (!ego_history_.empty() && time < ego_history_.back().time) ego_history_.clear();
    if (ego_history_.empty() || time >= ego_history_.back().time) {
      if (!ego_history_.empty() && time == ego_history_.back().time) ego_history_.pop_back();
      const auto& p = ego.pose.pose.position;
      const auto& q = ego.pose.pose.orientation;
      ego_history_.push_back({time, {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z},
          {status.velocity_x_mps, ego.twist.twist.linear.y, ego.twist.twist.linear.z},
          {status.angular_velocity_x_radps, status.angular_velocity_y_radps, status.angular_velocity_z_radps}});
      while (ego_history_.size() > 2 && time - ego_history_[1].time > 1.0) ego_history_.pop_front();
    }
  }
  Eigen::Isometry3d observationFrame(double time) const {
    const auto after = std::lower_bound(ego_history_.begin(), ego_history_.end(), time,
        [](const EgoPose& pose, double t) { return pose.time < t; });
    Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
    if (after != ego_history_.begin() && after != ego_history_.end() &&
        after->time - (after - 1)->time <= 0.2) {
      const auto& before = *(after - 1);
      const double u = (time - before.time) / (after->time - before.time);
      frame.linear() = before.rotation.slerp(u, after->rotation).toRotationMatrix();
      frame.translation() = (1 - u)*before.position + u*after->position;
    } else {
      const auto& pose = after == ego_history_.end() ? ego_history_.back() : *after;
      const double dt = time - pose.time;
      const double omega = pose.angular_velocity.norm();
      const double turn = omega*dt;
      const double a = std::abs(turn) < 1e-5 ? dt*dt*(0.5 - turn*turn/24) :
          (1 - std::cos(turn))/(omega*omega);
      const double b = std::abs(turn) < 1e-5 ? dt*dt*dt*(1.0/6 - turn*turn/120) :
          (turn - std::sin(turn))/(omega*omega*omega);
      const auto cross = pose.angular_velocity.cross(pose.velocity);
      frame.translation() = pose.position + pose.rotation *
          (dt*pose.velocity + a*cross + b*pose.angular_velocity.cross(cross));
      const Eigen::Quaterniond delta = omega == 0 ? Eigen::Quaterniond::Identity() :
          Eigen::Quaterniond(Eigen::AngleAxisd(turn, pose.angular_velocity / omega));
      // shortcut: constant body twist outside pose history; use timestamp-matched localization for tighter gaps.
      frame.linear() = (pose.rotation * delta).toRotationMatrix();
    }
    return frame;
  }
  static Box box(const interfaces::Object& object) {
    const auto& p = object.bbox_pose.position;
    const auto& q = object.bbox_pose.orientation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const double heading = std::atan2(2*(rotation.w()*rotation.z() + rotation.x()*rotation.y()),
                                      1 - 2*(rotation.y()*rotation.y() + rotation.z()*rotation.z()));
    const Eigen::Vector3d forward(std::cos(heading), std::sin(heading), 0);
    const Eigen::Vector3d lateral(-std::sin(heading), std::cos(heading), 0);
    const auto axes = rotation.toRotationMatrix();
    const auto& size = object.bbox_size;
    return {{p.x, p.y, heading},
        (std::abs(forward.dot(axes.col(0)))*size.x + std::abs(forward.dot(axes.col(1)))*size.y +
         std::abs(forward.dot(axes.col(2)))*size.z) / 2,
        (std::abs(lateral.dot(axes.col(0)))*size.x + std::abs(lateral.dot(axes.col(1)))*size.y +
         std::abs(lateral.dot(axes.col(2)))*size.z) / 2};
  }
  static std::vector<Tube> tubes(const PredictedObjects& predictions) {
    std::vector<Tube> result;
    for (const auto& object : predictions.front().objects) {
      Tube tube;
      for (std::size_t i = 0; i < predictions.size(); ++i) {
        const auto& list = predictions[i].objects;
        const auto predicted = std::find_if(list.begin(), list.end(),
            [&](const interfaces::Object& item) { return item.id == object.id; });
        tube.boxes[i + 1] = box(*predicted);
      }
      tube.boxes[0] = tube.boxes[1];
      tube.boxes[0].pose.x -= tube.boxes[2].pose.x - tube.boxes[1].pose.x;
      tube.boxes[0].pose.y -= tube.boxes[2].pose.y - tube.boxes[1].pose.y;
      tube.boxes[0].pose.yaw -= angle(tube.boxes[2].pose.yaw - tube.boxes[1].pose.yaw);
      // shortcut: back-project the first half-second linearly; include t=0 in upstream predictions for tighter occupancy.
      result.push_back(tube);
    }
    return result;
  }
  static Box boxAt(const Tube& tube, double time) {
    const double index = clamp(time / 0.5, 0, 10);
    const std::size_t before = std::min<std::size_t>(9, static_cast<std::size_t>(index));
    const double u = index - before;
    const auto& a = tube.boxes[before];
    const auto& b = tube.boxes[before + 1];
    return {{a.pose.x + u*(b.pose.x - a.pose.x), a.pose.y + u*(b.pose.y - a.pose.y),
             a.pose.yaw + u*angle(b.pose.yaw - a.pose.yaw)},
            std::max(a.half_length, b.half_length), std::max(a.half_width, b.half_width)};
  }
  static Pose poseAt(const Message& path, const std::vector<double>& station, double s) {
    const auto after = std::upper_bound(station.begin(), station.end(), s);
    const std::size_t i = std::min<std::size_t>(station.size() - 2, after - station.begin() - 1);
    const double u = clamp((s - station[i]) / (station[i + 1] - station[i]), 0, 1);
    const auto& a = path.points[i].pose.pose;
    const auto& b = path.points[i + 1].pose.pose;
    return {a.position.x + u*(b.position.x - a.position.x),
            a.position.y + u*(b.position.y - a.position.y),
            yaw(a.orientation) + u*angle(yaw(b.orientation) - yaw(a.orientation))};
  }
  Box egoBox(const Pose& pose) const {
    const double offset = (p_.vehicle_front_m - p_.vehicle_rear_m) / 2;
    return {{pose.x + offset*std::cos(pose.yaw), pose.y + offset*std::sin(pose.yaw), pose.yaw},
            (p_.vehicle_front_m + p_.vehicle_rear_m) / 2, p_.vehicle_half_width_m};
  }
  static double support(const Box& box, double axis) {
    return box.half_length*std::abs(std::cos(box.pose.yaw - axis)) +
           box.half_width*std::abs(std::sin(box.pose.yaw - axis));
  }
  static bool sweptOverlap(const Box& a0, const Box& a1, const Box& b0, const Box& b1, double pad) {
    pad += std::hypot(a0.half_length, a0.half_width)*std::abs(angle(a1.pose.yaw - a0.pose.yaw)) +
           std::hypot(b0.half_length, b0.half_width)*std::abs(angle(b1.pose.yaw - b0.pose.yaw));
    double lo = 0, hi = 1;
    for (double axis : {a0.pose.yaw, a0.pose.yaw + M_PI/2, b0.pose.yaw, b0.pose.yaw + M_PI/2}) {
      const double x = std::cos(axis), y = std::sin(axis);
      const double d0 = (b0.pose.x - a0.pose.x)*x + (b0.pose.y - a0.pose.y)*y;
      const double d1 = (b1.pose.x - a1.pose.x)*x + (b1.pose.y - a1.pose.y)*y;
      const double extent = support(a0, axis) + support(b0, axis) + pad;
      const double delta = d1 - d0;
      if (std::abs(delta) < 1e-12) {
        if (std::abs(d0) > extent) return false;
      } else {
        const double enter = (-extent - d0) / delta, leave = (extent - d0) / delta;
        lo = std::max(lo, std::min(enter, leave));
        hi = std::min(hi, std::max(enter, leave));
        if (lo > hi) return false;
      }
    }
    return true;
  }
  static double curvature(const Message& path, const std::vector<double>& station, double a, double b) {
    double maximum = 0;
    for (std::size_t i = 0; i + 1 < station.size(); ++i) {
      if (station[i + 1] < a || station[i] > b) continue;
      const double turn = angle(yaw(path.points[i + 1].pose.pose.orientation) -
                                yaw(path.points[i].pose.pose.orientation));
      maximum = std::max(maximum, std::abs(turn) / (station[i + 1] - station[i]));
    }
    return maximum;
  }
  bool admissible(const Edge& edge, double cap, double k) const {
    std::vector<double> times{0, edge.h};
    if (std::abs(edge.c3()) > 1e-12) {
      const double t = -edge.c2() / (3*edge.c3());
      if (t > 0 && t < edge.h) times.push_back(t);
    }
    const double a = 3*edge.c3(), b = 2*edge.c2(), c = edge.start.a;
    const double discriminant = b*b - 4*a*c;
    if (std::abs(a) > 1e-12 && discriminant >= 0) {
      for (double t : {(-b - std::sqrt(discriminant))/(2*a), (-b + std::sqrt(discriminant))/(2*a)})
        if (t > 0 && t < edge.h) times.push_back(t);
    } else if (std::abs(b) > 1e-12) {
      const double t = -c / b;
      if (t > 0 && t < edge.h) times.push_back(t);
    }
    for (double t : times) {
      const double v = edge.velocity(t), acceleration = edge.acceleration(t);
      if (v < -1e-9 || v > cap + 1e-9 || acceleration < -p_.max_deceleration_mps2 - 1e-9 ||
          acceleration > p_.max_acceleration_mps2 + 1e-9 || std::abs(edge.jerk(t)) > p_.max_jerk_mps3 + 1e-9 ||
          v*v*k > p_.max_lateral_acceleration_mps2 + 1e-9) return false;
    }
    return true;
  }
  bool clear(const Edge& edge, const Message& path, const std::vector<double>& station,
             const std::vector<Tube>& obstacles, double age) const {
    if (obstacles.empty()) return true;
    for (double t = 0; t < edge.h - 1e-10;) {
      double end = std::min(edge.h, t + 0.05);
      const double next_slice = (std::floor((age + edge.start.t + t) / 0.5 + 1e-9) + 1)*0.5;
      end = std::min(end, next_slice - age - edge.start.t);
      const auto knot = std::upper_bound(station.begin(), station.end(), edge.station(t) + 1e-9);
      if (knot != station.end() && edge.station(end) > *knot) {
        double lo = t, hi = end;
        for (int i = 0; i < 40; ++i) {
          const double middle = (lo + hi) / 2;
          if (edge.station(middle) < *knot) lo = middle;
          else hi = middle;
        }
        end = hi;
      }
      const double s0 = edge.station(t), s1 = edge.station(end);
      const auto a0 = egoBox(poseAt(path, station, s0));
      const auto a1 = egoBox(poseAt(path, station, s1));
      const double k = curvature(path, station, s0, s1);
      const double dt = end - t;
      const double vmax = std::max(edge.velocity(t), edge.velocity(end)) + p_.max_acceleration_mps2*dt;
      const double deviation = (std::max(p_.max_acceleration_mps2, p_.max_deceleration_mps2) + vmax*vmax*k)*dt*dt/8;
      const double offset = std::abs(p_.vehicle_front_m - p_.vehicle_rear_m) / 2;
      const double pad = p_.collision_margin_m + tracking_padding_ +
                         p_.prediction_uncertainty_mps*(age + edge.start.t + end) +
                         deviation + 2*offset*std::abs(angle(a1.pose.yaw - a0.pose.yaw));
      for (const auto& tube : obstacles) {
        const double begin_time = std::max(t, tube.start - age - edge.start.t);
        const double end_time = std::min(end, tube.end - age - edge.start.t);
        if (begin_time > end_time) continue;
        const auto from = egoBox(poseAt(path, station, edge.station(begin_time)));
        const auto to = egoBox(poseAt(path, station, edge.station(end_time)));
        if (sweptOverlap(from, to, boxAt(tube, age + edge.start.t + begin_time),
                         boxAt(tube, age + edge.start.t + end_time), pad)) return false;
      }
      t = end;
    }
    return true;
  }
  double stoppingTime(double v, double a) const {
    const double jerk = p_.max_jerk_mps3, deceleration = p_.max_deceleration_mps2;
    const double peak = std::sqrt(jerk*v + a*a/2);
    if (a < 0 && peak < -a - 1e-9) return std::numeric_limits<double>::infinity();
    if (peak <= deceleration) return (a + 2*peak) / jerk;
    return (a + 2*deceleration) / jerk + (v + (a*a - 2*deceleration*deceleration)/(2*jerk))/deceleration;
  }
  double stoppingDistance(double v, double a) const {
    if (!std::isfinite(stoppingTime(v, a))) return std::numeric_limits<double>::infinity();
    const double jerk = p_.max_jerk_mps3, d = p_.max_deceleration_mps2;
    const double peak = std::min(d, std::sqrt(jerk*v + a*a/2));
    const double first = (a + peak)/jerk;
    double distance = v*first + a*first*first/2 - jerk*first*first*first/6;
    v += a*first - jerk*first*first/2;
    const double middle = peak > 0 ? std::max(0.0, (v - peak*peak/(2*jerk))/peak) : 0;
    if (peak > 0) {
      distance += v*middle - peak*middle*middle/2;
      v -= peak*middle;
    }
    const double last = peak/jerk;
    return distance + v*last - peak*last*last/2 + jerk*last*last*last/6;
  }
  std::vector<State> stoppingTail(const State& start, double horizon, double cap,
                                  const Message& path, const std::vector<double>& station,
                                  const std::vector<Tube>& obstacles, double age) const {
    if (stoppingTime(start.v, start.a) > horizon - start.t + 1e-9) return {};
    const double jerk = p_.max_jerk_mps3, deceleration = p_.max_deceleration_mps2;
    const double peak = std::sqrt(jerk*start.v + start.a*start.a/2);
    std::vector<std::pair<double, double>> phases;
    if (peak <= deceleration) {
      phases = {{(start.a + peak)/jerk, -jerk}, {peak/jerk, jerk}};
    } else {
      phases = {{(start.a + deceleration)/jerk, -jerk},
                {(start.v + (start.a*start.a - 2*deceleration*deceleration)/(2*jerk))/deceleration, 0},
                {deceleration/jerk, jerk}};
    }
    State q = start;
    std::vector<State> result{q};
    const auto advance = [&](double duration, double rate) {
      for (double remaining = duration; remaining > 1e-10;) {
        const double h = std::min(p_.time_step_sec, remaining);
        const double v = q.v + q.a*h + rate*h*h/2;
        const double a = q.a + rate*h;
        const Edge edge{q, h, std::abs(v) < 1e-9 ? 0 : v, std::abs(a) < 1e-9 ? 0 : a};
        const double s = edge.station(h);
        if (s > station.back() + 1e-9 || !admissible(edge, cap, curvature(path, station, q.s, s)) ||
            !clear(edge, path, station, obstacles, age)) return false;
        const double cost = q.cost + h*(std::pow(p_.desired_speed_mps - edge.v1, 2) +
            0.1*edge.a1*edge.a1 + 0.02*(std::pow(edge.jerk(0), 2) + std::pow(edge.jerk(h), 2))) - 2*(s - q.s);
        q = {q.t + h, s, edge.v1, edge.a1, cost, -1};
        result.push_back(q);
        remaining -= h;
      }
      return true;
    };
    for (const auto& phase : phases)
      if (!advance(phase.first, phase.second)) return {};
    if (!advance(std::max(0.0, horizon - q.t), 0)) return {};
    return result;
  }
  static std::vector<State> trace(const std::vector<std::vector<State>>& layers, int index) {
    std::vector<State> result;
    for (std::size_t layer = layers.size(); layer-- > 0;) {
      const auto q = layers[layer][static_cast<std::size_t>(index)];
      result.push_back(q);
      index = q.parent;
    }
    std::reverse(result.begin(), result.end());
    return result;
  }
  std::vector<State> search(const Message& path, const std::vector<double>& station,
                            const std::vector<Tube>& obstacles, double age, double horizon,
                            double v0, double a0) const {
    if (station.size() < 2 || horizon <= 0) return {};
    const double cap = std::max(p_.desired_speed_mps,
        v0 + std::pow(std::max(0.0, a0), 2)/(2*p_.max_jerk_mps3));
    std::vector<std::vector<State>> layers{{{0, 0, v0, a0, 0, -1}}};
    std::vector<State> best;
    for (double time = 0; time < horizon - 1e-9;) {
      const double h = std::min(p_.time_step_sec, horizon - time);
      using Key = std::tuple<long long, long long, long long>;
      std::map<Key, State> cells;
      const auto& previous = layers.back();
      const auto braking = std::min_element(previous.begin(), previous.end(),
          [&](const State& a, const State& b) { return stoppingTime(a.v, a.a) < stoppingTime(b.v, b.a); });
      // Analytic braking phases retain a feasible terminal stop between lattice timestamps.
      for (int index : {0, static_cast<int>(braking - previous.begin())}) {
        auto tail = stoppingTail(previous[index], horizon, cap, path, station, obstacles, age);
        if (!tail.empty() && (best.empty() || tail.back().cost < best.back().cost)) {
          best = trace(layers, index);
          best.insert(best.end(), tail.begin() + 1, tail.end());
        }
      }
      for (std::size_t index = 0; index < previous.size(); ++index) {
        const auto& q = previous[index];
        const double lo_a = std::max(-p_.max_deceleration_mps2, q.a - p_.max_jerk_mps3*h);
        const double hi_a = std::min(p_.max_acceleration_mps2, q.a + p_.max_jerk_mps3*h);
        for (double a : {lo_a, hi_a, clamp(0, lo_a, hi_a), clamp(q.a, lo_a, hi_a)}) {
          const double lo_v = std::max(0.0, q.v + std::max((4*q.a + 2*a)*h - p_.max_jerk_mps3*h*h,
                                                        (2*q.a + 4*a)*h - p_.max_jerk_mps3*h*h)/6);
          const double hi_v = std::min(cap, q.v + std::min((4*q.a + 2*a)*h + p_.max_jerk_mps3*h*h,
                                                         (2*q.a + 4*a)*h + p_.max_jerk_mps3*h*h)/6);
          if (lo_v > hi_v + 1e-10) continue;
          for (double v : {lo_v, std::max(lo_v, hi_v), clamp(0, lo_v, std::max(lo_v, hi_v)),
                           clamp(q.v + (q.a + a)*h/2, lo_v, std::max(lo_v, hi_v)),
                           clamp(p_.desired_speed_mps, lo_v, std::max(lo_v, hi_v))}) {
            const Edge edge{q, h, v, a};
            const double s = edge.station(h);
            if (s + stoppingDistance(v, a) > station.back() + 1e-9 ||
                !admissible(edge, cap, curvature(path, station, q.s, s)) ||
                !clear(edge, path, station, obstacles, age)) continue;
            if (time + h >= horizon - 1e-9) {
              const State terminal{time + h, s, v, a, 0, -1};
              if (stoppingTail(terminal, terminal.t + stoppingTime(v, a), cap,
                               path, station, {}, age).empty()) continue;
            }
            const double cost = q.cost + h*(std::pow(p_.desired_speed_mps - v, 2) +
                0.1*a*a + 0.02*(std::pow(edge.jerk(0), 2) + std::pow(edge.jerk(h), 2))) - 2*(s - q.s);
            const State next{time + h, s, v, a, cost, static_cast<int>(index)};
            const auto velocity_cell = v == 0 ? 0LL : 1 + std::llround(v / 0.125);
            const auto acceleration_cell = a == 0 ? 0LL : (a > 0 ? 1LL : -1LL) + std::llround(a / 0.25);
            const Key key{std::llround(s / 0.25), velocity_cell, acceleration_cell};
            const auto found = cells.find(key);
            if (found == cells.end() || cost < found->second.cost) cells[key] = next;
          }
        }
      }
      std::vector<State> next;
      for (const auto& cell : cells) next.push_back(cell.second);
      std::sort(next.begin(), next.end(), [](const State& a, const State& b) { return a.cost < b.cost; });
      if (next.empty()) break;
      if (next.size() > static_cast<std::size_t>(p_.beam_width)) {
        // shortcut: finite beam can miss a feasible gap; widen it or use an exhaustive lattice when required.
        const auto braking = std::min_element(next.begin(), next.end(),
            [&](const State& a, const State& b) { return stoppingTime(a.v, a.a) < stoppingTime(b.v, b.a); });
        const State keep_braking = *braking;
        const auto stopped = std::find_if(next.begin(), next.end(),
            [](const State& q) { return std::abs(q.v) < 1e-9 && std::abs(q.a) < 1e-9; });
        const bool have_stopped = stopped != next.end();
        const State keep = have_stopped ? *stopped : next.front();
        next.resize(p_.beam_width);
        next.back() = keep_braking;
        if (have_stopped && next.size() > 1) next[next.size() - 2] = keep;
      }
      layers.push_back(std::move(next));
      time += h;
    }
    if (layers.back().front().t >= horizon - 1e-9 &&
        (best.empty() || layers.back().front().cost < best.back().cost)) return trace(layers, 0);
    return best;
  }
  static State stateAt(const std::vector<State>& schedule, double time) {
    const auto after = std::upper_bound(schedule.begin(), schedule.end(), time,
        [](double t, const State& state) { return t < state.t; });
    if (after == schedule.end()) return schedule.back();
    const auto& before = *(after - 1);
    const Edge edge{before, after->t - before.t, after->v, after->a};
    const double t = time - before.t;
    return {time, edge.station(t), std::max(0.0, edge.velocity(t)), edge.acceleration(t), 0, -1};
  }

  RoundaboutMergeParameters p_;
  std::deque<EgoPose> ego_history_;
  double last_speed_ = 0;
  double last_speed_time_ = -std::numeric_limits<double>::infinity();
  double last_acceleration_ = 0;
  double tracking_padding_ = 0;
};
}  // namespace path_planner
