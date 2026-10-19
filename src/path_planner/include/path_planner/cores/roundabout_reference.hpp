#pragma once
#include <geometry_msgs/Point.h>
#include <path_planner/libraries/types.hpp>
#include <Eigen/Geometry>
#include <ros/console.h>
#include <array>
#include <deque>
#include <map>
#include <tuple>
#include <interfaces/Path.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace path_planner {
struct RoundaboutReferenceParameters {
  double reference_start_m = 820.0;
  double reference_end_m = 980.0;
  double control_step_m = 2.0;
  double integration_step_m = 0.05;
  double sample_step_m = 0.25;
  double local_path_length_m = 40.0;
  double entry_shift_m = 0.15;
  double entry_shift_center_m = 886.0;
  double entry_shift_half_length_m = 7.0;
  double exit_shift_m = 0.4;
  double exit_shift_center_m = 914.0;
  double exit_shift_half_length_m = 9.0;
  double desired_speed_mps = 5.0;
  double approach_speed_mps = 3.0;
  double entered_min_speed_mps = 1.0;
  double curve_speed_ratio = 0.75;
  double entry_line_station_m = 885.1006875594753;
  double entry_stop_margin_m = 0.8;
  double entry_decision_distance_m = 2.0;
  double entry_check_length_m = 6.0;
  double entry_gap_confirmation_sec = 0.1;
  double planning_horizon_sec = 3.0;
  double max_acceleration_mps2 = 2.0;
  double max_deceleration_mps2 = 4.0;
  double max_jerk_mps3 = 4.0;
  double max_lateral_acceleration_mps2 = 2.0;
  double time_step_sec = 0.25;
  double command_step_sec = 0.8;
  int beam_width = 64;
  double collision_margin_m = 0.5;
  double prediction_uncertainty_mps = 0.2;
  double vehicle_front_m = 3.845008;
  double vehicle_rear_m = 0.790000;
  double vehicle_half_width_m = 1.080418;
  double following_time_gap_sec = 1.2;
  std::vector<geometry_msgs::Point> global_path;
};

class RoundaboutReference {
 public:
  explicit RoundaboutReference(RoundaboutReferenceParameters parameters) : p_(std::move(parameters)) {
    raw_.push_back(p_.global_path.front());
    raw_station_.push_back(0.0);
    for (std::size_t i = 1; i < p_.global_path.size(); ++i) {
      const auto& point = p_.global_path[i];
      const double length_m = distance(raw_.back(), point);
      if (length_m > 0.0) {
        raw_station_.push_back(raw_station_.back() + length_m);
        raw_.push_back(point);
      }
    }
    const double span_m = p_.reference_end_m - p_.reference_start_m;
    const std::size_t cells = static_cast<std::size_t>(std::ceil(span_m / p_.control_step_m));
    control_step_m_ = span_m / cells;
    for (std::size_t i = 0; i <= cells; ++i)
      controls_.push_back(rawPoint(p_.reference_start_m + control_step_m_ * i));
    const auto original = controls_;
    for (std::size_t i = 0; i < controls_.size(); ++i) {
      const double station_m = p_.reference_start_m + control_step_m_ * i;
      const auto& before = original[i == 0 ? i : i - 1];
      const auto& after = original[std::min(i + 1, original.size() - 1)];
      const double heading = std::atan2(after.y - before.y, after.x - before.x);
      const double shift_m = clearanceShift(station_m, p_.entry_shift_center_m,
          p_.entry_shift_half_length_m, p_.entry_shift_m) + clearanceShift(station_m,
          p_.exit_shift_center_m, p_.exit_shift_half_length_m, p_.exit_shift_m);
      controls_[i].x += shift_m * std::sin(heading);
      controls_[i].y -= shift_m * std::cos(heading);
    }
    const std::size_t samples = static_cast<std::size_t>(std::ceil(span_m / p_.integration_step_m));
    for (std::size_t i = 0; i <= samples; ++i) {
      Sample sample;
      sample.u = static_cast<double>(cells) * i / samples;
      sample.point = evaluate(sample.u).pose.pose.position;
      if (!samples_.empty()) sample.station_m = samples_.back().station_m + distance(samples_.back().point, sample.point);
      samples_.push_back(sample);
    }
    entry_line_m_ = projectStation(rawPoint(p_.entry_line_station_m));
    yield_station_m_ = entry_line_m_ - p_.vehicle_front_m - p_.entry_stop_margin_m;
    departure_station_m_ = projectStation(rawPoint(p_.exit_shift_center_m));
  }

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
    auto path = referencePath(planned_ego);
    if (path.points.size() == 1) {
      const bool past_entry = entered_ || projectStation(path.points.front().pose.pose.position) >=
          entry_line_m_ - p_.vehicle_front_m;
      path.points.front().velocity = past_entry ? p_.entered_min_speed_mps : 0.0;
      path.points.front().acceleration = past_entry ? 0.0 : -p_.max_deceleration_mps2;
      return path;
    }
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
    path_yaw_.resize(path.points.size());
    path_turn_.resize(path.points.size() - 1);
    path_curvature_.resize(path.points.size() - 1);
    for (std::size_t i = 0; i < path.points.size(); ++i)
      path_yaw_[i] = yaw(path.points[i].pose.pose.orientation);
    for (std::size_t i = 0; i + 1 < path.points.size(); ++i) {
      path_turn_[i] = angle(path_yaw_[i + 1] - path_yaw_[i]);
      path_curvature_[i] = std::abs(path_turn_[i]) / (station[i + 1] - station[i]);
    }
    auto obstacles = tubes(predictions);
    for (const auto& object : objects.objects) {
      Tube current;
      current.boxes.fill(box(object));
      current.start = objects.header.stamp.toSec() - observation_time;
      current.end = current.start + 0.5;
      prepareTube(current);
      obstacles.push_back(current);
    }
    const double age = plan_time - observation_time;
    const double horizon = std::min(p_.planning_horizon_sec, 5.0 - age);
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
    const double current_station = projectStation(projected.position);
    if (current_station < previous_station_m_ - 5.0) {
      entered_ = false;
      gate_open_ = false;
      gap_since_sec_ = -1.0;
    }
    previous_station_m_ = current_station;
    if (current_station >= entry_line_m_ - p_.vehicle_front_m) entered_ = true;
    const double distance_to_yield = yield_station_m_ - current_station;
    std::vector<State> schedule;
    const auto plan = [&](double desired_speed, double stop_station) {
      planning_end_m_ = std::min(station.back(), std::max(0.0, stop_station));
      planning_speed_mps_ = desired_speed;
      // Local curvature targets with braking preview; a bend far ahead must not
      // impose one cruise speed on the whole merge and its straight departure.
      path_speed_.assign(station.size(), desired_speed);
      for (std::size_t i = 0; i < path_curvature_.size(); ++i)
        if (path_curvature_[i] > 0.0)
          path_speed_[i] = std::min(desired_speed, p_.curve_speed_ratio*
              std::sqrt(p_.max_lateral_acceleration_mps2/path_curvature_[i]));
      for (std::size_t i = station.size() - 1; i > 0; --i)
        path_speed_[i - 1] = std::min(path_speed_[i - 1], std::sqrt(path_speed_[i]*path_speed_[i] +
            p_.max_deceleration_mps2*(station[i] - station[i - 1])));
      return search(path, station, obstacles, age, horizon, initial_velocity, acceleration);
    };
    Phase phase = Phase::Approach;
    if (entered_) {
      schedule = plan(p_.desired_speed_mps, station.back());
      phase = current_station >= departure_station_m_ ? Phase::Depart : Phase::Follow;
    } else {
      if (distance_to_yield <= p_.entry_decision_distance_m) {
        auto entry = plan(p_.desired_speed_mps, station.back());
        const double clearance = distance_to_yield + p_.entry_check_length_m;
        const bool clear_entry = !entry.empty() && entry.back().s >= clearance && entry.back().v > 0.5;
        if (clear_entry) {
          if (gap_since_sec_ < 0.0 || observation_time < gap_since_sec_)
            gap_since_sec_ = observation_time;
          if (objects.objects.empty() && predictions.front().objects.empty()) gate_open_ = true;
          if (observation_time - gap_since_sec_ >= p_.entry_gap_confirmation_sec) gate_open_ = true;
        } else {
          gate_open_ = false;
          gap_since_sec_ = -1.0;
        }
        if (gate_open_) {
          schedule = std::move(entry);
          phase = Phase::Enter;
        } else {
          phase = Phase::Yield;
        }
      } else {
        gate_open_ = false;
        gap_since_sec_ = -1.0;
      }
      if (!gate_open_) schedule = plan(p_.approach_speed_mps, distance_to_yield);
    }
    if (phase != phase_) {
      static const char* names[] = {"approach", "yield", "enter", "follow", "depart"};
      ROS_INFO("roundabout_reference: %s, rear axle to yield target=%.2f m",
               names[static_cast<int>(phase)], distance_to_yield);
      phase_ = phase;
    }
    if (schedule.empty()) {
      if (entered_)
        ROS_ERROR_THROTTLE(1.0, "roundabout_reference: no collision-free moving trajectory; "
            "non-stop request requires unverified %.2f m/s rolling target", p_.entered_min_speed_mps);
      else
        ROS_WARN_THROTTLE(1.0, "roundabout_reference: no feasible merge trajectory; requesting stop");
      const double rolling_acceleration = clamp((p_.entered_min_speed_mps - initial_velocity)/p_.command_step_sec,
          -p_.max_deceleration_mps2, p_.max_acceleration_mps2);
      for (auto& point : path.points) {
        point.velocity = entered_ ? p_.entered_min_speed_mps : 0.0;
        point.acceleration = entered_ ? rolling_acceleration : -p_.max_deceleration_mps2;
      }
      return path;
    }
    const auto terminal = schedule.back();
    const double cap = std::max(planning_speed_mps_,
        initial_velocity + std::pow(std::max(0.0, acceleration), 2)/(2*p_.max_jerk_mps3));
    // The terminal braking continuation is collision-checked over all remaining
    // prediction coverage, including the interval after the shorter search horizon.
    const auto tail = terminalTail(terminal, terminal.t + transitionTime(terminal.v, terminal.a),
                                   cap, path, station, obstacles, age);
    schedule.insert(schedule.end(), tail.begin() + 1, tail.end());
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto after = std::lower_bound(schedule.begin(), schedule.end(), station[i],
          [](const State& state, double s) { return state.s < s; });
      if (after == schedule.end()) {
        path.points[i].velocity = minimumSpeed();
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
    // The controller reads this actionable target; keep it below the current cruise/curve limit.
    const auto command = stateAt(schedule, std::min(p_.command_step_sec, horizon));
    path.points.front().velocity = std::min(command.v, speedTarget(station, initial_velocity*p_.command_step_sec));
    path.points.front().acceleration = command.a;
    if (entered_) {
      for (auto& point : path.points) {
        if (point.velocity < p_.entered_min_speed_mps) {
          point.velocity = p_.entered_min_speed_mps;
          point.acceleration = std::max(0.0, point.acceleration);
        }
      }
    }
    return path;
  }

 private:
  enum class Phase { Approach, Yield, Enter, Follow, Depart };

  double projectStation(const geometry_msgs::Point& position) const {
    double closest = std::numeric_limits<double>::infinity(), station = 0.0;
    for (std::size_t i = 0; i + 1 < samples_.size(); ++i) {
      const auto& a = samples_[i];
      const auto& b = samples_[i + 1];
      const double dx = b.point.x - a.point.x, dy = b.point.y - a.point.y;
      const double u = clamp(((position.x-a.point.x)*dx + (position.y-a.point.y)*dy)/(dx*dx+dy*dy), 0, 1);
      const double squared = std::pow(a.point.x+u*dx-position.x, 2) +
                             std::pow(a.point.y+u*dy-position.y, 2);
      if (squared < closest) {
        closest = squared;
        station = a.station_m + u*(b.station_m-a.station_m);
      }
    }
    return station;
  }

  interfaces::Path referencePath(const nav_msgs::Odometry& ego) const {
    const auto& position = ego.pose.pose.position;
    double closest_squared_m = std::numeric_limits<double>::infinity();
    double start_m = 0.0;
    for (std::size_t i = 0; i + 1 < samples_.size(); ++i) {
      const auto& a = samples_[i];
      const auto& b = samples_[i + 1];
      const double dx = b.point.x - a.point.x, dy = b.point.y - a.point.y;
      const double ratio = std::max(0.0, std::min(1.0,
          ((position.x - a.point.x) * dx + (position.y - a.point.y) * dy) / (dx * dx + dy * dy)));
      const double error_x = a.point.x + ratio * dx - position.x;
      const double error_y = a.point.y + ratio * dy - position.y;
      const double squared_m = error_x * error_x + error_y * error_y;
      if (squared_m < closest_squared_m) {
        closest_squared_m = squared_m;
        start_m = a.station_m + ratio * (b.station_m - a.station_m);
      }
    }
    const double end_m = std::min(start_m + p_.local_path_length_m, samples_.back().station_m);
    interfaces::Path path;
    const std::size_t count = static_cast<std::size_t>(std::floor((end_m - start_m) / p_.sample_step_m));
    path.points.reserve(count + 2);
    for (std::size_t i = 0; i <= count; ++i)
      path.points.push_back(evaluate(parameterAt(std::min(start_m + i * p_.sample_step_m, end_m))));
    if (end_m - start_m - count * p_.sample_step_m > 1e-8)
      path.points.push_back(evaluate(parameterAt(end_m)));
    else
      path.points.back() = evaluate(parameterAt(end_m));
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
    std::array<double, 10> turn;
    double min_x, max_x, min_y, max_y;
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
      prepareTube(tube);
      result.push_back(tube);
    }
    return result;
  }
  static void prepareTube(Tube& tube) {
    double radius = 0.0, turn = 0.0;
    tube.min_x = tube.min_y = std::numeric_limits<double>::infinity();
    tube.max_x = tube.max_y = -std::numeric_limits<double>::infinity();
    double length = 0.0, width = 0.0;
    for (std::size_t i = 0; i < tube.boxes.size(); ++i) {
      const auto& box = tube.boxes[i];
      tube.min_x = std::min(tube.min_x, box.pose.x);
      tube.max_x = std::max(tube.max_x, box.pose.x);
      tube.min_y = std::min(tube.min_y, box.pose.y);
      tube.max_y = std::max(tube.max_y, box.pose.y);
      length = std::max(length, box.half_length);
      width = std::max(width, box.half_width);
      if (i + 1 < tube.boxes.size()) {
        tube.turn[i] = angle(tube.boxes[i + 1].pose.yaw - box.pose.yaw);
        turn = std::max(turn, std::abs(tube.turn[i]));
      }
    }
    // boxAt uses maximum endpoint extents. Include those extents and the SAT angular padding.
    radius = std::hypot(length, width) * (1.0 + std::sqrt(2.0)*turn);
    tube.min_x -= radius;
    tube.max_x += radius;
    tube.min_y -= radius;
    tube.max_y += radius;
  }
  static Box boxAt(const Tube& tube, double time) {
    const double index = clamp(time / 0.5, 0, 10);
    const std::size_t before = std::min<std::size_t>(9, static_cast<std::size_t>(index));
    const double u = index - before;
    const auto& a = tube.boxes[before];
    const auto& b = tube.boxes[before + 1];
    return {{a.pose.x + u*(b.pose.x - a.pose.x), a.pose.y + u*(b.pose.y - a.pose.y),
             a.pose.yaw + u*tube.turn[before]},
            std::max(a.half_length, b.half_length), std::max(a.half_width, b.half_width)};
  }
  Pose poseAt(const Message& path, const std::vector<double>& station, double s) const {
    const auto after = std::upper_bound(station.begin(), station.end(), s);
    const std::size_t i = std::min<std::size_t>(station.size() - 2, after - station.begin() - 1);
    const double u = clamp((s - station[i]) / (station[i + 1] - station[i]), 0, 1);
    const auto& a = path.points[i].pose.pose;
    const auto& b = path.points[i + 1].pose.pose;
    return {a.position.x + u*(b.position.x - a.position.x),
            a.position.y + u*(b.position.y - a.position.y),
            path_yaw_[i] + u*path_turn_[i]};
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
  double curvature(const Message&, const std::vector<double>& station, double a, double b) const {
    double maximum = 0;
    const auto first = std::lower_bound(station.begin() + 1, station.end(), a);
    for (std::size_t i = first - station.begin() - 1;
         i + 1 < station.size() && station[i] <= b; ++i) {
      if (station[i + 1] < a) continue;
      maximum = std::max(maximum, path_curvature_[i]);
    }
    return maximum;
  }
  double speedTarget(const std::vector<double>& station, double s) const {
    const auto after = std::upper_bound(station.begin(), station.end(), s);
    const auto index = static_cast<std::size_t>(after - station.begin());
    return path_speed_[index == 0 ? 0 : index - 1];
  }
  bool admissible(const Edge& edge, double cap, double k) const {
    std::vector<double> times{0, edge.h};
    const double activation_time = p_.command_step_sec - edge.start.t;
    if (entered_ && activation_time > 0.0 && activation_time < edge.h)
      times.push_back(activation_time);
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
      if ((entered_ && edge.start.t + t >= p_.command_step_sec &&
           v < p_.entered_min_speed_mps - 1e-9) || v < -1e-9 || v > cap + 1e-9 || acceleration < -p_.max_deceleration_mps2 - 1e-9 ||
          acceleration > p_.max_acceleration_mps2 + 1e-9 || std::abs(edge.jerk(t)) > p_.max_jerk_mps3 + 1e-9 ||
          v*v*k > p_.max_lateral_acceleration_mps2 + 1e-9) return false;
    }
    return true;
  }
  bool clear(const Edge& edge, const Message& path, const std::vector<double>& station,
             const std::vector<Tube>& obstacles, double age) const {
    if (obstacles.empty()) return true;
    const double first_station = edge.start.s, last_station = edge.station(edge.h);
    const auto first_pose = poseAt(path, station, first_station);
    const auto last_pose = poseAt(path, station, last_station);
    double min_x = std::min(first_pose.x, last_pose.x), max_x = std::max(first_pose.x, last_pose.x);
    double min_y = std::min(first_pose.y, last_pose.y), max_y = std::max(first_pose.y, last_pose.y);
    for (auto it = std::upper_bound(station.begin(), station.end(), first_station);
         it != station.end() && *it < last_station; ++it) {
      const auto& point = path.points[it - station.begin()].pose.pose.position;
      min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
    }
    const double k = curvature(path, station, first_station, last_station);
    const double turn = std::min(M_PI, k * std::max(0.0, last_station - first_station));
    const double vmax = std::max(edge.start.v, edge.v1) + p_.max_acceleration_mps2 * (edge.h + 0.05);
    const double gap = p_.following_time_gap_sec * vmax;
    const double radius = std::hypot(std::max(p_.vehicle_front_m, p_.vehicle_rear_m),
                                     p_.vehicle_half_width_m);
    const double box_radius = std::hypot((p_.vehicle_front_m + p_.vehicle_rear_m) / 2 + gap/2,
                                         p_.vehicle_half_width_m);
    // Bound every narrow-phase slice, including follow distance and all existing safety padding.
    const double padding = p_.collision_margin_m + tracking_padding_ +
        p_.prediction_uncertainty_mps * (age + edge.start.t + edge.h) +
        (std::max(p_.max_acceleration_mps2, p_.max_deceleration_mps2) + vmax*vmax*k)*0.05*0.05/8 +
        (std::abs(p_.vehicle_front_m - p_.vehicle_rear_m) + box_radius)*turn;
    // Padding is applied along rotated SAT axes; sqrt(2) also encloses their corners in XY.
    const double expansion = radius + gap + std::sqrt(2.0)*padding + 1e-8;
    std::vector<const Tube*> candidates;
    candidates.reserve(obstacles.size());
    for (const auto& tube : obstacles) {
      if (tube.end < age + edge.start.t || tube.start > age + edge.start.t + edge.h) continue;
      if (tube.max_x < min_x - expansion || tube.min_x > max_x + expansion ||
          tube.max_y < min_y - expansion || tube.min_y > max_y + expansion) continue;
      candidates.push_back(&tube);
    }
    if (candidates.empty()) return true;
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
      for (const auto* candidate : candidates) {
        const auto& tube = *candidate;
        const double begin_time = std::max(t, tube.start - age - edge.start.t);
        const double end_time = std::min(end, tube.end - age - edge.start.t);
        if (begin_time > end_time) continue;
        auto from = begin_time == t ? a0 : egoBox(poseAt(path, station, edge.station(begin_time)));
        auto to = end_time == end ? a1 : egoBox(poseAt(path, station, edge.station(end_time)));
        const auto other_from = boxAt(tube, age + edge.start.t + begin_time);
        const auto other_to = boxAt(tube, age + edge.start.t + end_time);
        const double ahead = (other_from.pose.x - from.pose.x)*std::cos(from.pose.yaw) +
                             (other_from.pose.y - from.pose.y)*std::sin(from.pose.yaw);
        if (ahead > 0 && std::abs(angle(other_from.pose.yaw - from.pose.yaw)) < M_PI/3) {
          // Extend only the front envelope: standing/crossing traffic still uses geometric clearance.
          const double gap = p_.following_time_gap_sec *
              std::max(edge.velocity(begin_time), edge.velocity(end_time));
          for (auto* footprint : {&from, &to}) {
            footprint->pose.x += gap/2*std::cos(footprint->pose.yaw);
            footprint->pose.y += gap/2*std::sin(footprint->pose.yaw);
            footprint->half_length += gap/2;
          }
        }
        if (sweptOverlap(from, to, other_from, other_to, pad)) return false;
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
  double minimumSpeed() const { return entered_ ? p_.entered_min_speed_mps : 0.0; }
  double transitionTime(double v, double a) const {
    // Before entry the terminal state is a stop; after entry it is rolling.
    return v + 1e-9 < minimumSpeed() ? std::numeric_limits<double>::infinity() :
        stoppingTime(std::max(0.0, v - minimumSpeed()), a);
  }
  std::vector<State> terminalTail(const State& start, double horizon, double cap,
                                  const Message& path, const std::vector<double>& station,
                                  const std::vector<Tube>& obstacles, double age) const {
    const double transition_time = transitionTime(start.v, start.a);
    if (!std::isfinite(transition_time) || transition_time > horizon - start.t + 1e-9) return {};
    const double jerk = p_.max_jerk_mps3, deceleration = p_.max_deceleration_mps2;
    const double relative_speed = std::max(0.0, start.v - minimumSpeed());
    const double peak = std::sqrt(jerk*relative_speed + start.a*start.a/2);
    std::vector<std::pair<double, double>> phases;
    if (peak <= deceleration) {
      phases = {{(start.a + peak)/jerk, -jerk}, {peak/jerk, jerk}};
    } else {
      phases = {{(start.a + deceleration)/jerk, -jerk},
                {(relative_speed + (start.a*start.a - 2*deceleration*deceleration)/(2*jerk))/deceleration, 0},
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
        if (s > planning_end_m_ + 1e-9 || !admissible(edge, cap, curvature(path, station, q.s, s)) ||
            !clear(edge, path, station, obstacles, age)) return false;
        const double cost = q.cost + h*(std::pow(speedTarget(station, s) - edge.v1, 2) +
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
    const double cap = std::max(planning_speed_mps_,
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
      // Analytic braking phases retain a feasible stop or rolling terminal speed between lattice timestamps.
      for (int index : {0, static_cast<int>(braking - previous.begin())}) {
        auto tail = terminalTail(previous[index], horizon, cap, path, station, obstacles, age);
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
                           clamp(planning_speed_mps_, lo_v, std::max(lo_v, hi_v))}) {
            const Edge edge{q, h, v, a};
            const double s = edge.station(h);
            if (s + stoppingDistance(v, a) > planning_end_m_ + 1e-9 ||
                !admissible(edge, cap, curvature(path, station, q.s, s)) ||
                !clear(edge, path, station, obstacles, age)) continue;
            if (time + h >= horizon - 1e-9) {
              const State terminal{time + h, s, v, a, 0, -1};
              if (terminalTail(terminal, terminal.t + transitionTime(v, a), cap,
                               path, station, obstacles, age).empty()) continue;
            }
            const double cost = q.cost + h*(std::pow(speedTarget(station, s) - v, 2) +
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

  struct Sample {
    double u = 0.0;
    double station_m = 0.0;
    geometry_msgs::Point point;
  };

  static double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
  }

  static double clearanceShift(double station_m, double center_m, double half_length_m, double shift_m) {
    const double q = (station_m - center_m) / half_length_m;
    return std::abs(q) < 1.0 ? shift_m * std::pow(1.0 - q * q, 3) : 0.0;
  }

  geometry_msgs::Point rawPoint(double station_m) const {
    const auto after = std::upper_bound(raw_station_.begin(), raw_station_.end(), station_m);
    const std::size_t i = std::min(static_cast<std::size_t>(after - raw_station_.begin()), raw_.size() - 1);
    const double t = (station_m - raw_station_[i - 1]) / (raw_station_[i] - raw_station_[i - 1]);
    geometry_msgs::Point point;
    point.x = raw_[i - 1].x + t * (raw_[i].x - raw_[i - 1].x);
    point.y = raw_[i - 1].y + t * (raw_[i].y - raw_[i - 1].y);
    point.z = raw_[i - 1].z + t * (raw_[i].z - raw_[i - 1].z);
    return point;
  }

  geometry_msgs::Point control(int index) const {
    if (index >= 0 && index < static_cast<int>(controls_.size())) return controls_[index];
    const auto& a = index < 0 ? controls_.front() : controls_.back();
    const auto& b = index < 0 ? controls_[1] : controls_[controls_.size() - 2];
    geometry_msgs::Point point;
    point.x = 2 * a.x - b.x;
    point.y = 2 * a.y - b.y;
    return point;
  }

  interfaces::PathPoint evaluate(double u) const {
    const int cell = std::min(static_cast<int>(u), static_cast<int>(controls_.size()) - 2);
    const double t = u - cell, v = 1.0 - t;
    const double weights[] = {v * v * v / 6, (3 * t * t * t - 6 * t * t + 4) / 6,
                             (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6, t * t * t / 6};
    const double derivatives[] = {-v * v / 2, 1.5 * t * t - 2 * t,
                                 -1.5 * t * t + t + 0.5, t * t / 2};
    interfaces::PathPoint point;
    double dx = 0.0, dy = 0.0;
    for (int j = 0; j < 4; ++j) {
      const auto c = control(cell + j - 1);
      point.pose.pose.position.x += weights[j] * c.x;
      point.pose.pose.position.y += weights[j] * c.y;
      dx += derivatives[j] * c.x;
      dy += derivatives[j] * c.y;
    }
    point.pose.pose.position.z = rawPoint(p_.reference_start_m + control_step_m_ * u).z;
    const double heading = std::atan2(dy, dx);
    point.pose.pose.orientation.z = std::sin(heading / 2);
    point.pose.pose.orientation.w = std::cos(heading / 2);
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    return point;
  }

  double parameterAt(double station_m) const {
    const auto after = std::upper_bound(samples_.begin(), samples_.end(), station_m,
        [](double value, const Sample& sample) { return value < sample.station_m; });
    const std::size_t i = std::min(static_cast<std::size_t>(after - samples_.begin()), samples_.size() - 1);
    const auto& a = samples_[i - 1];
    const auto& b = samples_[i];
    return a.u + (station_m - a.station_m) * (b.u - a.u) / (b.station_m - a.station_m);
  }

  std::deque<EgoPose> ego_history_;
  double last_speed_ = 0;
  double last_speed_time_ = -std::numeric_limits<double>::infinity();
  double last_acceleration_ = 0;
  double tracking_padding_ = 0;
  double entry_line_m_ = 0.0, yield_station_m_ = 0.0, departure_station_m_ = 0.0;
  double previous_station_m_ = 0.0;
  double gap_since_sec_ = -1.0;
  double planning_speed_mps_ = 0.0, planning_end_m_ = 0.0;
  bool entered_ = false, gate_open_ = false;
  Phase phase_ = Phase::Approach;
  std::vector<double> path_yaw_, path_turn_, path_curvature_, path_speed_;
  RoundaboutReferenceParameters p_;
  std::vector<geometry_msgs::Point> raw_, controls_;
  std::vector<double> raw_station_;
  std::vector<Sample> samples_;
  double control_step_m_ = 0.0;
};
}  // namespace path_planner
