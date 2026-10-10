#pragma once
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <numeric>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace path_planner {
struct NpcHistoryPredictionParameters {
  struct Curve {
    std::string model_label, actor_label;
    double length_m = 0.0;
    double nominal_rate_mps = 0.0;
    double history_sec = 30.0;
    std::vector<Eigen::Vector3d> points;
  };
  std::vector<Curve> curves;
  double nominal_fit_slack_m = 0.02;
  double path_rms_limit_m = 0.5;
};

class NpcHistoryPrediction {
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
                            interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu,
                            Route, PredictedObjects>;
  explicit NpcHistoryPrediction(NpcHistoryPredictionParameters parameters)
      : parameters_(std::move(parameters)) {
    trees_.resize(parameters_.curves.size());
    for (std::size_t i = 0; i < parameters_.curves.size(); ++i) {
      const auto& curve = parameters_.curves[i];
      std::vector<std::size_t> order(curve.points.size());
      std::iota(order.begin(), order.end(), 0);
      buildTree(curve, order, 0, order.size(), 0, trees_[i]);
      by_label_[curve.model_label].push_back(i);
      if (curve.actor_label != curve.model_label) by_label_[curve.actor_label].push_back(i);
    }
  }

  Result process(const nav_msgs::Odometry& ego,
                 const interfaces::CompetitionVehicleStatus& competition,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) {
    const double now = objects.header.stamp.toSec();
    const double ego_time = ego.header.stamp.toSec();
    if ((!ego_history_.empty() && ego_time < ego_history_.back().time) || now < last_stamp_) {
      tracks_.clear();
      ego_history_.clear();
      last_stamp_ = -std::numeric_limits<double>::infinity();
    }
    if (ego_history_.empty() || ego_time > ego_history_.back().time) {
      const auto& q = ego.pose.pose.orientation;
      const auto& p = ego.pose.pose.position;
      const auto& v = ego.twist.twist.linear;
      const auto& w = ego.twist.twist.angular;
      ego_history_.push_back({ego_time, {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z},
                              {v.x, v.y, v.z}, {w.x, w.y, w.z}});
      while (ego_history_.size() > 2 && ego_time - ego_history_[1].time > 30.2)
        ego_history_.pop_front();
    }
    if (now == last_stamp_ && objects == last_objects_) {
      publishPredictions(last_predictions_);
      return std::make_tuple(ego, competition, objects, tf, imu, route, last_predictions_);
    }
    if (now != last_stamp_) {
      observation_frame_ = observationFrame(now);
      last_stamp_ = now;
    }
    for (const auto& object : objects.objects) {
      auto& history = tracks_[object.id];
      const auto& p = object.position;
      const Eigen::Vector3d point = observation_frame_ * Eigen::Vector3d(p.x, p.y, p.z);
      if (history.empty() || history.back().time != now) history.push_back({now, point});
      else history.back().point = point;
      while (!history.empty() && history.front().time < now - 30.0 - 1e-9)
        history.pop_front();
    }
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (now - it->second.back().time > 30.0) it = tracks_.erase(it);
      else ++it;
    }
    PredictedObjects predictions;
    predictions.fill(objects);
    for (std::size_t i = 0; i < objects.objects.size(); ++i) {
      const auto& input = objects.objects[i];
      const auto& history = tracks_[input.id];
      const auto matching = by_label_.find(input.label);
      double maximum_window = 1.0;
      if (matching != by_label_.end())
        maximum_window = parameters_.curves[matching->second.front()].history_sec;
      const double span = continuousSpan(history);
      double window = 1.0;
      for (double candidate : {2.0, 3.0, 5.0, 10.0, 20.0, 30.0})
        if (candidate <= maximum_window && candidate <= span + 1e-9) window = candidate;
      Fit selected;
      // A shorter recent path can describe a vehicle just entering the recorded ring.
      for (double candidate : {30.0, 20.0, 10.0, 5.0, 3.0, 2.0, 1.0}) {
        if (candidate > window) continue;
        const auto samples = observations(history, now, candidate);
        Fit free_fit, nominal_fit;
        if (matching == by_label_.end()) break;
        for (const auto index : matching->second) {
          const auto& curve = parameters_.curves[index];
          auto fitted = fit(curve, trees_[index], samples, false);
          if (fitted.score < free_fit.score) free_fit = fitted;
          if (candidate == 1.0 && curve.nominal_rate_mps > 0.0) {
            fitted = fit(curve, trees_[index], samples, true);
            if (fitted.score < nominal_fit.score) nominal_fit = fitted;
          }
        }
        const double slack_squared = parameters_.nominal_fit_slack_m * parameters_.nominal_fit_slack_m;
        const Fit chosen = nominal_fit.curve && nominal_fit.score <= free_fit.score + slack_squared
            ? nominal_fit : free_fit;
        if (chosen.curve && chosen.score <= parameters_.path_rms_limit_m * parameters_.path_rms_limit_m) {
          selected = chosen;
          break;
        }
      }
      const auto recent = observations(history, now, 1.0);
      const Motion motion = estimateMotion(recent);
      const auto& observed = history.back().point;
      const Eigen::Quaterniond frame_rotation(observation_frame_.rotation());
      const auto& source_q = input.bbox_pose.orientation;
      const Eigen::Quaterniond orientation(source_q.w, source_q.x, source_q.y, source_q.z);
      for (std::size_t k = 0; k < predictions.size(); ++k) {
        const double dt = 0.5 * (k + 1);
        Eigen::Vector3d point, velocity, acceleration;
        double yaw_delta;
        if (selected.curve) {
          const auto current = evaluate(*selected.curve, selected.phase);
          const auto future = evaluate(*selected.curve, selected.phase + selected.rate * dt);
          point = future.first;
          point.z() = observed.z() + motion.velocity.z() * dt;
          velocity = future.second * selected.rate;
          velocity.z() = motion.velocity.z();
          acceleration.setZero();
          yaw_delta = std::atan2(future.second.y(), future.second.x()) -
                      std::atan2(current.second.y(), current.second.x());
        } else {
          yaw_delta = motion.turn_rate * dt;
          const double sinc = std::abs(yaw_delta) < 1e-6 ? 1.0 - yaw_delta*yaw_delta/6.0 :
                              std::sin(yaw_delta) / yaw_delta;
          const double cosc = std::abs(yaw_delta) < 1e-6 ? yaw_delta/2.0 :
                              2.0*std::pow(std::sin(yaw_delta/2.0), 2) / yaw_delta;
          point = observed + dt * Eigen::Vector3d(
              sinc*motion.velocity.x() - cosc*motion.velocity.y(),
              cosc*motion.velocity.x() + sinc*motion.velocity.y(), motion.velocity.z());
          velocity = Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()) * motion.velocity;
          acceleration = Eigen::Vector3d(-motion.turn_rate*velocity.y(),
                                         motion.turn_rate*velocity.x(), 0.0);
        }
        auto& output = predictions[k].objects[i];
        const Eigen::Vector3d local_point = observation_frame_.inverse() * point;
        const Eigen::Vector3d local_velocity = frame_rotation.conjugate() * velocity;
        const Eigen::Vector3d local_acceleration = frame_rotation.conjugate() * acceleration;
        output.position.x = local_point.x(); output.position.y = local_point.y(); output.position.z = local_point.z();
        output.bbox_pose.position = output.position;
        const Eigen::Quaterniond future_q = (frame_rotation.conjugate() *
            Eigen::Quaterniond(Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ())) *
            frame_rotation * orientation).normalized();
        output.bbox_pose.orientation.x = future_q.x(); output.bbox_pose.orientation.y = future_q.y();
        output.bbox_pose.orientation.z = future_q.z(); output.bbox_pose.orientation.w = future_q.w();
        output.velocity.x = local_velocity.x(); output.velocity.y = local_velocity.y(); output.velocity.z = local_velocity.z();
        output.velocity_available = recent.size() > 1;
        output.acceleration.x = local_acceleration.x(); output.acceleration.y = local_acceleration.y();
        output.acceleration.z = local_acceleration.z();
        output.acceleration_available = !selected.curve && recent.size() > 2;
      }
    }
    last_objects_ = objects;
    last_predictions_ = predictions;
    publishPredictions(predictions);
    return std::make_tuple(ego, competition, objects, tf, imu, route, predictions);
  }

 private:
  using Curve = NpcHistoryPredictionParameters::Curve;
  struct Observation { double time; Eigen::Vector3d point; };
  struct EgoPose {
    double time;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d velocity, angular_velocity;
  };
  struct Fit {
    const Curve* curve = nullptr;
    double phase = 0.0, rate = 0.0;
    double score = std::numeric_limits<double>::infinity();
  };
  struct Motion { Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); double turn_rate = 0.0; };
  struct KdNode { std::size_t point; int left, right, axis; double x, y; };

  void publishPredictions(const PredictedObjects& predictions) {
    for (std::size_t i = 0; i < predictions.size(); ++i) prediction_visualizers_[i].publish(predictions[i]);
  }

  static int buildTree(const Curve& curve, std::vector<std::size_t>& order, std::size_t begin,
                       std::size_t end, int axis, std::vector<KdNode>& tree) {
    if (begin == end) return -1;
    const auto middle = begin+(end-begin)/2;
    std::nth_element(order.begin()+begin, order.begin()+middle, order.begin()+end,
        [&](std::size_t a, std::size_t b) { return curve.points[a][axis] < curve.points[b][axis]; });
    const int node = tree.size();
    const auto point = order[middle];
    tree.push_back({point, -1, -1, axis, curve.points[point].x(), curve.points[point].y()});
    const int left = buildTree(curve, order, begin, middle, 1-axis, tree);
    const int right = buildTree(curve, order, middle+1, end, 1-axis, tree);
    tree[node].left = left; tree[node].right = right;
    return node;
  }
  static void nearestPoint(const std::vector<KdNode>& tree, int node, double x, double y,
                           std::size_t& nearest, double& distance) {
    if (node < 0) return;
    const auto& point = tree[node];
    const double dx = point.x-x, dy = point.y-y, squared = dx*dx+dy*dy;
    if (squared < distance || (squared == distance && point.point < nearest)) {
      distance = squared; nearest = point.point;
    }
    const double split = point.axis == 0 ? dx : dy;
    nearestPoint(tree, split > 0.0 ? point.left : point.right, x, y, nearest, distance);
    if (split*split <= distance)
      nearestPoint(tree, split > 0.0 ? point.right : point.left, x, y, nearest, distance);
  }
  Eigen::Isometry3d observationFrame(double time) const {
    const auto after = std::lower_bound(ego_history_.begin(), ego_history_.end(), time,
        [](const EgoPose& pose, double t) { return pose.time < t; });
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    if (after != ego_history_.begin() && after != ego_history_.end() &&
        after->time-(after-1)->time <= 0.2+1e-9) {
      const auto& before = *(after - 1);
      const double u = (time - before.time) / (after->time - before.time);
      result.linear() = before.rotation.slerp(u, after->rotation).toRotationMatrix();
      result.translation() = (1.0-u)*before.position + u*after->position;
    } else {
      const auto& pose = after == ego_history_.end() ? ego_history_.back() : *after;
      const double dt = time - pose.time;
      const double omega = pose.angular_velocity.norm();
      const double angle = omega*dt;
      const double a = std::abs(angle) < 1e-5 ? dt*dt*(0.5-angle*angle/24.0) :
                       (1.0-std::cos(angle))/(omega*omega);
      const double b = std::abs(angle) < 1e-5 ? dt*dt*dt*(1.0/6.0-angle*angle/120.0) :
                       (angle-std::sin(angle))/(omega*omega*omega);
      const Eigen::Vector3d cross = pose.angular_velocity.cross(pose.velocity);
      result.translation() = pose.position + pose.rotation *
          (dt*pose.velocity + a*cross + b*pose.angular_velocity.cross(cross));
      const Eigen::Quaterniond delta = omega == 0.0 ? Eigen::Quaterniond::Identity() :
          Eigen::Quaterniond(Eigen::AngleAxisd(angle, pose.angular_velocity / omega));
      // shortcut: constant body twist across pose gaps or outside the buffer; dense poses use interpolation.
      result.linear() = (pose.rotation * delta).toRotationMatrix();
    }
    return result;
  }
  static double continuousSpan(const std::deque<Observation>& history) {
    std::size_t begin = history.size()-1;
    while (begin > 0 && history[begin].time - history[begin-1].time <= 0.15 + 1e-9) --begin;
    return history.back().time - history[begin].time;
  }
  static std::vector<Observation> observations(const std::deque<Observation>& history,
                                              double now, double window) {
    std::size_t begin = history.size()-1;
    while (begin > 0 && history[begin-1].time >= now-window-1e-9 &&
           history[begin].time-history[begin-1].time <= 0.15+1e-9) --begin;
    std::vector<Observation> result;
    std::size_t index = begin;
    for (double desired = now-window; desired < now-1e-9; desired += 0.1) {
      while (index+1 < history.size() && history[index+1].time <= desired+1e-9) ++index;
      if (result.empty() || result.back().time != history[index].time) result.push_back(history[index]);
    }
    if (result.empty() || result.back().time != now) result.push_back(history.back());
    return result;
  }
  static std::pair<std::size_t, double> segment(const Curve& curve, double phase) {
    double wrapped = std::fmod(phase, curve.length_m);
    if (wrapped < 0.0) wrapped += curve.length_m;
    const double coordinate = wrapped * curve.points.size() / curve.length_m;
    const auto index = std::min(static_cast<std::size_t>(coordinate), curve.points.size()-1);
    return {index, coordinate-index};
  }
  static std::pair<Eigen::Vector3d, Eigen::Vector3d> evaluate(const Curve& curve, double phase) {
    const auto location = segment(curve, phase);
    const auto index = location.first;
    const auto& p = curve.points[index];
    const Eigen::Vector3d delta = curve.points[(index+1)%curve.points.size()] - p;
    const double u = location.second;
    // Centered tangents avoid a heading jump at each polyline knot.
    const Eigen::Vector3d tangent = ((1.0-u)*(curve.points[(index+1)%curve.points.size()]-
        curve.points[(index+curve.points.size()-1)%curve.points.size()]) +
        u*(curve.points[(index+2)%curve.points.size()]-p)) * curve.points.size() / (2.0*curve.length_m);
    return {p + u*delta, tangent};
  }
  static Fit fit(const Curve& curve, const std::vector<KdNode>& tree,
                 const std::vector<Observation>& samples, bool nominal) {
    std::vector<double> phases;
    phases.reserve(samples.size());
    for (const auto& sample : samples) {
      std::size_t nearest = 0;
      double distance = std::numeric_limits<double>::infinity();
      const double x = sample.point.x(), y = sample.point.y();
      nearestPoint(tree, 0, x, y, nearest, distance);
      double phase = 0.0;
      distance = std::numeric_limits<double>::infinity();
      for (std::size_t start : {nearest, (nearest+curve.points.size()-1)%curve.points.size()}) {
        const auto& p = curve.points[start];
        const auto& next = curve.points[(start+1)%curve.points.size()];
        const double dx = next.x()-p.x(), dy = next.y()-p.y();
        const double u = std::max(0.0, std::min(1.0, ((x-p.x())*dx+(y-p.y())*dy)/(dx*dx+dy*dy)));
        const double ex = x-p.x()-u*dx, ey = y-p.y()-u*dy;
        const double error = ex*ex+ey*ey;
        if (error < distance) { distance = error; phase = (start+u)*curve.length_m/curve.points.size(); }
      }
      if (!phases.empty()) phase = phases.back()+std::remainder(phase-phases.back(), curve.length_m);
      phases.push_back(phase);
    }
    double mean_t = 0.0, mean_s = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      mean_t += samples[i].time-samples.back().time; mean_s += phases[i];
    }
    mean_t /= samples.size(); mean_s /= samples.size();
    double covariance = 0.0, variance = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const double t = samples[i].time-samples.back().time-mean_t;
      covariance += t*(phases[i]-mean_s); variance += t*t;
    }
    Fit fitted;
    fitted.curve = &curve;
    fitted.rate = nominal ? curve.nominal_rate_mps : (variance == 0.0 ? 0.0 : covariance/variance);
    fitted.phase = mean_s-fitted.rate*mean_t;
    fitted.score = 0.0;
    for (const auto& sample : samples) {
      const auto location = segment(curve, fitted.phase+fitted.rate*(sample.time-samples.back().time));
      const auto& p = curve.points[location.first];
      const auto& next = curve.points[(location.first+1)%curve.points.size()];
      const double dx = p.x()+location.second*(next.x()-p.x())-sample.point.x();
      const double dy = p.y()+location.second*(next.y()-p.y())-sample.point.y();
      fitted.score += dx*dx+dy*dy;
    }
    fitted.score /= samples.size();
    return fitted;
  }
  static Motion estimateMotion(const std::vector<Observation>& samples) {
    const int columns = std::min<int>(3, samples.size());
    Eigen::MatrixXd design(samples.size(), columns), positions(samples.size(), 3);
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const double t = samples[i].time-samples.back().time;
      design(i, 0) = 1.0;
      if (columns > 1) design(i, 1) = t;
      if (columns > 2) design(i, 2) = 0.5*t*t;
      positions.row(i) = samples[i].point.transpose();
    }
    const Eigen::MatrixXd coefficients = design.colPivHouseholderQr().solve(positions);
    Motion motion;
    if (columns > 1) motion.velocity = coefficients.row(1).transpose();
    if (columns > 2) {
      const double speed_squared = motion.velocity.head<2>().squaredNorm();
      if (speed_squared > 1e-12)
        motion.turn_rate = (motion.velocity.x()*coefficients(2,1)-motion.velocity.y()*coefficients(2,0)) / speed_squared;
    }
    // Vertical coordinates remain in the observed datum; use a linear vertical trend.
    if (samples.size() > 1) {
      double mean_t = 0.0, mean_z = 0.0, covariance = 0.0, variance = 0.0;
      for (const auto& sample : samples) { mean_t += sample.time; mean_z += sample.point.z(); }
      mean_t /= samples.size(); mean_z /= samples.size();
      for (const auto& sample : samples) {
        covariance += (sample.time-mean_t)*(sample.point.z()-mean_z);
        variance += (sample.time-mean_t)*(sample.time-mean_t);
      }
      motion.velocity.z() = variance == 0.0 ? 0.0 : covariance/variance;
    }
    return motion;
  }

  NpcHistoryPredictionParameters parameters_;
  std::vector<std::vector<KdNode>> trees_;
  std::unordered_map<std::string, std::vector<std::size_t>> by_label_;
  std::unordered_map<std::uint32_t, std::deque<Observation>> tracks_;
  interfaces::Objects last_objects_;
  PredictedObjects last_predictions_;
  std::deque<EgoPose> ego_history_;
  Eigen::Isometry3d observation_frame_ = Eigen::Isometry3d::Identity();
  double last_stamp_ = -std::numeric_limits<double>::infinity();
  using ObjectsVisualizer = node_pipeline::Visualizer<interfaces::Objects>;
  std::array<ObjectsVisualizer, 10> prediction_visualizers_{{
      ObjectsVisualizer("predicted_objects_0_5s"), ObjectsVisualizer("predicted_objects_1_0s"),
      ObjectsVisualizer("predicted_objects_1_5s"), ObjectsVisualizer("predicted_objects_2_0s"),
      ObjectsVisualizer("predicted_objects_2_5s"), ObjectsVisualizer("predicted_objects_3_0s"),
      ObjectsVisualizer("predicted_objects_3_5s"), ObjectsVisualizer("predicted_objects_4_0s"),
      ObjectsVisualizer("predicted_objects_4_5s"), ObjectsVisualizer("predicted_objects_5_0s")}};
};
}  // namespace path_planner
