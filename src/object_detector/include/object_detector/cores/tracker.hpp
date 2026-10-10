#pragma once
#include <interfaces/Objects.h>
#include <cstdint>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include <algorithm>
#include <deque>
#include <limits>

namespace object_detector {
struct TrackerParameters {
  double association_gate_m = 2.5;
  double shape_weight = 0.2;
  double max_age_sec = 0.6;
  int history_size = 31;
  int min_acceleration_observations = 5;
};
class Tracker {
 public:
  explicit Tracker(TrackerParameters parameters = {}) : parameters_(parameters) {}
  interfaces::Objects process(const interfaces::Objects& objects,
                              const ros::Time& stamp) {
    const double time = stamp.toSec();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
      return time - track.history.back().time > parameters_.max_age_sec;
    }), tracks_.end());

    std::vector<Eigen::Vector3d> positions;
    for (const auto& object : objects.objects)
      positions.push_back(vector(object.position));
    const auto matches = associate(objects, positions, time);
    interfaces::Objects output;
    output.objects = objects.objects;
    for (std::size_t i = 0; i < output.objects.size(); ++i) {
      const std::size_t index = matches[i];
      if (index == noTrack()) {
        Track track;
        track.id = next_id_++;
        tracks_.push_back(std::move(track));
      }
      auto& track = tracks_[index == noTrack() ? tracks_.size() - 1 : index];
      track.label = output.objects[i].label;
      track.size = vector(output.objects[i].bbox_size);
      if (!track.history.empty() && track.history.back().time == time)
        track.history.back().position = positions[i];
      else track.history.push_back({time, positions[i]});
      while (track.history.size() > static_cast<std::size_t>(parameters_.history_size))
        track.history.pop_front();
      fitMotion(track);
      auto& object = output.objects[i];
      object.id = track.id;
      object.velocity = message(track.velocity);
      object.acceleration = message(track.acceleration);
      object.velocity_available = track.velocity_available;
      object.acceleration_available = track.acceleration_available;
    }
    return output;
  }
 private:
  struct Observation { double time; Eigen::Vector3d position; };
  struct Track {
    std::uint32_t id = 0;
    std::string label;
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    std::deque<Observation> history;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    bool velocity_available = false, acceleration_available = false;
  };
  template<class Vector> static Eigen::Vector3d vector(const Vector& value) {
    return {value.x, value.y, value.z};
  }
  static geometry_msgs::Vector3 message(const Eigen::Vector3d& value) {
    geometry_msgs::Vector3 result;
    result.x = value.x(); result.y = value.y(); result.z = value.z();
    return result;
  }
  void fitMotion(Track& track) const {
    const auto count = track.history.size();
    track.velocity_available = count >= 2;
    track.acceleration_available = count >= static_cast<std::size_t>(parameters_.min_acceleration_observations);
    if (!track.velocity_available) return;
    const int degree = track.acceleration_available ? 3 : 1;
    const double now = track.history.back().time;
    const double span = now - track.history.front().time;
    Eigen::MatrixXd design(count, degree + 1), observations(count, 3);
    for (std::size_t i = 0; i < count; ++i) {
      const double t = (track.history[i].time - now) / span;
      design(i, 0) = 1;
      for (int j = 1; j <= degree; ++j) design(i, j) = design(i, j - 1) * t;
      observations.row(i) = (track.history[i].position - track.history.back().position).transpose();
    }
    const Eigen::MatrixXd coefficients = design.colPivHouseholderQr().solve(observations);
    track.velocity = coefficients.row(1).transpose() / span;
    track.acceleration = track.acceleration_available
      ? Eigen::Vector3d(2 * coefficients.row(2).transpose() / (span * span)) : Eigen::Vector3d::Zero();
  }
  std::vector<std::size_t> associate(const interfaces::Objects& objects,
                                    const std::vector<Eigen::Vector3d>& positions, double time) const {
    const std::size_t rows = positions.size(), columns = tracks_.size() + rows;
    std::vector<std::size_t> result(rows, noTrack());
    if (rows == 0) return result;
    const double unmatched = parameters_.association_gate_m * parameters_.association_gate_m;
    const double forbidden = 1e12;
    std::vector<std::vector<double>> costs(rows, std::vector<double>(columns, unmatched));
    for (std::size_t i = 0; i < rows; ++i) {
      for (std::size_t j = 0; j < tracks_.size(); ++j) {
        const auto& track = tracks_[j];
        const double dt = time - track.history.back().time;
        Eigen::Vector3d expected = track.history.back().position;
        if (track.velocity_available) expected += track.velocity * dt;
        if (track.acceleration_available) expected += 0.5 * track.acceleration * dt * dt;
        const double distance = (positions[i] - expected).squaredNorm();
        const auto& label = objects.objects[i].label;
        if (distance > unmatched || (label != "unknown" && track.label != "unknown" && label != track.label)) {
          costs[i][j] = forbidden;
          continue;
        }
        const Eigen::Vector3d size = vector(objects.objects[i].bbox_size);
        // Visibility can change either observed extent; compare them symmetrically.
        const Eigen::Vector3d delta = (size - track.size).cwiseQuotient(
          size.cwiseAbs() + track.size.cwiseAbs() + Eigen::Vector3d::Constant(0.1));
        costs[i][j] = distance + parameters_.shape_weight * delta.squaredNorm();
      }
    }
    // Rectangular Hungarian assignment with unmatched columns for new observations.
    std::vector<double> u(rows + 1), v(columns + 1);
    std::vector<std::size_t> owner(columns + 1), previous(columns + 1);
    for (std::size_t row = 1; row <= rows; ++row) {
      owner[0] = row;
      std::size_t column = 0;
      std::vector<double> minimum(columns + 1, forbidden);
      std::vector<bool> used(columns + 1, false);
      do {
        used[column] = true;
        const auto current = owner[column];
        double step = forbidden;
        std::size_t next = 0;
        for (std::size_t j = 1; j <= columns; ++j) if (!used[j]) {
          const double cost = costs[current - 1][j - 1] - u[current] - v[j];
          if (cost < minimum[j]) { minimum[j] = cost; previous[j] = column; }
          if (minimum[j] < step) { step = minimum[j]; next = j; }
        }
        for (std::size_t j = 0; j <= columns; ++j) {
          if (used[j]) { u[owner[j]] += step; v[j] -= step; }
          else minimum[j] -= step;
        }
        column = next;
      } while (owner[column] != 0);
      do {
        const auto next = previous[column];
        owner[column] = owner[next];
        column = next;
      } while (column != 0);
    }
    for (std::size_t j = 1; j <= tracks_.size(); ++j)
      if (owner[j] != 0 && costs[owner[j] - 1][j - 1] < unmatched)
        result[owner[j] - 1] = j - 1;
    return result;
  }
  static std::size_t noTrack() { return std::numeric_limits<std::size_t>::max(); }
  TrackerParameters parameters_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_ = 1;
};
}  // namespace object_detector
