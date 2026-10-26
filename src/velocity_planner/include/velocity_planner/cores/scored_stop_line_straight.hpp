#pragma once
#include <velocity_planner/banks/stop_line_state.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Path.h>
#include <interfaces/TrafficLight.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace velocity_planner {
struct ScoredStopLineStraightParameters {
  struct Corridor {
    std::int64_t stop_line_id;
    std::vector<std::array<double, 2>> left;
    std::vector<std::array<double, 2>> right;
    double projection_m = 0;
  };
  std::vector<Corridor> corridors;
  std::vector<std::int64_t> scored_stop_line_ids;
  std::vector<std::int64_t> left_signal_stop_line_ids;
};

class ScoredStopLineStraight {
 public:
  explicit ScoredStopLineStraight(ScoredStopLineStraightParameters value, StopLineState& state)
      : parameters_(std::move(value)), state_(state) {}
  std::pair<interfaces::Path, tf2_msgs::TFMessage> process(
      const interfaces::Path& path, const interfaces::TrafficLight& light,
      const interfaces::CompetitionVehicleStatus& competition, const tf2_msgs::TFMessage& tf) {
    std::vector<double> station(path.points.size(), 0);
    for (std::size_t i = 1; i < path.points.size(); ++i) {
      const auto& from = path.points[i - 1].pose.pose.position;
      const auto& to = path.points[i].pose.pose.position;
      station[i] = station[i - 1] + std::hypot(to.x - from.x, to.y - from.y);
    }
    const auto contact = firstContact(path, station);
    const bool scored_latch = state_.stopping && isScored(state_.stop_line_id);
    if (!scored_latch && !isScored(contact.id)) {
      if (!state_.stopping && isScored(state_.stop_line_id)) {
        state_.stop_line_id = 0;
        state_.green_frames = 0;
      }
      return legacy(path, light, competition, tf);
    }
    const auto target_id = scored_latch ? state_.stop_line_id : contact.id;
    const bool straight = std::find(parameters_.left_signal_stop_line_ids.begin(),
        parameters_.left_signal_stop_line_ids.end(), target_id) == parameters_.left_signal_stop_line_ids.end();
    if (state_.stop_line_id != target_id || state_.accepts_circular_green != straight) {
      state_.green_frames = 0;
      state_.stop_line_id = target_id;
      state_.accepts_circular_green = straight;
    }
    state_.released_stop_line_id = 0;
    const auto signal = scoredSignal(light.state, straight);
    if (signal != Signal::Go) state_.green_frames = 0;
    const auto stamp_ns = light.header.stamp.toNSec();
    if (stamp_ns > state_.last_light_stamp_ns) {
      state_.last_light_stamp_ns = stamp_ns;
      if (signal == Signal::Go && state_.green_frames < state_.parameters.n_frames)
        ++state_.green_frames;
    }
    const bool confirmed = signal == Signal::Go &&
        state_.green_frames >= state_.parameters.n_frames;
    interfaces::Path output = path;
    if (confirmed) {
      state_.stopping = false;
      return {std::move(output), tf};
    }
    const auto& parameters = state_.parameters;
    // The rear-axle target leaves at least 0.5 m before the front axle reaches the line.
    const double target_station = contact.station_m - std::max(parameters.x_m, 3.5);
    const double speed = std::abs(static_cast<double>(competition.signed_velocity_mps));
    if (!state_.stopping && (signal != Signal::Yellow ||
        speed * speed / (2 * parameters.z_mps2) <= target_station)) {
      state_.stopping = true;
      state_.stopping_started_sec = path.header.stamp.toSec();
    }
    if (!state_.stopping) return {std::move(output), tf};
    if (contact.id != target_id) {
      for (auto& point : output.points) {
        point.velocity = 0;
        point.acceleration = 0;
      }
      return {std::move(output), tf};
    }
    for (std::size_t i = 0; i < output.points.size(); ++i) {
      auto& point = output.points[i];
      const double remaining = std::max(target_station - station[i], 0.0);
      const double limit = std::sqrt(2 * parameters.z_mps2 * remaining);
      const double velocity = std::min(static_cast<double>(point.velocity), limit);
      if (remaining == 0) {
        point.velocity = 0;
        point.acceleration = 0;
      } else if (velocity < point.velocity) {
        point.velocity = velocity;
        point.acceleration = -parameters.z_mps2;
      }
    }
    return {std::move(output), tf};
  }

 private:
  enum class Signal { Ignore, Stop, Yellow, Go };
  bool isScored(std::int64_t id) const {
    return std::find(parameters_.scored_stop_line_ids.begin(),
        parameters_.scored_stop_line_ids.end(), id) != parameters_.scored_stop_line_ids.end();
  }

  static Signal scoredSignal(std::uint8_t state, bool straight) {
    static const std::array<Signal, 17> straight_mapping{{
      Signal::Stop, Signal::Stop, Signal::Stop, Signal::Yellow, Signal::Go,
      Signal::Stop, Signal::Yellow, Signal::Go, Signal::Stop, Signal::Go, Signal::Yellow,
      Signal::Go, Signal::Go, Signal::Yellow, Signal::Go, Signal::Go, Signal::Go
    }};
    static const std::array<Signal, 17> left_mapping{{
      Signal::Stop, Signal::Stop, Signal::Stop, Signal::Yellow, Signal::Stop,
      Signal::Go, Signal::Yellow, Signal::Stop, Signal::Go, Signal::Yellow, Signal::Go,
      Signal::Go, Signal::Yellow, Signal::Go, Signal::Go, Signal::Go, Signal::Go
    }};
    return straight ? straight_mapping[state] : left_mapping[state];
  }

  std::pair<interfaces::Path, tf2_msgs::TFMessage> legacy(
      const interfaces::Path& path, const interfaces::TrafficLight& light,
      const interfaces::CompetitionVehicleStatus& competition, const tf2_msgs::TFMessage& tf) {
    interfaces::Path output = path;
    const auto& parameters = state_.parameters;
    if (state_.stopping &&
        path.header.stamp.toSec() - state_.stopping_started_sec >= parameters.stop_timeout_sec) {
      state_.released_stop_line_id = state_.stop_line_id;
      state_.stopping = false;
      state_.stop_line_id = 0;
      state_.green_frames = 0;
    }
    const auto signal = signalFor(light.state);
    const bool accepts_circular_green = signalFor(interfaces::TrafficLight::GREEN) == Signal::Go;
    if (state_.accepts_circular_green != accepts_circular_green) {
      state_.accepts_circular_green = accepts_circular_green;
      state_.green_frames = 0;
    }
    const auto stamp_ns = light.header.stamp.toNSec();
    if (stamp_ns > state_.last_light_stamp_ns) {
      state_.last_light_stamp_ns = stamp_ns;
      if (state_.stopping) {
        state_.green_frames = signal == Signal::Go ? state_.green_frames + 1 : 0;
        if (state_.green_frames >= parameters.n_frames) {
          state_.stopping = false;
          state_.stop_line_id = 0;
          state_.green_frames = 0;
        }
      }
    }

    std::vector<double> station(path.points.size(), 0);
    for (std::size_t i = 1; i < path.points.size(); ++i) {
      const auto& from = path.points[i - 1].pose.pose.position;
      const auto& to = path.points[i].pose.pose.position;
      station[i] = station[i - 1] + std::hypot(to.x - from.x, to.y - from.y);
    }
    const auto contact = firstContact(path, station);
    if (contact.id != state_.released_stop_line_id) state_.released_stop_line_id = 0;
    const double stop_station_m = contact.station_m - parameters.x_m;
    const double speed_mps = std::abs(static_cast<double>(competition.signed_velocity_mps));
    if (!state_.stopping && contact.id != 0 && contact.id != state_.released_stop_line_id &&
        (signal == Signal::Stop ||
         (signal == Signal::Yellow && speed_mps * speed_mps / (2 * parameters.z_mps2) <= stop_station_m))) {
      state_.stopping = true;
      state_.stopping_started_sec = path.header.stamp.toSec();
      state_.stop_line_id = contact.id;
      state_.green_frames = 0;
    }
    if (!state_.stopping || contact.id == 0) return {std::move(output), tf};

    if (stop_station_m < parameters.y_m) {
      for (auto& point : output.points) {
        point.velocity = 0;
        point.acceleration = 0;
      }
      return {std::move(output), tf};
    }

    const double start_station_m = stop_station_m - parameters.y_m;
    const auto upper = std::upper_bound(station.begin(), station.end(), start_station_m);
    const std::size_t right = upper - station.begin();
    const std::size_t left = right - 1;
    const double ratio = (start_station_m - station[left]) / (station[right] - station[left]);
    const double start_velocity_mps = path.points[left].velocity +
        ratio * (path.points[right].velocity - path.points[left].velocity);
    for (std::size_t i = 0; i < output.points.size(); ++i) {
      auto& point = output.points[i];
      if (station[i] >= stop_station_m) {
        point.velocity = 0;
        point.acceleration = 0;
      } else if (station[i] >= start_station_m) {
        point.velocity = start_velocity_mps * (stop_station_m - station[i]) / parameters.y_m;
        point.acceleration = -start_velocity_mps * point.velocity / parameters.y_m;
      }
    }
    return {std::move(output), tf};
  }

  struct Contact {
    std::int64_t id = 0;
    double station_m = std::numeric_limits<double>::infinity();
  };

  static Signal signalFor(std::uint8_t state) {
    static const std::array<Signal, 17> mapping{{
      Signal::Ignore, Signal::Ignore, Signal::Stop, Signal::Yellow, Signal::Go,
      Signal::Stop, Signal::Yellow, Signal::Go, Signal::Stop, Signal::Go, Signal::Yellow,
      Signal::Go, Signal::Go, Signal::Yellow, Signal::Go, Signal::Go, Signal::Go
    }};
    return mapping[state];
  }

  static double intersectionRatio(const std::array<double, 2>& a, const std::array<double, 2>& b,
                                  const std::array<double, 2>& c, const std::array<double, 2>& d) {
    const double rx = b[0] - a[0], ry = b[1] - a[1];
    const double sx = d[0] - c[0], sy = d[1] - c[1];
    const double qx = c[0] - a[0], qy = c[1] - a[1];
    const double length_squared = rx * rx + ry * ry;
    const double none = std::numeric_limits<double>::infinity();
    if (length_squared == 0) return none;
    const double denominator = rx * sy - ry * sx;
    if (std::abs(denominator) > 1e-9) {
      const double t = (qx * sy - qy * sx) / denominator;
      const double u = (qx * ry - qy * rx) / denominator;
      return t >= -1e-9 && t <= 1 + 1e-9 && u >= -1e-9 && u <= 1 + 1e-9
          ? std::max(0.0, std::min(1.0, t)) : none;
    }
    if (std::abs(qx * ry - qy * rx) > 1e-9) return none;
    const double first = (qx * rx + qy * ry) / length_squared;
    const double last = first + (sx * rx + sy * ry) / length_squared;
    const double lower = std::max(0.0, std::min(first, last));
    const double upper = std::min(1.0, std::max(first, last));
    return lower <= upper + 1e-9 ? std::min(1.0, lower) : none;
  }

  static bool insideCorridor(const std::array<double, 2>& point,
                             const ScoredStopLineStraightParameters::Corridor& corridor) {
    const auto distance2 = [](const std::array<double, 2>& a, const std::array<double, 2>& b) {
      const double dx = a[0] - b[0], dy = a[1] - b[1];
      return dx * dx + dy * dy;
    };
    auto left = corridor.left;
    auto right = corridor.right;
    const auto project = [&corridor](std::vector<std::array<double, 2>>& boundary) {
      const auto first = boundary.front();
      const auto last = boundary.back();
      const double start_dx = first[0] - boundary[1][0];
      const double start_dy = first[1] - boundary[1][1];
      const double end_dx = last[0] - boundary[boundary.size() - 2][0];
      const double end_dy = last[1] - boundary[boundary.size() - 2][1];
      const double start_length = std::hypot(start_dx, start_dy);
      const double end_length = std::hypot(end_dx, end_dy);
      if (start_length > 0) {
        boundary.front()[0] += corridor.projection_m * start_dx / start_length;
        boundary.front()[1] += corridor.projection_m * start_dy / start_length;
      }
      if (end_length > 0) {
        boundary.back()[0] += corridor.projection_m * end_dx / end_length;
        boundary.back()[1] += corridor.projection_m * end_dy / end_length;
      }
    };
    project(left);
    project(right);
    const bool same_direction = distance2(left.front(), right.front()) +
        distance2(left.back(), right.back()) <= distance2(left.front(), right.back()) +
        distance2(left.back(), right.front());
    std::vector<std::array<double, 2>> polygon(left.begin(), left.end());
    if (same_direction) polygon.insert(polygon.end(), right.rbegin(), right.rend());
    else polygon.insert(polygon.end(), right.begin(), right.end());
    bool inside = false;
    constexpr double epsilon = 1e-8;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      const auto& a = polygon[j];
      const auto& b = polygon[i];
      const double dx = b[0] - a[0], dy = b[1] - a[1];
      const double px = point[0] - a[0], py = point[1] - a[1];
      const double length2 = dx * dx + dy * dy;
      const double cross = dx * py - dy * px;
      const double dot = dx * px + dy * py;
      if ((length2 == 0 && distance2(point, a) <= epsilon * epsilon) ||
          (length2 > 0 && cross * cross <= epsilon * epsilon * length2 &&
           dot >= -epsilon * std::sqrt(length2) && dot <= length2 + epsilon * std::sqrt(length2)))
        return true;
      if ((a[1] > point[1]) != (b[1] > point[1]) &&
          point[0] < a[0] + (point[1] - a[1]) * dx / dy)
        inside = !inside;
    }
    return inside;
  }

  double corridorIntersectionRatio(const std::array<double, 2>& a,
                                   const std::array<double, 2>& b,
                                   const StopLineStateParameters::StopLine& line) const {
    const double none = std::numeric_limits<double>::infinity();
    if (!isScored(line.id)) return none;
    const auto& c = line.points.front();
    const auto& d = line.points.back();
    const double rx = b[0] - a[0], ry = b[1] - a[1];
    const double sx = d[0] - c[0], sy = d[1] - c[1];
    const double denominator = rx * sy - ry * sx;
    if (std::abs(denominator) <= 1e-9) return none;
    const double ratio = ((c[0] - a[0]) * sy - (c[1] - a[1]) * sx) / denominator;
    if (ratio < 0 || ratio > 1) return none;
    const std::array<double, 2> point{{a[0] + ratio * rx, a[1] + ratio * ry}};
    for (const auto& corridor : parameters_.corridors)
      if (corridor.stop_line_id == line.id && insideCorridor(point, corridor)) return ratio;
    return none;
  }

  Contact firstContact(const interfaces::Path& path, const std::vector<double>& station) const {
    Contact contact;
    for (std::size_t i = 1; i < path.points.size(); ++i) {
      const auto& from = path.points[i - 1].pose.pose.position;
      const auto& to = path.points[i].pose.pose.position;
      for (const auto& line : state_.parameters.stop_lines) {
        if (state_.stopping && line.id != state_.stop_line_id) continue;
        for (std::size_t j = 1; j < line.points.size(); ++j) {
          const double ratio = intersectionRatio({{from.x, from.y}}, {{to.x, to.y}},
                                                 line.points[j - 1], line.points[j]);
          const double distance_m = station[i - 1] + ratio * (station[i] - station[i - 1]);
          if (std::isfinite(distance_m) && (distance_m < contact.station_m ||
              (distance_m == contact.station_m && line.id < contact.id)))
            contact = {line.id, distance_m};
        }
        const double corridor_ratio = corridorIntersectionRatio(
            {{from.x, from.y}}, {{to.x, to.y}}, line);
        const double corridor_station = station[i - 1] + corridor_ratio *
            (station[i] - station[i - 1]);
        if (std::isfinite(corridor_station) && (corridor_station < contact.station_m ||
            (corridor_station == contact.station_m && line.id < contact.id)))
          contact = {line.id, corridor_station};
      }
      if (contact.id != 0) break;
    }
    return contact;
  }

  ScoredStopLineStraightParameters parameters_;
  StopLineState& state_;
};
}  // namespace velocity_planner
