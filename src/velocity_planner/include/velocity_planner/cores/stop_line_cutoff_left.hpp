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
class StopLineCutoffLeft {
 public:
  explicit StopLineCutoffLeft(StopLineState& state) : state_(state) {}
  std::pair<interfaces::Path, tf2_msgs::TFMessage> process(
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

 private:
  enum class Signal { Ignore, Stop, Yellow, Go };
  struct Contact {
    std::int64_t id = 0;
    double station_m = std::numeric_limits<double>::infinity();
  };

  static Signal signalFor(std::uint8_t state) {
    static const std::array<Signal, 17> mapping{{
      Signal::Ignore, Signal::Ignore, Signal::Stop, Signal::Yellow, Signal::Stop,
      Signal::Go, Signal::Yellow, Signal::Stop, Signal::Go, Signal::Yellow, Signal::Go,
      Signal::Go, Signal::Yellow, Signal::Go, Signal::Go, Signal::Go, Signal::Go
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
      }
      if (contact.id != 0) break;
    }
    return contact;
  }

  StopLineState& state_;
};
}  // namespace velocity_planner
