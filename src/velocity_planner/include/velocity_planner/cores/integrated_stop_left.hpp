#pragma once
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
struct IntegratedStopLeftParameters {
  double extra_stop_offset_m=0;
  double blind_detection_distance_m = 0.0;  // 0 disables the blind-approach limit.
  double blind_deceleration_mps2 = 2.5;
};
class IntegratedStopLeft {
 public:
  std::pair<interfaces::Path, tf2_msgs::TFMessage> process(const interfaces::Path& path, const interfaces::TrafficLight& light,
      const interfaces::CompetitionVehicleStatus& competition, const tf2_msgs::TFMessage& tf) {
    auto result=calculate(path,light,competition,tf);
    const bool timed=std::any_of(path.points.begin(),path.points.end(),
        [](const interfaces::PathPoint& p){return !p.time_from_start.isZero();});
    bool changed=result.first.points.size()!=path.points.size();
    for(std::size_t i=0;i<path.points.size() && !changed;++i)
      changed=path.points[i].velocity!=result.first.points[i].velocity ||
              path.points[i].acceleration!=result.first.points[i].acceleration;
    if(timed && changed)reschedule(result.first,path.points.back().time_from_start.toSec());
    return result;
  }
  explicit IntegratedStopLeft(StopLineState& state) : state_(state) {}
  IntegratedStopLeft(IntegratedStopLeftParameters parameters,StopLineState& state)
      : parameters_(parameters),state_(state) {}
  std::pair<interfaces::Path, tf2_msgs::TFMessage> calculate(
      const interfaces::Path& path, const interfaces::TrafficLight& light,
      const interfaces::CompetitionVehicleStatus& competition, const tf2_msgs::TFMessage& tf) {
    interfaces::Path output = path;
    if (releaseAfterJump(path)) go_line_id_ = 0;
    releasePassedLine(path);
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
    interfaces::Path detection=path;
    if(!path.lateral_reference.empty()) {
      detection.points.clear();
      for(const auto& pose:path.lateral_reference) {
        interfaces::PathPoint point;point.pose.pose=pose;detection.points.push_back(point);
      }
    }
    std::vector<double> detection_station(detection.points.size(),0);
    for(std::size_t i=1;i<detection.points.size();++i) {
      const auto& a=detection.points[i-1].pose.pose.position;const auto& b=detection.points[i].pose.pose.position;
      detection_station[i]=detection_station[i-1]+std::hypot(b.x-a.x,b.y-a.y);
    }
    const auto contact = firstContact(detection, detection_station);
    if (contact.id != state_.released_stop_line_id) state_.released_stop_line_id = 0;
    const double stop_offset_m = parameters.x_m + parameters_.extra_stop_offset_m;
    const double stop_station_m = contact.station_m - stop_offset_m;
    const double speed_mps = std::abs(static_cast<double>(competition.signed_velocity_mps));
    if (!state_.stopping && contact.id != 0 && contact.id != state_.released_stop_line_id &&
        (signal == Signal::Stop ||
         (signal == Signal::Yellow && speed_mps * speed_mps / (2 * parameters.z_mps2) <= stop_station_m))) {
      state_.stopping = true;
      state_.stopping_started_sec = path.header.stamp.toSec();
      state_.stop_line_id = contact.id;
      state_.green_frames = 0;
    }
    limitIntersectionSpeed(output, contact);
    // A GO observed on the current approach lifts the blind-approach limit until that line changes.
    if (go_line_id_ != contact.id) go_line_id_ = 0;
    if (contact.id != 0 && signal == Signal::Go) go_line_id_ = contact.id;
    if (!state_.stopping) {
      limitBlindApproach(output, station, contact, signal, stop_offset_m);
      return {std::move(output), tf};
    }
    if (contact.id == 0) {
      for (auto& point : output.points) {point.velocity=0;point.acceleration=0;}
      return {std::move(output),tf};
    }

    const double response = parameters.z_mps2 * parameters.stop_response_time_sec;
    for (std::size_t i = 0; i < output.points.size(); ++i) {
      auto& point = output.points[i];
      const double remaining_m = stop_station_m - station[i];
      if (remaining_m <= parameters.stop_position_tolerance_m) {
        point.velocity = 0;
        point.acceleration = 0;
      } else if (remaining_m <= parameters.y_m) {
        // d = v^2/(2a) + v*T: retain forward motion until the stop target,
        // with response distance and a smooth low-speed approach near the line.
        const double braking = 2 * parameters.z_mps2 * remaining_m;
        const double limit = braking / (std::sqrt(response * response + braking) + response);
        if (limit < point.velocity) {
          point.velocity = limit;
          point.acceleration = std::min(point.acceleration,
              -parameters.z_mps2 * limit / (limit + response));
        }
      }
    }
    return {std::move(output), tf};
  }

 private:
  void releasePassedLine(const interfaces::Path& path) {
    if(!state_.stopping || path.points.empty())return;
    const auto found=std::find_if(state_.parameters.stop_lines.begin(),state_.parameters.stop_lines.end(),
        [&](const StopLineStateParameters::StopLine& line){return line.id==state_.stop_line_id;});
    if(found==state_.parameters.stop_lines.end())return;
    const auto& a=found->points.front();const auto& b=found->points.back();
    const double dx=b[0]-a[0],dy=b[1]-a[1],length=std::hypot(dx,dy);
    if(length<1e-9)return;
    double nx=-dy/length,ny=dx/length;
    const auto& pose=path.points.front().pose.pose;const auto& q=pose.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    if(!found->directional_extension) {
      const double alignment=nx*std::cos(yaw)+ny*std::sin(yaw);
      if(std::abs(alignment)<.5)return;
      if(alignment<0){nx=-nx;ny=-ny;}
    }
    const double passed=(pose.position.x-a[0])*nx+(pose.position.y-a[1])*ny;
    if(passed<=state_.parameters.stop_position_tolerance_m)return;
    state_.released_stop_line_id=state_.stop_line_id;state_.stopping=false;
    state_.stop_line_id=0;state_.green_frames=0;
  }
  static void reschedule(interfaces::Path& path,double horizon) {
    if(path.points.empty())return;
    auto source=std::move(path.points);path.points.clear();
    source.front().time_from_start=ros::Duration(0);path.points.push_back(source.front());
    double clock=0;
    for(std::size_t i=1;i<source.size();++i) {
      const auto& a=path.points.back();auto point=source[i];
      const double dx=point.pose.pose.position.x-a.pose.pose.position.x;
      const double dy=point.pose.pose.position.y-a.pose.pose.position.y,ds=std::hypot(dx,dy);
      if(ds<1e-9) {
        // Retain a scheduled stationary wait before a future departure.
        if(a.velocity==0 && point.velocity==0) {
          const double wait=std::max(0.,source[i].time_from_start.toSec()-source[i-1].time_from_start.toSec());
          clock=std::min(horizon,clock+wait);point.acceleration=0;point.time_from_start=ros::Duration(clock);
          if(point.time_from_start>path.points.back().time_from_start)path.points.push_back(point);
          if(clock>=horizon)return;
        }
        continue;
      }
      const double sum=a.velocity+point.velocity;
      if(sum<=1e-9)break;
      const double dt=2*ds/sum;
      const double acceleration=(point.velocity-a.velocity)/dt;
      if(clock+dt>horizon) {
        const double duration=std::max(0.,horizon-clock);
        const double distance=a.velocity*duration+.5*acceleration*duration*duration;
        const double u=std::max(0.,std::min(1.,distance/ds));
        point.pose.pose.position.x=a.pose.pose.position.x+u*dx;
        point.pose.pose.position.y=a.pose.pose.position.y+u*dy;
        point.pose.pose.position.z=a.pose.pose.position.z+u*(point.pose.pose.position.z-a.pose.pose.position.z);
        tf2::Quaternion qa,qb;tf2::fromMsg(a.pose.pose.orientation,qa);tf2::fromMsg(point.pose.pose.orientation,qb);
        point.pose.pose.orientation=tf2::toMsg(qa.slerp(qb,u));
        point.velocity=std::max(0.,a.velocity+acceleration*duration);
        point.acceleration=acceleration;point.time_from_start=ros::Duration(horizon);
        if(point.time_from_start>path.points.back().time_from_start)path.points.push_back(point);
        return;
      }
      clock+=dt;point.time_from_start=ros::Duration(clock);point.acceleration=acceleration;
      if(point.time_from_start<=path.points.back().time_from_start)continue;
      path.points.push_back(point);
      if(point.velocity==0)break;
    }
    if(path.points.back().velocity==0 && clock<horizon) {
      auto wait=path.points.back();wait.acceleration=0;wait.time_from_start=ros::Duration(horizon);
      if(wait.time_from_start>path.points.back().time_from_start)path.points.push_back(wait);
    }
  }
  enum class Signal { Ignore, Stop, Yellow, Go };
  struct Contact {
    std::int64_t id = 0;
    double station_m = std::numeric_limits<double>::infinity();
  };

  // While the light of the approached line is unobserved (UNKNOWN/OFF) and no GO was seen for it,
  // reach blind_detection_distance_m before the line no faster than the stop profile allows for a
  // target at that distance, so a RED first observed there still stops before the line.
  void limitBlindApproach(interfaces::Path& path, const std::vector<double>& station, const Contact& contact,
                          Signal signal, double stop_offset_m) const {
    const auto& p = state_.parameters;
    const double detection_m = parameters_.blind_detection_distance_m;
    if (detection_m <= 0 || contact.id == 0 || contact.id == state_.released_stop_line_id ||
        contact.id == go_line_id_ || signal != Signal::Ignore) return;
    const double response = p.z_mps2 * p.stop_response_time_sec;
    const double braking = 2 * p.z_mps2 * std::max(0.0, detection_m - stop_offset_m);
    const double creep_mps = braking / (std::sqrt(response * response + braking) + response);
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const double remaining_m = contact.station_m - station[i];
      if (remaining_m <= 0) continue;
      const double beyond_m = std::max(0.0, remaining_m - detection_m);
      const double limit = std::sqrt(creep_mps * creep_mps + 2 * parameters_.blind_deceleration_mps2 * beyond_m);
      auto& point = path.points[i];
      if (limit < point.velocity) {
        point.velocity = limit;
        point.acceleration = std::min(point.acceleration, beyond_m > 0 ? -parameters_.blind_deceleration_mps2 : 0.0);
      }
    }
  }

  // A teleport or checkpoint return moves the path origin farther than driving allows since the
  // previous cutoff call; a stop latched at the previous place does not apply to the new one.
  bool releaseAfterJump(const interfaces::Path& path) {
    if (path.points.empty()) return false;
    constexpr double kMaximumElapsedSec = 2.0;
    const auto& p = state_.parameters;
    const auto& origin = path.points.front().pose.pose.position;
    const double stamp_sec = path.header.stamp.toSec();
    bool jumped = false;
    if (state_.previous_path_stamp_sec >= 0) {
      const double elapsed_sec = std::max(0.0, std::min(kMaximumElapsedSec, stamp_sec - state_.previous_path_stamp_sec));
      const double moved_m = std::hypot(origin.x - state_.previous_path_x_m, origin.y - state_.previous_path_y_m);
      jumped = stamp_sec < state_.previous_path_stamp_sec || moved_m > p.jump_distance_m + p.jump_speed_mps * elapsed_sec;
      if (jumped) {
        state_.stopping = false;
        state_.stop_line_id = 0;
        state_.released_stop_line_id = 0;
        state_.green_frames = 0;
        // A rewound clock gives older image stamps; GO confirmation must count them again.
        state_.last_light_stamp_ns = 0;
        state_.intersection_remaining_m = -1.0;
      }
    }
    state_.previous_path_x_m = origin.x;
    state_.previous_path_y_m = origin.y;
    state_.previous_path_stamp_sec = stamp_sec;
    return jumped;
  }
  // Keep this calculation in each selected Core; the Bank carries only state.
  void limitIntersectionSpeed(interfaces::Path& path, const Contact& contact) {
    const auto& p = state_.parameters;
    if (p.intersection_speed_mps <= 0 || path.points.empty()) return;
    const auto& origin = path.points.front().pose.pose.position;
    if (state_.intersection_remaining_m >= 0) {
      state_.intersection_remaining_m -= std::hypot(origin.x - state_.intersection_previous_x_m,
                                                   origin.y - state_.intersection_previous_y_m);
    }
    state_.intersection_previous_x_m = origin.x;
    state_.intersection_previous_y_m = origin.y;
    if (contact.id != 0) state_.intersection_remaining_m = contact.station_m + p.intersection_after_m;
    if (state_.intersection_remaining_m < 0) return;
    const double distance = std::max(0.0, state_.intersection_remaining_m - p.intersection_after_m - p.intersection_before_m);
    const double limit = std::sqrt(p.intersection_speed_mps * p.intersection_speed_mps +
                                   2 * p.intersection_deceleration_mps2 * distance);
    for (auto& point : path.points) {
      if (point.velocity > limit) {
        point.velocity = limit;
        point.acceleration = std::min(point.acceleration, distance > 0 ? -p.intersection_deceleration_mps2 : 0.0);
      }
    }
  }

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
          auto a=line.points[j-1], b=line.points[j];
          const double dx=b[0]-a[0],dy=b[1]-a[1],length=std::hypot(dx,dy);
          if (length < 1e-9) continue;
          const double extension=state_.parameters.stop_line_extension_m;
          if (j==1) {a[0]-=extension*dx/length;a[1]-=extension*dy/length;}
          if (j+1==line.points.size()) {b[0]+=extension*dx/length;b[1]+=extension*dy/length;}
          const double ratio = intersectionRatio({{from.x, from.y}}, {{to.x, to.y}}, a, b);
          if (std::isfinite(ratio) && line.directional_extension &&
              (to.x-from.x)*(-dy)+(to.y-from.y)*dx <= 0 &&
              !std::isfinite(intersectionRatio({{from.x, from.y}}, {{to.x, to.y}}, line.points[j-1], line.points[j])))
            continue;  // An opposing MORAI stop line may not extend into this approach lane.
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

  IntegratedStopLeftParameters parameters_;
  StopLineState& state_;
  std::int64_t go_line_id_ = 0;
};
}  // namespace velocity_planner
