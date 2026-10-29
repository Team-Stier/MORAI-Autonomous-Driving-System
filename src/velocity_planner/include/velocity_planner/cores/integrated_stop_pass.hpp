#pragma once
#include <velocity_planner/libraries/localization_jump.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <velocity_planner/banks/stop_line_state.hpp>
#include <interfaces/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <utility>
#include <algorithm>
#include <cmath>

namespace velocity_planner {
class IntegratedStopPass {
 public:
  std::pair<interfaces::Path, tf2_msgs::TFMessage> process(const interfaces::Path& path, const tf2_msgs::TFMessage& tf) {
    releaseStopAfterJump(state_, tf, path.header.stamp.toSec());
    auto result=calculate(path,tf);
    const bool timed=std::any_of(path.points.begin(),path.points.end(),
        [](const interfaces::PathPoint& p){return !p.time_from_start.isZero();});
    bool changed=result.first.points.size()!=path.points.size();
    for(std::size_t i=0;i<path.points.size() && !changed;++i)
      changed=path.points[i].velocity!=result.first.points[i].velocity ||
              path.points[i].acceleration!=result.first.points[i].acceleration;
    if(timed && changed)reschedule(result.first,path.points.back().time_from_start.toSec());
    return result;
  }
  explicit IntegratedStopPass(StopLineState& state) : state_(state) {}
  std::pair<interfaces::Path, tf2_msgs::TFMessage> calculate(
      const interfaces::Path& path, const tf2_msgs::TFMessage& tf) {
    interfaces::Path output = path;
    releasePassedLine(path);
    limitIntersectionSpeed(output);
    state_.green_frames = 0;
    if (state_.stopping && path.header.stamp.toSec() - state_.stopping_started_sec >=
        state_.parameters.stop_timeout_sec) {
      state_.released_stop_line_id = state_.stop_line_id;
      state_.stopping = false;
      state_.stop_line_id = 0;
    }
    if (state_.stopping) {
      for (auto& point : output.points) {
        point.velocity = 0;
        point.acceleration = 0;
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
  // Keep this calculation in each selected Core; the Bank carries only state.
  void limitIntersectionSpeed(interfaces::Path& path) {
    const auto& p = state_.parameters;
    if (p.intersection_speed_mps <= 0 || path.points.empty()) return;
    const auto& origin = path.points.front().pose.pose.position;
    if (state_.intersection_remaining_m >= 0) {
      state_.intersection_remaining_m -= std::hypot(origin.x - state_.intersection_previous_x_m,
                                                   origin.y - state_.intersection_previous_y_m);
    }
    state_.intersection_previous_x_m = origin.x;
    state_.intersection_previous_y_m = origin.y;
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

  StopLineState& state_;
};
}  // namespace velocity_planner
