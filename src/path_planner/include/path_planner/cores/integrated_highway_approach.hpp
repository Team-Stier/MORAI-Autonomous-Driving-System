#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <lanelet2_core/LaneletMap.h>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <ros/console.h>
#include <node_pipeline/extension.hpp>
#include <visualization_msgs/Marker.h>
#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace integrated_highway_approach_detail {
using ::path_planner::Message;
using ::path_planner::Route;
using ::path_planner::PredictedObjects;
using ::path_planner::Checkpoint;
using ::path_planner::Checkpoints;
using ::path_planner::SharedMapParameters;
using ::path_planner::SharedMap;

// ---- inlined from libraries/integrated_path_mission.hpp ----
// Frozen mission behavior from local workspace 69419e3.
namespace path_planner {
struct PathPlannerMissionParameters {
  bool reference_smoothing_enabled=true;
  double refined_curve_smoothing_sigma_m=1.0;
  bool dynamic_zone_enabled = false;
  double dynamic_zone_min_x_m = -75;
  double dynamic_zone_max_x_m = -40;
  double dynamic_zone_min_y_m = -100;
  double dynamic_zone_max_y_m = 85;
  double dynamic_zone_minimum_detection_m = 20;
  double dynamic_zone_reaction_sec = 0.3;
  double dynamic_zone_braking_mps2 = 6;
  double dynamic_zone_clear_sec = 0.5;

  std::vector<lanelet::Id> refined_curve_lanelet_ids;
  double refined_curve_right_bias_m=0;
  bool use_boundary_midpoint_centerline = false;
  double centerline_step_m = 0.25;
  double local_path_length_m = 50;
  double sample_step_m = 0.5;
  double wheelbase_m = 3.0;
  double vehicle_front_m = 3.845008;
  double vehicle_rear_m = 0.79;
  double vehicle_half_width_m = 1.080418;
  double collision_margin_m = 0.25;
  double static_speed_mps = 0.3;
  double dynamic_speed_mps = 0.8;
  double dynamic_confirmation_sec = 0.3;
  double static_confirmation_sec = 0.5;
  double track_expiry_sec = 1.0;
  double dynamic_prediction_sec = 3.0;
  double avoidance_speed_mps = 3.0;
  double maximum_lateral_acceleration_mps2 = 1.5;
  double avoidance_lead_m = 2.0;
  double transition_length_m = 10.0;
  double return_transition_length_m = 14.0;
  double maximum_curvature_per_m = 0.25;
  double maximum_lane_offset_m = 6.0;
  double maximum_join_gap_m = 1.5;
  double completion_lateral_m = 0.35;
  double completion_heading_rad = 0.2;
  double stop_buffer_m = 2.0;
  double stopping_deceleration_mps2 = 2.0;
};

// Route extraction, motion classification and Lanelet lane changes stay in this Core.
class PathPlannerMission {
  struct Pose { double x, y, z, yaw; };
  struct LaneSpan {
    lanelet::ConstLanelet lane;
    double begin_m, end_m;
  };
  struct Track {
    interfaces::Object object;
    double last_stamp = 0;
    double fast_since = -1, slow_since = -1;
    bool moving = false, stationary = false;
  };
 public:
  PathPlannerMission(PathPlannerMissionParameters parameters, lanelet::LaneletMap& map)
      : parameters_(std::move(parameters)), planning_length_m_(parameters_.local_path_length_m),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(map, *rules_)) {}

  Message process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                  const interfaces::Objects& objects, const Route& route) {
    const double speed=std::abs(competition.velocity_x_mps);
    prediction_speed_mps_=speed;
    planning_length_m_=std::max(parameters_.local_path_length_m,
        speed*std::sqrt(6*parameters_.maximum_lane_offset_m/parameters_.maximum_lateral_acceleration_mps2)+
        parameters_.vehicle_front_m+parameters_.stop_buffer_m);
    reference_lane_change_=false;
    // A new lap can restart the clock. Object memories and a maneuver from the
    // later time would never expire, so plan from here as on the first call.
    if(ego.header.stamp.toSec()<last_call_sec_) {
      tracks_.clear();lane_change_.points.clear();normal_lane_change_=false;handover_m_=0;
      continuation_route_=Route{};continuation_index_=0;
      dynamic_zone_last_blocked_sec_=-std::numeric_limits<double>::infinity();
      ROS_INFO("[path_planner] planning state restarted after a clock rewind");
    }
    last_call_sec_=ego.header.stamp.toSec();
    updateTracks(objects, ego.header.stamp.toSec());
    publishMotion(objects);
    Message current;
    if (!lane_change_.points.empty()) {
      current = trim(lane_change_, ego.pose.pose.position);
      const auto& closest=current.points.front().pose.pose.position;
      const auto& position=ego.pose.pose.position;
      // Another section's Core may have driven beyond this saved maneuver.
      // Discard it before extending its old route into the current road.
      if(std::hypot(closest.x-position.x,closest.y-position.y)>parameters_.maximum_lane_offset_m) {
        lane_change_.points.clear();
        normal_lane_change_=false;
        handover_m_=0;
        continuation_route_=Route{};
        continuation_index_=0;
        ROS_INFO("[path_planner] discarded maneuver outside current road");
      }
    }
    if (!lane_change_.points.empty()) {
      extendAvoidanceTail(current);
      current = trim(current,ego.pose.pose.position);
      const auto& closest = current.points.front().pose.pose;
      const auto& position = ego.pose.pose.position;
      const double lateral_m = std::hypot(closest.position.x-position.x, closest.position.y-position.y);
      const double heading_rad = headingOf(ego.pose.pose)-headingOf(closest);
      const bool completed = signedStation(lane_change_,position.x,position.y)>=handover_m_ &&
          lateral_m<=parameters_.completion_lateral_m &&
          std::abs(std::atan2(std::sin(heading_rad),std::cos(heading_rad)))<=parameters_.completion_heading_rad;
      if (completed) {
        lane_change_.points.clear();
        ROS_INFO_STREAM("[path_planner] " <<
                        (normal_lane_change_?"route lane change completed":"avoidance return completed"));
        normal_lane_change_=false;
      } else if (distances(current).back()<parameters_.stop_buffer_m) {
        report("avoidance_end_stop");
        return stopped(std::move(current),0);
      }
    }
    if (lane_change_.points.empty()) current = reference(ego,route);
    const auto& zone_position = ego.pose.pose.position;
    const bool dynamic_zone = parameters_.dynamic_zone_enabled &&
        zone_position.x >= parameters_.dynamic_zone_min_x_m && zone_position.x <= parameters_.dynamic_zone_max_x_m &&
        zone_position.y >= parameters_.dynamic_zone_min_y_m && zone_position.y <= parameters_.dynamic_zone_max_y_m;
    if (dynamic_zone) {
      const double speed = std::abs(competition.velocity_x_mps);
      const double detection = std::max(parameters_.dynamic_zone_minimum_detection_m,
          speed*parameters_.dynamic_zone_reaction_sec + speed*speed/(2*parameters_.dynamic_zone_braking_mps2) +
          parameters_.vehicle_front_m + parameters_.stop_buffer_m);
      bool blocked_now = false;
      for (const auto& entry : tracks_)
        if (zoneCollision(current, entry.second) <= detection) {blocked_now=true;break;}
      const double now = ego.header.stamp.toSec();
      if (blocked_now) dynamic_zone_last_blocked_sec_ = now;
      if (blocked_now || now-dynamic_zone_last_blocked_sec_ < parameters_.dynamic_zone_clear_sec) {
        for (auto& point : current.points) {point.velocity=0;point.acceleration=0;}
        report("dynamic_zone_emergency_stop");
        return current;
      }
      // This zone waits for obstacles; it never initiates a static avoidance maneuver.
      report("dynamic_zone_clear");
      return current;
    }
    dynamic_zone_last_blocked_sec_ = -std::numeric_limits<double>::infinity();

    if (current.points.size()<2) {
      auto end=current.points.front();
      const double yaw=headingOf(ego.pose.pose);
      end.pose.pose.position.x+=parameters_.sample_step_m*std::cos(yaw);
      end.pose.pose.position.y+=parameters_.sample_step_m*std::sin(yaw);
      current.points.push_back(end);
      return stopped(std::move(current),0);
    }
    // Follow the fixed maneuver. Reconnecting from the actual rear axle every
    // tick erases tracking error and keeps moving the return curve forward.
    if(!lane_change_.points.empty()) {
      if(normal_lane_change_) {
        for(auto& point:current.points) point.velocity=std::numeric_limits<double>::quiet_NaN();
      } else limitManeuverSpeed(current);
    }
    double blocked = std::numeric_limits<double>::infinity();
    double unconfirmed = std::numeric_limits<double>::infinity();
    bool moving_collision=false;
    for (const auto& entry : tracks_) {
      const double collision = firstCollision(current, entry.second);
      if (!std::isfinite(collision)) continue;
      if(entry.second.moving) moving_collision=true;
      else if(!entry.second.stationary) unconfirmed=std::min(unconfirmed,collision);
      else blocked=std::min(blocked,collision);
    }
    if(moving_collision) {
      report("confirmed_dynamic_stop");
      return stopped(std::move(current),0);
    }
    if(std::isfinite(unconfirmed)) {
      report("motion_unconfirmed_brake");
      return stopped(std::move(current),std::max(0.0,std::min(unconfirmed,blocked)-parameters_.stop_buffer_m));
    }
    if (!std::isfinite(blocked)) {
      if(lane_change_.points.empty() && reference_lane_change_) {
        lane_change_=current;normal_lane_change_=true;
        handover_m_=reference_handover_m_;
        continuation_route_=route;continuation_index_=reference_end_index_;
      }
      report(lane_change_.points.empty()?"local_path":normal_lane_change_?"route_lane_change":"lanelet_lane_change");
      return current;
    }
    if (lane_change_.points.empty()) {
      Message candidate = changeLane(current, blocked,ego,competition);
      if (!candidate.points.empty()) {
        limitManeuverSpeed(candidate);
        lane_change_ = candidate;normal_lane_change_=false;
        continuation_route_ = route;
        continuation_index_ = reference_end_index_;
        report("lanelet_lane_change");
        return candidate;
      }
    }
    report("lanelet_blocked_stop");
    return stopped(std::move(current), std::max(0.0, blocked-parameters_.stop_buffer_m));
  }

 private:
  double prediction_speed_mps_=0;
  double last_call_sec_=-std::numeric_limits<double>::infinity();
  double dynamic_zone_last_blocked_sec_ = -std::numeric_limits<double>::infinity();
  static double headingOf(const geometry_msgs::Pose& p) {
    return std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                      1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));
  }
  void report(const std::string& mode) {
    if (mode != mode_) { ROS_INFO_STREAM("[path_planner] " << mode); mode_ = mode; }
  }
  void updateTracks(const interfaces::Objects& objects, double now) {
    const double stamp = objects.header.stamp.toSec();
    for (const auto& object : objects.objects) {
      auto& track = tracks_[object.id];
      if (stamp <= track.last_stamp) continue;
      if (stamp - track.last_stamp > parameters_.track_expiry_sec) track = Track{};
      const bool was_moving = track.moving;
      const double speed = object.velocity_available ? std::hypot(object.velocity.x, object.velocity.y) :
          std::numeric_limits<double>::infinity();
      if (!object.velocity_available) {
        track.fast_since = track.slow_since = -1;
        track.stationary = false;
      } else if (speed >= parameters_.dynamic_speed_mps) {
        track.slow_since = -1;
        if (track.fast_since < 0) track.fast_since = stamp;
        if (stamp - track.fast_since >= parameters_.dynamic_confirmation_sec) track.moving = true;
        track.stationary = false;
      } else if (speed <= parameters_.static_speed_mps) {
        track.fast_since = -1;
        if (track.slow_since < 0) track.slow_since = stamp;
        if (stamp - track.slow_since >= parameters_.static_confirmation_sec) track.moving = false;
        track.stationary = !track.moving;
      } else {
        track.fast_since = track.slow_since = -1;
        // The interval between static and dynamic thresholds retains an already
        // established static state; it cannot initialize one or clear moving.
        track.stationary = track.stationary && !track.moving;
      }
      if (was_moving != track.moving)
        ROS_INFO_STREAM("[path_planner] object " << object.id <<
                        (track.moving ? " moving" : " stationary") << " speed_mps=" << speed);
      track.object = object;
      track.last_stamp = stamp;
    }
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (now-it->second.last_stamp > parameters_.track_expiry_sec) it = tracks_.erase(it);
      else ++it;
    }
  }
  void publishMotion(const interfaces::Objects& objects) {
    visualization_msgs::Marker marker;
    marker.header = objects.header;
    marker.ns = "object_motion";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = objects.objects.empty() ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.28;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0.5);
    for (const auto& object : objects.objects) {
      const auto track = tracks_.find(object.id);
      std_msgs::ColorRGBA color;
      color.a = 1;
      if (track != tracks_.end() && track->second.moving) { color.r = 1; color.g = 0.15; color.b = 0.15; }
      else if (track != tracks_.end() && track->second.stationary) { color.r = 0.15; color.g = 0.55; color.b = 1; }
      else { color.r = 1; color.g = 0.85; color.b = 0.1; }
      const auto& q = object.bbox_pose.orientation;
      std::array<geometry_msgs::Point, 8> corners;
      for (unsigned i = 0; i < corners.size(); ++i) {
        const double x = (i & 1 ? 0.5 : -0.5)*object.bbox_size.x;
        const double y = (i & 2 ? 0.5 : -0.5)*object.bbox_size.y;
        const double z = (i & 4 ? 0.5 : -0.5)*object.bbox_size.z;
        auto& p = corners[i];
        p.x = object.bbox_pose.position.x + (1-2*(q.y*q.y+q.z*q.z))*x + 2*(q.x*q.y-q.w*q.z)*y + 2*(q.x*q.z+q.w*q.y)*z;
        p.y = object.bbox_pose.position.y + 2*(q.x*q.y+q.w*q.z)*x + (1-2*(q.x*q.x+q.z*q.z))*y + 2*(q.y*q.z-q.w*q.x)*z;
        p.z = object.bbox_pose.position.z + 2*(q.x*q.z-q.w*q.y)*x + 2*(q.y*q.z+q.w*q.x)*y + (1-2*(q.x*q.x+q.y*q.y))*z;
      }
      for (unsigned i = 0; i < corners.size(); ++i) {
        for (unsigned axis = 0; axis < 3; ++axis) {
          const unsigned neighbor = i ^ (1u << axis);
          if (neighbor <= i) continue;
          marker.points.push_back(corners[i]); marker.colors.push_back(color);
          marker.points.push_back(corners[neighbor]); marker.colors.push_back(color);
        }
      }
    }
    motion_visualizer_.publish(marker);
  }
  static std::vector<double> distances(const Message& path) {
    std::vector<double> s(path.points.size(), 0);
    for (std::size_t i=1;i<s.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position; const auto& b=path.points[i].pose.pose.position;
      s[i]=s[i-1]+std::hypot(b.x-a.x,b.y-a.y);
    }
    return s;
  }
  static Pose sample(const Message& path, const std::vector<double>& s, double station) {
    auto upper=std::upper_bound(s.begin(),s.end(),station);
    const std::size_t right=std::min(s.size()-1,std::max<std::size_t>(1,upper-s.begin()));
    const std::size_t left=right-1;
    const double t=std::max(0.0,std::min(1.0,(station-s[left])/(s[right]-s[left])));
    const auto& a=path.points[left].pose.pose.position; const auto& b=path.points[right].pose.pose.position;
    return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z),std::atan2(b.y-a.y,b.x-a.x)};
  }
  static void headings(Message& path) {
    for (std::size_t i=0;i+1<path.points.size();++i) {
      auto& q=path.points[i].pose.pose.orientation;
      const auto& a=path.points[i].pose.pose.position; const auto& b=path.points[i+1].pose.pose.position;
      const double yaw=std::atan2(b.y-a.y,b.x-a.x); q.x=q.y=0; q.z=std::sin(yaw/2); q.w=std::cos(yaw/2);
    }
    path.points.back().pose.pose.orientation=path.points[path.points.size()-2].pose.pose.orientation;
  }
  Message resample(const Message& input) const {
    const auto s=distances(input); Message result;
    for (double d=0;d<s.back();d+=parameters_.sample_step_m) {
      const auto p=sample(input,s,d); appendPoint(result,{p.x,p.y,p.z});
    }
    result.points.push_back(input.points.back()); headings(result); return result;
  }
  static double signedStation(const Message& path, double x, double y) {
    double station=0,nearest=std::numeric_limits<double>::infinity(),chosen=0;
    for (std::size_t i=1;i<path.points.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position;const auto& b=path.points[i].pose.pose.position;
      const double dx=b.x-a.x,dy=b.y-a.y,len=std::hypot(dx,dy);
      if (len==0) continue;
      const double raw=((x-a.x)*dx+(y-a.y)*dy)/(len*len);
      const double t=std::max(0.0,std::min(1.0,raw));
      const double distance=std::hypot(x-a.x-t*dx,y-a.y-t*dy);
      if (distance<nearest) {nearest=distance;chosen=station+((i==1&&raw<0)?raw:t)*len;}
      station+=len;
    }
    return chosen;
  }
  // Preserve the configured dynamic-zone emergency detection envelope.
  double zoneCollision(const Message& path, const Track& track) const {
    const auto& object = track.object;
    const auto stations = distances(path);
    double collision = std::numeric_limits<double>::infinity();
    const int steps = track.stationary || !object.velocity_available ? 0 :
        static_cast<int>(std::ceil(parameters_.dynamic_prediction_sec/0.25));
    for (std::size_t i=1;i<path.points.size();++i) {
      const auto& a = path.points[i-1].pose.pose.position;
      const auto& b = path.points[i].pose.pose.position;
      const double dx=b.x-a.x, dy=b.y-a.y, length=std::hypot(dx,dy);
      if (length == 0) continue;
      const double yaw=std::atan2(dy,dx), relative=headingOf(object.bbox_pose)-yaw;
      const double half_width=parameters_.vehicle_half_width_m+parameters_.collision_margin_m+
          (std::abs(std::sin(relative))*object.bbox_size.x+std::abs(std::cos(relative))*object.bbox_size.y)/2;
      const double half_length=(std::abs(std::cos(relative))*object.bbox_size.x+
          std::abs(std::sin(relative))*object.bbox_size.y)/2;
      for (int k=0;k<=steps;++k) {
        const double t=0.25*k;
        const double x=object.bbox_pose.position.x+(steps?object.velocity.x*t:0)-a.x;
        const double y=object.bbox_pose.position.y+(steps?object.velocity.y*t:0)-a.y;
        const double along=(x*dx+y*dy)/length, lateral=(-x*dy+y*dx)/length;
        if (std::abs(lateral)>half_width || along < -half_length-parameters_.vehicle_rear_m ||
            along>length+half_length+parameters_.vehicle_front_m) continue;
        collision=std::min(collision,std::max(0.0,stations[i-1]+along-half_length-parameters_.vehicle_front_m));
      }
    }
    return collision;
  }

  // Compare rear traffic with ego at the same future time. Front and crossing
  // traffic retain the existing stopping envelope; static geometry stays complete.
  double firstCollision(const Message& path,const Track& track) const {
    const auto& object=track.object;const auto station=distances(path);
    const bool moving=!track.stationary && object.velocity_available &&
        std::hypot(object.velocity.x,object.velocity.y)>=parameters_.static_speed_mps;
    if(moving && path.points.size()>1) {
      const auto& a=path.points[0].pose.pose.position;const auto& b=path.points[1].pose.pose.position;
      const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
      const double along=length>0?((object.bbox_pose.position.x-a.x)*dx+(object.bbox_pose.position.y-a.y)*dy)/length:0;
      if(along>=-parameters_.vehicle_rear_m)return zoneCollision(path,track);
    }
    const double travel_speed=moving?std::max(.01,prediction_speed_mps_):1.;
    const double vx=moving?object.velocity.x:0,vy=moving?object.velocity.y:0;
    const double object_yaw=headingOf(object.bbox_pose),oc=std::cos(object_yaw),os=std::sin(object_yaw);
    const double center=(parameters_.vehicle_front_m-parameters_.vehicle_rear_m)/2;
    const double half_length=(parameters_.vehicle_front_m+parameters_.vehicle_rear_m)/2;
    const double half_width=parameters_.vehicle_half_width_m+parameters_.collision_margin_m;
    for(std::size_t i=1;i<path.points.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position;const auto& b=path.points[i].pose.pose.position;
      const double length=std::hypot(b.x-a.x,b.y-a.y);if(length==0)continue;
      const double begin=moving?station[i-1]/travel_speed:0;
      if(moving && begin>parameters_.dynamic_prediction_sec)break;
      const double duration=moving?std::min(length/travel_speed,parameters_.dynamic_prediction_sec-begin):length;
      const double c=(b.x-a.x)/length,s=(b.y-a.y)/length;
      const double x=object.bbox_pose.position.x+vx*begin-a.x-center*c;
      const double y=object.bbox_pose.position.y+vy*begin-a.y-center*s;
      double lo=0,hi=duration;
      for(const auto& axis:std::array<std::array<double,2>,4>{{{{c,s}},{{-s,c}},{{oc,os}},{{-os,oc}}}}) {
        const double forward=axis[0]*c+axis[1]*s,lateral=-axis[0]*s+axis[1]*c;
        const double radius=half_length*std::abs(forward)+half_width*std::abs(lateral)+
            .5*object.bbox_size.x*std::abs(axis[0]*oc+axis[1]*os)+.5*object.bbox_size.y*std::abs(-axis[0]*os+axis[1]*oc);
        const double d=axis[0]*x+axis[1]*y;
        const double rate=travel_speed*forward-axis[0]*vx-axis[1]*vy;
        if(std::abs(rate)<1e-10) {if(std::abs(d)>radius){lo=duration+1;break;}}
        else {double first=(d-radius)/rate,last=(d+radius)/rate;if(first>last)std::swap(first,last);lo=std::max(lo,first);hi=std::min(hi,last);}
        if(lo>hi)break;
      }
      if(lo<=hi)return station[i-1]+travel_speed*lo;
    }
    return std::numeric_limits<double>::infinity();
  }

  Message trim(const Message& path, const geometry_msgs::Point& position) const {
    lanelet::BasicLineString3d line;
    for (const auto& p:path.points) {
      const auto& q=p.pose.pose.position;line.emplace_back(q.x,q.y,q.z);
    }
    const auto projection=project(line,{position.x,position.y});
    Message result;appendPoint(result,projection.point);
    double length=0;appendLine(result,line,projection.segment+1,length);
    if (result.points.size()==1) {
      auto endpoint=result.points.front();
      const double yaw=headingOf(path.points.back().pose.pose);
      endpoint.pose.pose.position.x+=parameters_.sample_step_m*std::cos(yaw);
      endpoint.pose.pose.position.y+=parameters_.sample_step_m*std::sin(yaw);
      result.points.push_back(endpoint);
    }
    headings(result);
    for(auto& p:result.points) p.velocity=parameters_.avoidance_speed_mps;
    return result;
  }

  static double blend(double t) {
    t=std::max(0.0,std::min(1.0,t));
    return t*t*t*(10+t*(-15+6*t));
  }
  bool follows(const lanelet::ConstLanelet& from,const lanelet::ConstLanelet& to) const {
    const auto next=graph_->following(from,false);
    return std::find(next.begin(),next.end(),to)!=next.end();
  }
  bool adjacent(const lanelet::ConstLanelet& from,const lanelet::ConstLanelet& to) const {
    const auto right=graph_->right(from),left=graph_->left(from);
    return (right && *right==to) || (left && *left==to);
  }
  bool connected(const lanelet::ConstLanelet& from,const lanelet::ConstLanelet& to) {
    if(!follows(from,to)) return false;
    return (referenceCenterline(from).back()-referenceCenterline(to).front()).head<2>().norm()<=
        parameters_.maximum_join_gap_m;
  }
  lanelet::BasicLineString3d forwardLine(const lanelet::routing::LaneletPath& route,std::size_t index) {
    auto line=referenceCenterline(route[index]);
    for(std::size_t i=index+1;i<route.size() && connected(route[i-1],route[i]);++i) {
      const auto& next=referenceCenterline(route[i]);
      for(const auto& point:next)
        if((point-line.back()).head<2>().norm()>1e-6) line.push_back(point);
    }
    return line;
  }
  void extendAvoidanceTail(Message& path) {
    const auto line=forwardLine(*continuation_route_,continuation_index_);
    const auto& tail=path.points.back().pose.pose.position;
    const auto end=project(line,{tail.x,tail.y});
    double length=distances(path).back();
    appendLine(path,line,end.segment+1,length);
    path=resample(path);
    for(auto& point:path.points) point.velocity=parameters_.avoidance_speed_mps;
  }
  // Continue only alongside consecutive lanes in the supplied checkpoint route.
  lanelet::BasicLineString3d parallelLine(const lanelet::routing::LaneletPath& route,
                                         std::size_t target_index,lanelet::ConstLanelet source) {
    auto line=referenceCenterline(source);
    for(std::size_t i=target_index+1;i<route.size() && connected(route[i-1],route[i]);++i) {
      const auto left=graph_->left(route[i]),right=graph_->right(route[i]);
      lanelet::Optional<lanelet::ConstLanelet> next;
      if(left && connected(source,*left)) next=*left;
      else if(right && connected(source,*right)) next=*right;
      if(!next) break;
      const auto& points=referenceCenterline(*next);
      for(const auto& point:points)
        if((point-line.back()).head<2>().norm()>1e-6) line.push_back(point);
      source=*next;
    }
    return line;
  }
  bool drivable(const Message& path) const {
    for(std::size_t i=1;i+1<path.points.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position;
      const auto& b=path.points[i].pose.pose.position;
      const auto& c=path.points[i+1].pose.pose.position;
      const double ux=b.x-a.x,uy=b.y-a.y,vx=c.x-b.x,vy=c.y-b.y;
      const double first=std::hypot(ux,uy),second=std::hypot(vx,vy);
      if(first<1e-6 || second<1e-6) continue;
      const double angle=std::abs(std::atan2(ux*vy-uy*vx,ux*vx+uy*vy));
      if(angle/((first+second)/2)>parameters_.maximum_curvature_per_m) return false;
    }
    return true;
  }
  Message changeLane(const Message& base,double blocked,const nav_msgs::Odometry& ego,
                     const interfaces::CompetitionVehicleStatus& competition) {
    const auto sampled=resample(base);
    const auto stations=distances(sampled);
    double pass_m=0;
    for(const auto& entry:tracks_) {
      if(!std::isfinite(firstCollision(base,entry.second))) continue;
      const auto& object=entry.second.object;
      const double station=signedStation(base,object.bbox_pose.position.x,object.bbox_pose.position.y);
      const auto road=sample(sampled,stations,station);
      const double relative=headingOf(object.bbox_pose)-road.yaw;
      const double extent=(std::abs(std::cos(relative))*object.bbox_size.x+
                           std::abs(std::sin(relative))*object.bbox_size.y)/2;
      pass_m=std::max(pass_m,station+extent+parameters_.vehicle_rear_m+parameters_.stop_buffer_m);
    }
    // Spans contain only the ordered route geometry within the planning horizon.
    for(bool right:{true,false}) for(std::size_t span_index=0;span_index<reference_spans_.size();++span_index) {
      const auto& span=reference_spans_[span_index];
      const auto neighbor=right?graph_->right(span.lane):graph_->left(span.lane);
      if(!neighbor || span.begin_m>=blocked) continue;
      // Begin a small in-lane preparation before the neighboring span starts.
      const double begin=std::max(0.0,span.begin_m-parameters_.avoidance_lead_m);
      const auto start=sample(sampled,stations,begin);
      const auto legal_start=sample(sampled,stations,span.begin_m);
      const auto& neighbor_line=referenceCenterline(*neighbor);
      const auto projection=project(neighbor_line,{legal_start.x,legal_start.y});
      const double offset=std::sqrt(projection.squared_distance);
      if(offset<0.5 || offset>parameters_.maximum_lane_offset_m) continue;
      Message target;
      appendPoint(target,projection.point+lanelet::BasicPoint3d(start.x-legal_start.x,start.y-legal_start.y,start.z-legal_start.z));
      double length=0;appendLine(target,lanelet::BasicLineString3d{projection.point},0,length);
      appendLine(target,neighbor_line,projection.segment+1,length);
      auto previous_neighbor=*neighbor;
      double corridor_end=span.end_m;
      for(std::size_t i=span_index+1;i<reference_spans_.size();++i) {
        const auto& next_span=reference_spans_[i];
        const auto next=right?graph_->right(next_span.lane):graph_->left(next_span.lane);
        if(!next || !connected(reference_spans_[i-1].lane,next_span.lane) ||
            !connected(previous_neighbor,*next)) break;
        const auto& line=referenceCenterline(*next);
        appendLine(target,line,0,length);
        corridor_end=next_span.end_m;
        previous_neighbor=*next;
      }
      if(target.points.size()<2) continue;
      const auto target_s=distances(target);
      const double transition=std::max(parameters_.transition_length_m,
          std::sqrt(6*offset/parameters_.maximum_curvature_per_m));
      // Depart at the earliest legal span. Feasibility is decided on the swept
      // rear-axle trajectory with the full asymmetric vehicle footprint below.
      const double return_begin=std::max(begin+transition,pass_m);
      const double return_space=std::min({corridor_end-return_begin,stations.back()-return_begin,
                                         target_s.back()+begin-return_begin});
      const double return_transition=std::max(transition,
          std::min(parameters_.return_transition_length_m,return_space));
      const double return_end=return_begin+return_transition;
      if(return_end>corridor_end || return_end>stations.back() || return_end-begin>target_s.back()) continue;
      const auto first_target=sample(target,target_s,0);
      if(std::cos(first_target.yaw-start.yaw)<0.8) continue;
      Message candidate=sampled;
      for(std::size_t i=0;i<candidate.points.size();++i) {
        const double station=stations[i];
        auto& point=candidate.points[i];
        if(station>begin && station<return_end) {
          const auto destination=sample(target,target_s,station-begin);
          const double weight=station<begin+transition?blend((station-begin)/transition):
              station<return_begin?1:1-blend((station-return_begin)/return_transition);
          auto& p=point.pose.pose.position;
          p.x+=(destination.x-p.x)*weight;
          p.y+=(destination.y-p.y)*weight;
          p.z+=(destination.z-p.z)*weight;
        }
        point.velocity=parameters_.avoidance_speed_mps;
      }
      headings(candidate);
      candidate=rearAxleConnection(candidate,ego,competition,begin+transition);
      if(candidate.points.empty() || !drivable(candidate)) continue;
      bool occupied=false;
      for(const auto& entry:tracks_)
        if(std::isfinite(firstCollision(candidate,entry.second))) {occupied=true;break;}
      if(occupied) continue;
      // Completion station is measured on the complete immutable avoidance path.
      const auto end_pose=sample(sampled,stations,return_end);
      handover_m_=signedStation(candidate,end_pose.x,end_pose.y);
      return candidate;
    }
    return {};
  }
  // Path poses describe the rear axle, with p'=heading and k=tan(steering)/wheelbase.
  Message rearAxleConnection(const Message& path,const nav_msgs::Odometry& ego,
                             const interfaces::CompetitionVehicleStatus& competition,
                             double maximum_join_m=std::numeric_limits<double>::infinity()) const {
    const auto stations=distances(path);
    const auto& origin=ego.pose.pose.position;
    const double yaw=headingOf(ego.pose.pose);
    // MORAI competition status reports the opposite sign to ROS left-positive
    // yaw and the normalized control command (verified against recorded yaw).
    const double curvature=-std::tan(competition.steering_angle_rad)/parameters_.wheelbase_m;
    const auto& first=path.points.front().pose.pose;
    const double difference=headingOf(first)-yaw;
    if(std::hypot(first.position.x-origin.x,first.position.y-origin.y)<1e-6 &&
        std::abs(std::atan2(std::sin(difference),std::cos(difference)))<1e-4 &&
        std::abs(curvature)<1e-4) return path;
    for(double join=parameters_.transition_length_m;join<=std::min({stations.back(),3*parameters_.transition_length_m,maximum_join_m});join+=1.0) {
      const auto end=sample(path,stations,join);
      const auto before=sample(path,stations,std::max(0.0,join-0.5));
      const auto after=sample(path,stations,std::min(stations.back(),join+0.5));
      const double angle=after.yaw-before.yaw;
      const double end_curvature=std::atan2(std::sin(angle),std::cos(angle))/
          (std::min(stations.back(),join+0.5)-std::max(0.0,join-0.5));
      const double scale=join;
      const std::array<double,2> start{{origin.x,origin.y}},finish{{end.x,end.y}};
      const std::array<double,2> tangent{{std::cos(yaw),std::sin(yaw)}},last_tangent{{std::cos(end.yaw),std::sin(end.yaw)}};
      std::array<std::array<double,6>,2> coefficients;
      for(unsigned axis=0;axis<2;++axis) {
        const double v0=scale*tangent[axis],v1=scale*last_tangent[axis];
        const double a0=scale*scale*curvature*(axis==0?-tangent[1]:tangent[0]);
        const double a1=scale*scale*end_curvature*(axis==0?-last_tangent[1]:last_tangent[0]);
        const double r=finish[axis]-start[axis]-v0-a0/2,v=v1-v0-a0,a=a1-a0;
        coefficients[axis]={{start[axis],v0,a0/2,10*r-4*v+a/2,-15*r+7*v-a,6*r-3*v+a/2}};
      }
      Message result;
      const int steps=static_cast<int>(std::ceil(scale/std::min(0.25,parameters_.sample_step_m)));
      bool feasible=true;
      for(int i=0;i<=steps;++i) {
        const double t=static_cast<double>(i)/steps;
        std::array<double,2> p,d,dd;
        for(unsigned axis=0;axis<2;++axis) {
          const auto& c=coefficients[axis];
          p[axis]=c[0]+t*(c[1]+t*(c[2]+t*(c[3]+t*(c[4]+t*c[5]))));
          d[axis]=c[1]+t*(2*c[2]+t*(3*c[3]+t*(4*c[4]+t*5*c[5])));
          dd[axis]=2*c[2]+t*(6*c[3]+t*(12*c[4]+t*20*c[5]));
        }
        const double speed=std::hypot(d[0],d[1]);
        if(speed<1e-6 || d[0]*(end.x-origin.x)+d[1]*(end.y-origin.y)<=0 ||
            std::abs(d[0]*dd[1]-d[1]*dd[0])/(speed*speed*speed)>parameters_.maximum_curvature_per_m) {
          feasible=false;break;
        }
        appendPoint(result,{p[0],p[1],origin.z+t*(end.z-origin.z)});
        auto& orientation=result.points.back().pose.pose.orientation;
        const double heading=std::atan2(d[1],d[0]);
        orientation.z=std::sin(heading/2);orientation.w=std::cos(heading/2);
      }
      if(!feasible) continue;
      const auto next=std::upper_bound(stations.begin(),stations.end(),join+1e-6);
      result.points.insert(result.points.end(),path.points.begin()+(next-stations.begin()),path.points.end());
      if(!drivable(result)) continue;
      bool occupied=false;
      for(const auto& entry:tracks_) if(std::isfinite(firstCollision(result,entry.second))) {occupied=true;break;}
      if(occupied) continue;
      for(auto& point:result.points) point.velocity=parameters_.avoidance_speed_mps;
      return result;
    }
    return {};
  }
  void limitManeuverSpeed(Message& path) const {
    const auto station=distances(path);
    for(auto& point:path.points) {point.velocity=parameters_.avoidance_speed_mps;point.acceleration=0;}
    for(std::size_t i=1;i+1<path.points.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position;
      const auto& b=path.points[i].pose.pose.position;
      const auto& c=path.points[i+1].pose.pose.position;
      const double ux=b.x-a.x,uy=b.y-a.y,vx=c.x-b.x,vy=c.y-b.y;
      const double denominator=std::hypot(ux,uy)*std::hypot(vx,vy)*std::hypot(c.x-a.x,c.y-a.y);
      if(denominator<1e-10) continue;
      const double curvature=2*std::abs(ux*vy-uy*vx)/denominator;
      if(curvature<1e-8) continue;
      const double speed=std::sqrt(parameters_.maximum_lateral_acceleration_mps2/curvature);
      // Bound both segments adjoining the fitted arc, including its endpoints.
      for(std::size_t j=i-1;j<=i+1;++j) path.points[j].velocity=std::min(path.points[j].velocity,speed);
    }
    // Brake before the curve rather than reducing the speed only inside it.
    for(std::size_t i=path.points.size()-1;i>0;--i) {
      const double next=path.points[i].velocity;
      path.points[i-1].velocity=std::min(path.points[i-1].velocity,
          std::sqrt(next*next+2*parameters_.stopping_deceleration_mps2*(station[i]-station[i-1])));
    }
    for(std::size_t i=0;i+1<path.points.size();++i) {
      const double length=station[i+1]-station[i];
      if(length>1e-8) path.points[i].acceleration=std::min(0.0,
          (path.points[i+1].velocity*path.points[i+1].velocity-path.points[i].velocity*path.points[i].velocity)/(2*length));
    }
  }
  Message stopped(Message path,double stop_station) const {
    const auto s=distances(path);
    for(std::size_t i=0;i<path.points.size();++i) {
      auto& p=path.points[i];p.velocity=std::min(std::isnan(p.velocity)?parameters_.avoidance_speed_mps:p.velocity,
          std::sqrt(2*parameters_.stopping_deceleration_mps2*std::max(0.0,stop_station-s[i])));
      p.acceleration=p.velocity>0?-parameters_.stopping_deceleration_mps2:0;
    }
    return path;
  }
  // Temporary experiment: pair boundary positions at the same normalized arclength.
  // The switch selects the source geometry before bounded reference smoothing.
  // Cache a world-fixed reference before projecting the moving ego onto it.
  // Gaussian arclength convolution removes short lateral teeth without fitting
  // long curves across a junction or changing the map boundaries.
  lanelet::BasicLineString3d smoothReference(const lanelet::ConstLanelet& lane,
                                            lanelet::BasicLineString3d source) const {
    if(!parameters_.reference_smoothing_enabled)return source;
    constexpr double step=.25, maximum_shift=.20;
    const bool refined=std::find(parameters_.refined_curve_lanelet_ids.begin(),
        parameters_.refined_curve_lanelet_ids.end(),lane.id())!=parameters_.refined_curve_lanelet_ids.end();
    const double sigma=refined?parameters_.refined_curve_smoothing_sigma_m:1.0;
    const int radius=sigma==1.0?8:static_cast<int>(std::ceil(3*sigma/step));
    auto before=graph_->previous(lane,false),after=graph_->following(lane,false);
    lanelet::BasicLineString3d prefix,suffix;
    double first_gap=.4,last_gap=.4;
    for(const auto& other:before) {
      const auto line=unsmoothedReference(other);
      const double gap=(line.back()-source.front()).head<2>().norm();
      if(gap<first_gap){first_gap=gap;prefix=line;}
    }
    for(const auto& other:after) {
      const auto line=unsmoothedReference(other);
      const double gap=(line.front()-source.back()).head<2>().norm();
      if(gap<last_gap){last_gap=gap;suffix=line;}
    }
    if(!prefix.empty())source.front()=(source.front()+prefix.back())/2;
    if(!suffix.empty())source.back()=(source.back()+suffix.front())/2;
    lanelet::BasicLineString3d context;
    if(!prefix.empty()) {prefix.back()=source.front();context=prefix;context.pop_back();}
    const std::size_t begin=context.size();
    context.insert(context.end(),source.begin(),source.end());
    const std::size_t end=context.size()-1;
    if(!suffix.empty())context.insert(context.end(),suffix.begin()+1,suffix.end());
    std::vector<double> station(context.size(),0.);
    for(std::size_t i=1;i<context.size();++i)station[i]=station[i-1]+(context[i]-context[i-1]).head<2>().norm();
    const auto at=[&](double s)->lanelet::BasicPoint3d {
      if(s<=0) {
        const auto index=std::min<std::size_t>(context.size()-1,std::upper_bound(station.begin(),station.end(),2.)-station.begin());
        const auto delta=(context[index]-context.front()).eval();
        return context.front()+s*delta/delta.head<2>().norm();
      }
      if(s>=station.back()) {
        const auto index=std::min<std::size_t>(context.size()-2,std::lower_bound(station.begin(),station.end(),station.back()-2.)-station.begin());
        const auto delta=(context.back()-context[index]).eval();
        return context.back()+(s-station.back())*delta/delta.head<2>().norm();
      }
      const auto k=std::upper_bound(station.begin(),station.end(),s)-station.begin();
      return context[k-1]+(s-station[k-1])/(station[k]-station[k-1])*(context[k]-context[k-1]);
    };
    lanelet::BasicLineString3d result;
    const double length=station[end]-station[begin];
    const unsigned count=std::max(1u,static_cast<unsigned>(std::ceil(length/step)));
    for(unsigned i=0;i<=count;++i) {
      const double s=station[begin]+length*i/count;
      const auto original=at(s);lanelet::BasicPoint3d value=lanelet::BasicPoint3d::Zero();double weight=0;
      for(int j=-radius;j<=radius;++j) {
        const double w=std::exp(-.5*std::pow(j*step/sigma,2));value+=w*at(s+j*step);weight+=w;
      }
      value/=weight;auto displacement=(value-original).head<2>().eval();
      if(displacement.norm()>maximum_shift)displacement*=maximum_shift/displacement.norm();
      value=original;value.head<2>()+=displacement;result.push_back(value);
    }
    return result;
  }

  const lanelet::BasicLineString3d& referenceCenterline(const lanelet::ConstLanelet& lane) {
    const auto key=std::make_pair(lane.id(),lane.inverted());
    const auto cached=centerlines_.find(key);
    if(cached!=centerlines_.end())return cached->second;
    return centerlines_.emplace(key,smoothReference(lane,unsmoothedReference(lane))).first->second;
  }
  lanelet::BasicLineString3d unsmoothedReference(const lanelet::ConstLanelet& lane) const {
    lanelet::BasicLineString3d result;
    if (!parameters_.use_boundary_midpoint_centerline) {
      result=lane.centerline3d().basicLineString();
    } else {
      const auto left=lane.leftBound3d().basicLineString();
      const auto right=lane.rightBound3d().basicLineString();
      auto stations=[](const lanelet::BasicLineString3d& line) {
        std::vector<double> s(line.size(),0);
        for (std::size_t i=1;i<line.size();++i) s[i]=s[i-1]+(line[i]-line[i-1]).head<2>().norm();
        return s;
      };
      const auto left_s=stations(left),right_s=stations(right);
      auto interpolate=[](const lanelet::BasicLineString3d& line,const std::vector<double>& s,double d) -> lanelet::BasicPoint3d {
        if (d<=0) return line.front();
        if (d>=s.back()) return line.back();
        const auto upper=std::upper_bound(s.begin(),s.end(),d);
        const std::size_t right=static_cast<std::size_t>(upper-s.begin()),left=right-1;
        const double t=(d-s[left])/(s[right]-s[left]);
        return line[left]+t*(line[right]-line[left]);
      };
      const double length=(left_s.back()+right_s.back())/2;
      const std::size_t count=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(length/parameters_.centerline_step_m)));
      result.reserve(count+1);
      for (std::size_t i=0;i<=count;++i) {
        const double ratio=static_cast<double>(i)/count;
        result.push_back((interpolate(left,left_s,ratio*left_s.back())+
                          interpolate(right,right_s,ratio*right_s.back()))/2);
      }
    }
    const auto selected=[&](const lanelet::ConstLanelet& q) {
      return std::find(parameters_.refined_curve_lanelet_ids.begin(),parameters_.refined_curve_lanelet_ids.end(),q.id())!=parameters_.refined_curve_lanelet_ids.end();
    };
    const auto predecessors=graph_->previous(lane,false),successors=graph_->following(lane,false);
    const bool here=selected(lane);
    bool before=false,after=false;
    for(const auto& q:predecessors)before=before||selected(q);
    for(const auto& q:successors)after=after||selected(q);
    if(parameters_.refined_curve_right_bias_m>0 && (here||before||after)) {
      const auto original=result;
      const auto p0=original.front(),p1=original.back();
      auto t0=(original[1]-p0).head<2>().normalized().eval();
      auto t1=(p1-original[original.size()-2]).head<2>().normalized().eval();
      if(here && !predecessors.empty()) {
        const auto q=predecessors.front().centerline3d().basicLineString();
        const auto incoming=(q.back()-q[q.size()-2]).head<2>().normalized().eval();
        if(t0.dot(incoming)>.5)t0=(t0+incoming).normalized().eval();
      }
      if(here && !successors.empty()) {
        const auto q=successors.front().centerline3d().basicLineString();
        const auto outgoing=(q[1]-q.front()).head<2>().normalized().eval();
        if(t1.dot(outgoing)>.5)t1=(t1+outgoing).normalized().eval();
      }
      std::vector<double> station(original.size(),0);
      for(std::size_t i=1;i<station.size();++i)station[i]=station[i-1]+(original[i]-original[i-1]).head<2>().norm();
      const double total=station.back();const unsigned count=std::max(2u,static_cast<unsigned>(std::ceil(total/.25)));
      result.clear();
      for(unsigned i=0;i<=count;++i) {
        const double u=static_cast<double>(i)/count,u2=u*u,u3=u2*u;
        lanelet::BasicPoint3d point;
        lanelet::BasicPoint2d tangent;
        if(here) {
          point=(2*u3-3*u2+1)*p0+(-2*u3+3*u2)*p1;
          point.head<2>()+=(u3-2*u2+u)*total*t0+(u3-u2)*total*t1;
          tangent=(6*u2-6*u)*p0.head<2>()+(-6*u2+6*u)*p1.head<2>()+
                  (3*u2-4*u+1)*total*t0+(3*u2-2*u)*total*t1;
        } else {
          const auto upper=std::upper_bound(station.begin(),station.end(),u*total);
          const std::size_t j=std::min(original.size()-1,std::max<std::size_t>(1,upper-station.begin()));
          const double w=(u*total-station[j-1])/(station[j]-station[j-1]);
          point=original[j-1]+w*(original[j]-original[j-1]);tangent=(original[j]-original[j-1]).head<2>();
        }
        const double bias=parameters_.refined_curve_right_bias_m*(here?1:before?1-blend(u):blend(u));
        const lanelet::BasicPoint2d normal(tangent.y()/tangent.norm(),-tangent.x()/tangent.norm());
        const auto right=project(lane.rightBound3d().basicLineString(),point.head<2>());
        const double available=std::max(0.0,std::sqrt(right.squared_distance)-parameters_.vehicle_half_width_m-.3);
        point.head<2>()+=std::min(bias,available)*normal;result.push_back(point);
      }
    }
    return result;
  }

  Message reference(const nav_msgs::Odometry& ego,const Route& global_path) {
    const auto& route=*global_path;
    reference_spans_.clear();
    const auto& position=ego.pose.pose.position;
    const lanelet::BasicPoint2d ego_xy(position.x,position.y);
    // GlobalPath starts with the predecessor and current lane. Never match a
    // distant later checkpoint merely because its centerline crosses nearby.
    std::size_t first_lane=0;
    auto projection=project(referenceCenterline(route.front()),ego_xy);
    for(std::size_t i=1;i<std::min<std::size_t>(2,route.size());++i) {
      const auto next=project(referenceCenterline(route[i]),ego_xy);
      if(next.squared_distance<projection.squared_distance) {first_lane=i;projection=next;}
    }
    Message message;appendPoint(message,projection.point);
    double length_m=0;
    for(std::size_t i=first_lane;i<route.size() && length_m<planning_length_m_;++i) {
      const auto& line=referenceCenterline(route[i]);
      const auto& p=message.points.back().pose.pose.position;
      const auto begin=project(line,{p.x,p.y});
      const double span_begin=length_m;
      reference_end_index_=i;
      if(i+1<route.size() && adjacent(route[i],route[i+1])) {
        const auto target_line=forwardLine(route,i+1);
        const auto source_line=parallelLine(route,i+1,route[i]);
        const auto target_projection=project(target_line,begin.point.head<2>());
        Message source;appendPoint(source,begin.point);double source_length=0;
        const auto source_projection=project(source_line,begin.point.head<2>());
        appendLine(source,source_line,source_projection.segment+1,source_length);
        Message target;appendPoint(target,target_projection.point);double target_length=0;
        appendLine(target,target_line,target_projection.segment+1,target_length);
        const double offset=std::sqrt(target_projection.squared_distance);
        const double transition=std::max({parameters_.transition_length_m,
            std::sqrt(6*offset/parameters_.maximum_curvature_per_m),
            std::abs(ego.twist.twist.linear.x)*std::sqrt(6*offset/parameters_.maximum_lateral_acceleration_mps2)});
        if(source.points.size()<2 || target.points.size()<2 ||
            offset>parameters_.maximum_lane_offset_m || transition>source_length || transition>target_length) break;
        const auto source_s=distances(source),target_s=distances(target);
        for(double d=parameters_.sample_step_m;d<transition && length_m<planning_length_m_;d+=parameters_.sample_step_m) {
          const auto a=sample(source,source_s,d),b=sample(target,target_s,d);
          const double w=blend(d/transition);
          const lanelet::BasicPoint3d point(a.x+(b.x-a.x)*w,a.y+(b.y-a.y)*w,a.z+(b.z-a.z)*w);
          appendLine(message,lanelet::BasicLineString3d{point},0,length_m);
        }
        if(length_m>=planning_length_m_) break;
        const auto end=sample(target,target_s,transition);
        appendLine(message,lanelet::BasicLineString3d{{end.x,end.y,end.z}},0,length_m);
        reference_lane_change_=true;reference_handover_m_=distances(message).back();
        std::size_t target_index=i+1;
        // Use arclength, including curved target segments, to locate the handoff lane.
        Message prefix;appendPoint(prefix,target_projection.point);double prefix_length=0;
        const auto& first_target_line=referenceCenterline(route[target_index]);
        const auto first_projection=project(first_target_line,target_projection.point.head<2>());
        appendLine(prefix,first_target_line,first_projection.segment+1,prefix_length);
        double covered=prefix_length;
        while(covered<transition && target_index+1<route.size() && connected(route[target_index],route[target_index+1])) {
          ++target_index;
          const auto& next=referenceCenterline(route[target_index]);
          for(std::size_t j=1;j<next.size();++j) covered+=(next[j]-next[j-1]).head<2>().norm();
        }
        reference_end_index_=target_index;
        if(length_m>=planning_length_m_) break;
        i=target_index-1;
        continue;
      } else {
        appendLine(message,line,begin.segment+1,length_m);
      }
      reference_spans_.push_back({route[i],span_begin,length_m});
      if(i+1==route.size() || length_m>=planning_length_m_) break;
      if(adjacent(route[i],route[i+1])) continue;
      if(!follows(route[i],route[i+1])) break;
      const auto& next_line=referenceCenterline(route[i+1]);
      const auto& last=message.points.back().pose.pose.position;
      if(std::hypot(last.x-next_line.front().x(),last.y-next_line.front().y())>
          parameters_.maximum_join_gap_m) break;
    }
    if(message.points.size()>1) headings(message);
    return message;
  }


  struct Projection {
    lanelet::BasicPoint3d point;
    std::size_t segment = 0;
    double squared_distance = std::numeric_limits<double>::infinity();
  };

  static Projection project(const lanelet::BasicLineString3d& line, const lanelet::BasicPoint2d& position) {
    Projection closest;
    for (std::size_t i = 0; i + 1 < line.size(); ++i) {
      const lanelet::BasicPoint2d from = line[i].head<2>();
      const lanelet::BasicPoint2d delta = line[i + 1].head<2>() - from;
      const double squared_length = delta.squaredNorm();
      if (squared_length == 0) continue;
      const double ratio = std::max(0.0, std::min(1.0, (position - from).dot(delta) / squared_length));
      const lanelet::BasicPoint3d point = line[i] + ratio * (line[i + 1] - line[i]);
      const double distance = (point.head<2>() - position).squaredNorm();
      if (distance < closest.squared_distance) closest = {point, i, distance};
    }
    return closest;
  }

  static void appendPoint(Message& message, const lanelet::BasicPoint3d& position) {
    interfaces::PathPoint point;
    point.pose.pose.position.x = position.x();
    point.pose.pose.position.y = position.y();
    point.pose.pose.position.z = position.z();
    point.pose.pose.orientation.w = 1;
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    message.points.push_back(point);
  }

  void appendLine(Message& message, const lanelet::BasicLineString3d& line,
                  std::size_t begin, double& length_m) const {
    for (std::size_t i = begin; i < line.size() && length_m < planning_length_m_; ++i) {
      const auto& previous = message.points.back().pose.pose.position;
      const lanelet::BasicPoint3d from(previous.x, previous.y, previous.z);
      const lanelet::BasicPoint3d delta = line[i] - from;
      const double segment_m = delta.head<2>().norm();
      // Do not publish tiny seam connectors as separate steering segments.
      if (segment_m < (parameters_.reference_smoothing_enabled?.1:1e-12)) continue;
      const double take_m = std::min(segment_m, planning_length_m_ - length_m);
      appendPoint(message, from + (take_m / segment_m) * delta);
      length_m += take_m;
    }
  }


  PathPlannerMissionParameters parameters_;
  double planning_length_m_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  std::vector<LaneSpan> reference_spans_;
  std::map<std::pair<lanelet::Id,bool>,lanelet::BasicLineString3d> centerlines_;
  std::unordered_map<std::uint32_t,Track> tracks_;
  node_pipeline::Visualizer<visualization_msgs::Marker> motion_visualizer_{"object_motion"};
  Message lane_change_;
  double handover_m_=0,reference_handover_m_=0;
  bool normal_lane_change_=false,reference_lane_change_=false;
  Route continuation_route_;
  std::size_t reference_end_index_=0,continuation_index_=0;
  std::string mode_;
};
}  // namespace path_planner
namespace path_planner {
struct IntegratedHighwayApproachParameters {
  PathPlannerMissionParameters mission;
  lanelet::Id entry_target_lane_id=96937;
};
// Keep the existing exit tracking; S11 owns the highway entry gap and maneuver.
class IntegratedHighwayApproach {
 public:
static Message publishLegacyPrefix(const Message& reference,double measured_speed,
                                   double minimum_length_m,double horizon_sec=5.) {
  if(reference.points.size()<2)return reference;
  std::vector<double> station(reference.points.size(),0.);
  for(std::size_t i=1;i<reference.points.size();++i) {
    const auto& a=reference.points[i-1].pose.pose.position;
    const auto& b=reference.points[i].pose.pose.position;
    station[i]=station[i-1]+std::hypot(b.x-a.x,b.y-a.y);
  }
  double speed=std::max(0.,measured_speed),distance=0.;
  for(double time=0;time<horizon_sec-1e-9;time+=1./30) {
    const auto after=std::upper_bound(station.begin(),station.end(),distance);
    const auto i=std::min<std::size_t>(reference.points.size()-1,after-station.begin());
    const double cap=std::isfinite(reference.points[i].velocity)?reference.points[i].velocity:
        std::max(0.,measured_speed);
    const double next=std::max(0.,speed+std::max(-4./30,std::min(2./30,cap-speed)));
    distance+=(speed+next)/60.;speed=next;
  }
  const double extent=std::max(minimum_length_m,distance);
  const auto after=std::lower_bound(station.begin(),station.end(),extent);
  const std::size_t count=std::min(reference.points.size(),std::max<std::size_t>(2,after-station.begin()+1));
  Message output=reference;
  output.points.resize(count);
  for(auto& point:output.points)point.time_from_start=ros::Duration(0);
  return output;
}

  IntegratedHighwayApproach(IntegratedHighwayApproachParameters p,SharedMap& map)
      : p_(std::move(p)),mission_(p_.mission,*map.map),graph_(lanelet::routing::RoutingGraph::build(*map.map,
          *lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,lanelet::Participants::Vehicle))) {}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& competition,
      const interfaces::Objects& objects,const Route& route) {
    lanelet::ConstLanelets lanes(route->begin(),route->end());
    for(std::size_t i=0;i+1<lanes.size();++i) {
      if(lanes[i+1].id()!=p_.entry_target_lane_id)continue;
      const auto left=graph_->left(lanes[i]),right=graph_->right(lanes[i]);
      if(!(left && left->id()==p_.entry_target_lane_id) && !(right && right->id()==p_.entry_target_lane_id))continue;
      lanes.resize(i+1);std::set<lanelet::Id> visited;for(const auto& lane:lanes)visited.insert(lane.id());
      double length=0;
      for(int step=0;step<20 && length<p_.mission.local_path_length_m+10;++step) {
        const auto next=graph_->following(lanes.back(),false);if(next.empty())break;
        const auto tail=lanes.back().centerline3d().basicLineString();const auto direction=(tail.back()-tail[tail.size()-2]).head<2>().normalized().eval();
        auto chosen=next.front();double best=-2;
        for(const auto& candidate:next) {
          const auto line=candidate.centerline3d().basicLineString();const double aligned=direction.dot((line[1]-line[0]).head<2>().normalized());
          if(aligned>best){best=aligned;chosen=candidate;}
        }
        if(!visited.insert(chosen.id()).second)break;
        const auto line=chosen.centerline3d().basicLineString();for(std::size_t j=1;j<line.size();++j)length+=(line[j]-line[j-1]).head<2>().norm();
        lanes.push_back(chosen);
      }
      break;
    }
    const Route approach=lanelet::routing::LaneletPath(std::move(lanes));
    return publishLegacyPrefix(mission_.process(ego,competition,objects,approach),
        std::max(0.,double(competition.velocity_x_mps)),47.);
  }
 private:
  IntegratedHighwayApproachParameters p_;
  PathPlannerMission mission_;
  lanelet::routing::RoutingGraphPtr graph_;
};
}
}  // namespace integrated_highway_approach_detail

namespace path_planner {
using IntegratedHighwayApproach = ::integrated_highway_approach_detail::path_planner::IntegratedHighwayApproach;
}  // namespace path_planner
