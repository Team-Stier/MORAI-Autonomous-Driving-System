#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <ros/console.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <set>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <lanelet2_core/LaneletMap.h>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <node_pipeline/extension.hpp>
#include <visualization_msgs/Marker.h>
#include <array>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <utility>
#include <unsupported/Eigen/Splines>
#include <cstddef>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <initializer_list>
#include <iomanip>
#include <sstream>
#include <memory>
#include <tuple>
#include <sensor_msgs/Imu.h>

namespace integrated_dynamic_detail {
using ::path_planner::Message;
using ::path_planner::Route;
using ::path_planner::PredictedObjects;
using ::path_planner::Checkpoint;
using ::path_planner::Checkpoints;
using ::path_planner::SharedMapParameters;
using ::path_planner::SharedMap;

// ---- inlined from libraries/integrated_dynamic_mission.hpp ----
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
// ---- inlined from libraries/integrated_solver.hpp ----
// Imported numerical planner: Team-Stier/MORAI-Autonomous-Driving-System
// path_planner_MPC @ 336f7c74d5923b7ebec100d44bc57d0df39bcc4b
namespace integrated_types {

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

namespace integrated_types {

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
  double wheelbase_m = 3.0;
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
  std::shared_ptr<const std::vector<Pose2>> spatial_reference{};
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
  double lane_change_confirmation_sec = 1.0;
  double lane_change_observation_gap_sec = 0.2;
  double lane_change_heading_tolerance_rad = 0.03490658503988659;
  double lane_change_steering_tolerance_rad = 0.05235987755982989;
  double horizon_m = 25.0;
  double preferred_reference_piece_m = 0;
  double min_horizon_m = 0.5;  // Minimum geometric calculation extent, not stopping distance.
  double road_boundary_step_m = 0.25;  // Polygon discretization; every reference knot is retained.
  double dp_step_m = 5.0;
  double lateral_step_m = 0.5;
  double dp_slope_sample = 0.2;
  double output_step_m = 0.25;
  double corridor_half_width_m = 0.3;
  double obstacle_margin_m = 0.25;
  double road_margin_m = 0.05;
  std::vector<std::vector<Vec2>> physical_boundaries;
  double reentry_response_sec = 0;
  // A body stopped partly off the road keeps the response sweep of leaving
  // at this speed; a measured faster departure uses its own speed.
  double reentry_minimum_speed_mps = 0;
  double pedestrian_caution_speed_mps = 0.0;  // 0 disables the optional VRU approach ceiling.
  double pedestrian_caution_distance_m = 0.0;
  double pedestrian_caution_lateral_margin_m = 0.0;  // 0 keeps the legacy body/bbox preview.
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
  double static_forward_shape_reserve_m = 0.0;
  bool recover_static_obstacle_margin = false;
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
  // Retiming checks both a geometric proposal and its conditioned variants.
  // Keep room for those variants when objects require interaction searches.
  unsigned maximum_center_proposals=8,maximum_frenet_proposals=4,maximum_speed_searches=4;
  std::size_t recovery_speed_beam_width=64;
  double speed_tracking_weight = 1.0;
  double acceleration_weight = 0.15;
  double longitudinal_jerk_weight = 0.05;
  double following_time_headway_sec = 1.5;
  double following_low_speed_headway_sec = 0.0;  // 0 preserves the legacy target.
  double following_headway_transition_mps = 5.0;
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

namespace integrated_native {
using ::path_planner::Message;
using ::path_planner::Route;
using ::path_planner::PredictedObjects;
using integrated_types::Vec2;
using integrated_types::Pose2;
using integrated_types::ObjectKind;
using integrated_types::MotionModel;
using integrated_types::PredictedObject;
using integrated_types::Predictions;
using integrated_types::ReferencePoint;
using integrated_types::EgoState;
using integrated_types::VehicleShape;
using integrated_types::ProgressTime;
using integrated_types::PathPoint;
using integrated_types::PlanningConstraints;
using integrated_types::PathParameters;
using integrated_types::kPredictionSteps;

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
// Adapter consumes detector /objects only. Ground truth is never a planning input.
class IntegratedSolver {
 private:
  // #WITHIN_CALL_GEOMETRY_REUSE
  // 축소 beam과 전체 beam은 같은 관측·예측·이전 경로와 도로에서 형상을 만든다.
  // 같은 시간표·공간 창·계산 모드의 DP/QP와 같은 C2 목표를 다시 계산하지 않는다.
  // 이 캐시는 아래 plan() 호출의 지역 변수로만 소유해 다음 관측으로 넘기지 않는다.
  // root reference와 초기 자세·곡률은 두 beam에서 같고, 다른 출발 차선 참조는
  // 이 캐시에 넣지 않는다. 시간표·horizon·기하/재시간화 모드·C2 복구 여부는
  // 정확한 값으로 구분한다. 실패도 동일 입력의 계산 결과로만 재사용한다.
  // 속도·가감속·객체 충돌·도로·최종 궤적 검사는 각 탐색에서 그대로 수행한다.
  using GeometrySchedule=std::vector<std::pair<double,double>>;
  struct PlanGeometryCache {
    std::map<std::tuple<GeometrySchedule,bool,double,bool>,std::vector<PathPoint>> generated;
    std::map<std::pair<GeometrySchedule,bool>,std::vector<PathPoint>> cartesian;
    const std::vector<Vec2>* independent_entries=nullptr;
  };
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
    std::shared_ptr<const NativeRoadRegion> original_lanelets;
    std::shared_ptr<const NativeRoadRegion> recovery_original;
    double reentry_outside_limit_m2=0;
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
      // Repeated polygon overlays can leave sub-millimetre sliver holes at
      // shared lanelet seams. Remove only numerical hairlines below 1 cm^2;
      // physical paint, outer boundaries and finite road holes stay intact.
      for(auto& polygon:polygons) {
        auto& holes=polygon.inners();
        holes.erase(std::remove_if(holes.begin(),holes.end(),[](const RoadPolygon::ring_type& hole) {
          const double area=std::abs(boost::geometry::area(hole));double perimeter=0;
          for(std::size_t i=1;i<hole.size();++i)perimeter+=std::hypot(hole[i].x()-hole[i-1].x(),hole[i].y()-hole[i-1].y());
          return area<=1e-4 && perimeter>0 && area/perimeter<=.0005;
        }),holes.end());
      }
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
      // Overlay roundoff must not remove any footprint already contained
      // in the original, permitted Lanelet union (A is a subset of A union B).
      if(original_lanelets && original_lanelets->containsBox(box))return true;
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
  std::shared_ptr<const NativeRoadRegion> nativeRoadRegion(const Route& route,bool include_neighbors=true) {
    if(!routing_graph_) return nullptr;
    std::map<LaneKey,lanelet::ConstLanelet> lanes;
    for(const auto& lane:*route) {
      lanes.emplace(laneKey(lane),lane);
      for(bool left:{false,true}) {
        const auto neighbor=include_neighbors ? corridorNeighbor(lane,left) : lanelet::Optional<lanelet::ConstLanelet>{};
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
  explicit IntegratedSolver(PathParameters parameters = {}) : p_(parameters) { validateParameters(); preparePhysicalBoundaries(); }
  IntegratedSolver(const IntegratedSolver& source,PathParameters parameters) : IntegratedSolver(source) {
    p_=parameters;validateParameters();preparePhysicalBoundaries();
  }
  IntegratedSolver(PathParameters parameters, const lanelet::LaneletMap& map) : IntegratedSolver(parameters) {
    const auto rules=lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany,lanelet::Participants::Vehicle);
    routing_graph_=lanelet::routing::RoutingGraph::build(map,*rules);
  }


  // Planned v/a are ground speed and tangential acceleration along the actual
  // rear-axle path. Reference progress is only for correspondence with the map.
  // Every result spans the available prediction interval; blocked motion is
  // represented by a physically reachable stop/wait, never a truncated prefix.
  std::vector<PathPoint> plan(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                                const Predictions& predictions, const PlanningConstraints& requested_limits,
                                const std::vector<PathPoint>& previous = {},bool pending_lane_change=false,
                                const std::vector<ReferencePoint>* hold_reference=nullptr,
                                const std::vector<Vec2>* independent_entries=nullptr) const {
    // #COLLISION_SEARCH_BUDGET
    // map 객체가 있으면 시간별 swept SAT가 많은 속도 후보에서 반복된다.
    // 동일한 전체 기하/속도 탐색을 작은 beam으로 먼저 수행하고, 모든 도로·
    // 충돌·속도·횡가속도·가감속·jerk 검사를 통과한 진행 경로만 채택한다.
    // 실제 상태 가지치기가 있었고 해가 없거나 종료 정지이면 원래 beam으로
    // 재탐색한다. 가지치기 전 실패는 큰 beam에서도 같으므로 반복하지 않는다.
    // 각 실패 기하마다 큰 탐색을 반복하지 않으며, 객체가 없는 곡선은 원래
    // beam을 유지한다. GT ID·센서·경로 시각·제어 입력 만료는 바꾸지 않는다.
    selected_extent_=0;selected_curvature_=0;selected_source_="none";geometry_reason_="none";road_reason_="none";failure_reason_="geometry";speed_searches_=0;geometry_searches_=0;validated_stop_.clear();rejected_.fill(0);path_check_reason_="none";failed_layer_=-1;extended_search_=false;
    initial_obstacle_margins_.clear();
    if(p_.recover_static_obstacle_margin) {
      const auto own=Geometry::egoBox(ego.pose,p_.vehicle);
      const auto forward=Geometry::unit(ego.pose.yaw_rad);
      const double offset=ego.stamp_sec-predictions.reference_stamp_sec;
      if(offset>=0 && offset<=predictions.step_sec*kPredictionSteps)
      for(std::size_t j=0;j<predictions.current.size();++j) {
        const auto object=samplePrediction(predictions,j,offset);
        bool stationary=Geometry::norm(object.velocity_mps)<1e-10;
        for(const auto& slice:predictions.future)
          stationary=stationary && Geometry::norm(Geometry::sub(slice[j].pose.position,object.pose.position))<1e-8;
        if(!stationary)continue;
        const auto other=Geometry::objectBox(object,0);
        const auto direction=Geometry::unit(other.pose.yaw_rad),normal=Geometry::normal(other.pose.yaw_rad);
        const auto delta=Geometry::sub(other.pose.position,own.pose.position);
        double margin=-std::numeric_limits<double>::infinity();
        for(const auto& axis:{forward,Geometry::normal(ego.pose.yaw_rad),direction,normal}) {
          const double gap=std::abs(Geometry::dot(delta,axis))-Geometry::extent(own,axis)-Geometry::extent(other,axis);
          const double coefficient=std::abs(Geometry::dot(direction,axis))+std::abs(Geometry::dot(normal,axis));
          margin=std::max(margin,gap/coefficient);
        }
        // Positive SAT separation proves the physical observed OBBs are apart.
        // Preserve that measured gap until the rear axle has passed this box;
        // do not require a larger buffer inside an already narrow passage.
        if(margin>1e-5 && margin<p_.obstacle_margin_m) {
          double clearance=0;
          for(const auto& corner:Geometry::corners(other))
            clearance=std::max(clearance,Geometry::dot(Geometry::sub(corner,ego.pose.position),forward)+p_.vehicle.rear_m);
          // Restore only after clearing the fully buffered far face as well.
          clearance+=std::sqrt(2.)*p_.obstacle_margin_m;
          initial_obstacle_margins_[object.id]={std::max(0.,margin-std::max(1e-6,4*p_.qp_tolerance)),clearance};
        }
      }
    }
    constexpr std::size_t fast_beam_width=32;
    PlanGeometryCache geometry_cache;geometry_cache.independent_entries=independent_entries;
    bool truncated=false;
    // #HIGHWAY_SEARCH_ALLOCATION
    // 고속 변경에서 축소 beam의 후보 비교 뒤 전체 beam을 재계산하면
    // 제어 입력 만료가 생길 수 있다. 실측 v/a의 기존 jerk 제한 정지거리가
    // 기존 긴 연결 창(2*horizon_m)보다 큰 고속 인접 변경에만 원래 전체
    // beam을 처음부터 한 번 배정한다. 짧은 창 안에서 정지 가능한 장면과
    // 일반도로는 원래 빠른 탐색·반환 순서를 유지한다. 이는 계산 배정 기준이며
    // 정지거리·물리 한도나 안전 보증을 새로 정의하지 않는다.
    // 후보·최대 beam·예측 시간을 늘리지 않으며, 같은 map 차체/도로/객체와
    // 속도·가감속·jerk·최종 궤적 검사를 모두 거친 결과만 반환한다.
    // 이전 경로/시각/입력 만료/MPC는 바꾸지 않는다. LiDAR 통합도 GT와 같은
    // map bbox·절대 속도·예측 시각 계약 및 최종 검사를 유지해야 한다.
    const bool highway_lane_change=pending_lane_change &&
        (requested_limits.maximum_speed_mps>p_.normal_speed_limit_mps ||
         ego.speed_mps>p_.normal_speed_limit_mps);
    const bool long_maneuver_search=highway_lane_change &&
        brakingDistance(ego.speed_mps,ego.acceleration_mps2)>2*p_.horizon_m;
    if(!long_maneuver_search && p_.static_forward_shape_reserve_m<=0 && !predictions.current.empty() && p_.speed_beam_width>fast_beam_width &&
       requested_limits.maximum_speed_mps>0 && ego.acceleration_mps2>=0) {
      auto quick=planWithBeam(ego,reference_map,predictions,requested_limits,previous,fast_beam_width,truncated,pending_lane_change,hold_reference,&geometry_cache);
      // #OBSTACLE_APPROACH_SEARCH
      // 가지치기한 탐색이 즉시 제동을 택했다면 이동 경로가 있다는 사실만으로
      // 채택하지 않는다. 기존 전체 beam으로 같은 객체·도로·동역학을 다시
      // 계산해 탐색 누락에 따른 불필요한 감속을 줄인다. 가지치기가 없거나
      // 정상 가속/순항이면 원래 빠른 경로를 유지하며 안전 검사는 생략하지 않는다.
      // 이미 감속 중이면 처음부터 전체 beam을 써 이중 계산을 피한다.
      const bool early_braking=std::any_of(quick.begin(),quick.end(),[&](const PathPoint& point) {
        return point.time_sec<=p_.speed_time_step_sec && point.acceleration_mps2<0;
      });
      if(!quick.empty() && quick.back().speed_mps>.5 && !early_braking) return quick;
    }
    auto result=planWithBeam(ego,reference_map,predictions,requested_limits,previous,p_.speed_beam_width,truncated,pending_lane_change,hold_reference,&geometry_cache);
    const bool free_window_stop=!result.empty() && result.back().speed_mps<.5 &&
        requested_limits.maximum_speed_mps>0 &&
        !std::isfinite(requested_limits.stop_line_progress_m);
    if(!result.empty() && !free_window_stop)return result;
    // A finite first pass is a calculation budget, not a blocked-road verdict.
    // Reuse its geometric proposals and widen only the speed search when no
    // moving solution was found. Every result is checked by the same rules.
    const bool retry=p_.recovery_speed_beam_width>p_.speed_beam_width &&
        std::string(failure_reason_)!="initial-collision" &&
        std::string(failure_reason_)!="initial-curvature" &&
        std::string(failure_reason_)!="initial-projection" &&
        std::string(failure_reason_)!="initial-longitudinal" &&
        std::string(failure_reason_)!="forecast-time";
    if(retry) {
      const auto retained=result;
      speed_searches_=0;geometry_searches_=0;extended_search_=true;
      result=planWithBeam(ego,reference_map,predictions,requested_limits,previous,
          p_.recovery_speed_beam_width,truncated,pending_lane_change,hold_reference,&geometry_cache);
      if(!retained.empty() && (result.empty() ||
         retained.back().progress_m>result.back().progress_m))result=retained;
    }
    return result.empty()?validated_stop_:result;
  }

  std::vector<PathPoint> planWithBeam(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                                const Predictions& predictions, const PlanningConstraints& requested_limits,
                                const std::vector<PathPoint>& previous,std::size_t beam_width,bool& truncated,
                                bool pending_lane_change,const std::vector<ReferencePoint>* hold_reference=nullptr,
                                PlanGeometryCache* geometry_cache=nullptr) const {
    auto limits=requested_limits;
    requireSceneDomain(ego,reference_map,predictions);
    requirePlanningDomain(ego,limits);
    // This independent calculation-window budget applies only to integrated
    // search. Do not invalidate finer legacy supplied-schedule configurations.
    if(p_.trajectory_horizon_m/p_.output_step_m>600 || p_.trajectory_horizon_m/p_.dp_step_m>100)
      throw std::invalid_argument("unbounded integrated planner discretization");
    if(!std::isfinite(brakingDistance(ego.speed_mps,ego.acceleration_mps2))) {
      failure_reason_="initial-longitudinal";return {};
    }
    const double offset=ego.stamp_sec-predictions.reference_stamp_sec;
    const double duration=predictions.step_sec*kPredictionSteps-offset;
    if (offset<0 || duration<=0){failure_reason_="forecast-time";return {};}
    // 절대 차량 곡률 한도는 유지한다. 실측 횡가속도 초과는 아래의
    // 고정된 가장 빠른 감속 포락선 안에서만 복귀하도록 시간 계획한다.
    if(std::max(0.0,std::abs(ego.curvature_inv_m)-1e-8)>
       p_.max_curvature_inv_m+1e-6+1e-12){failure_reason_="initial-curvature";return {};}
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
    std::map<std::uint32_t,double> active_pedestrian_caution;
    const double approach_distance=p_.pedestrian_caution_distance_m>0 ? approachDistance(ego) : 0;
    if(p_.pedestrian_caution_speed_mps>0 && approach_distance>0) {
      for(std::size_t index=0;index<predictions.current.size();++index) {
        if(predictions.current[index].kind!=ObjectKind::Pedestrian) continue;
        const auto object=samplePrediction(predictions,index,offset);
        const double radius=std::hypot(object.length_m,object.width_m)/2+object.radius_error_m;
        const auto delta=Geometry::sub(object.pose.position,ego.pose.position);
        const auto remembered=pedestrian_caution_distances_.find(object.id);
        const double preview=remembered==pedestrian_caution_distances_.end() ? approach_distance :
            std::max(approach_distance,remembered->second);
        if(Geometry::norm(delta)>preview+radius) continue;
        const auto spot=reference.project(object.pose.position,origin.s,
            std::min(reference.length(),origin.s+preview));
        const auto road=reference.at(spot.s);
        // #PEDESTRIAN_ROADSIDE_PREVIEW
        // 도로 옆 보행자의 접근 판정은 충돌 OBB와 별도로 설정한 도로 밖
        // 중심점 거리로 결정한다. 차체 반폭과 보행자 bbox를 합친 경계를
        // 사용하면 정지 보행자의 애니메이션 폭이 수 mm 변할 때 최초 감속이
        // 늦어질 수 있다. 입력은 같은 map 관측이며 종방향 접근 거리·뒤쪽
        // 제외·추적 ID 유지 순서는 그대로다. 실제 충돌 검사에는 원래 bbox,
        // 여유·속도·예측을 사용한다. 0 설정은 기존 판정을 보존한다.
        // LiDAR 통합 시에도 위치·분류·추적 ID를 같은 계약으로 공급해야 한다.
        const double padding=p_.pedestrian_caution_lateral_margin_m>0 ?
            p_.pedestrian_caution_lateral_margin_m : radius+p_.vehicle.width_m/2+p_.obstacle_margin_m;
        if(Geometry::dot(delta,Geometry::unit(ego.pose.yaw_rad))<-p_.vehicle.rear_m-radius) continue;
        if(remembered==pedestrian_caution_distances_.end() &&
           (spot.distance>std::max(std::abs(road.lower),std::abs(road.upper))+padding ||
            spot.l<road.lower-padding || spot.l>road.upper+padding)) continue;
        active_pedestrian_caution.emplace(object.id,preview);
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
    pedestrian_caution_distances_=std::move(active_pedestrian_caution);
    // #SPEED_DEPENDENT_CALCULATION_WINDOW
    // 일반 구간에서는 검증한 기존 네 기하 창(기본 100/75/50/37.5m)을 유지한다.
    // 고주로 목표 또는 이미 빠른 실측 상태일 때만 확장한 공간 계산 범위를 쓴다.
    // 같은 0.25m 표본과 전체 도로·충돌·시간 검사를 유지하며 일반 곡선/정적
    // 회피의 유효 후보를 새로운 고속 계산 범위의 비율로 바꾸지 않는다.
    const double window_cap=std::max(ego.speed_mps,std::min(p_.max_speed_mps,limits.maximum_speed_mps));
    const double accelerate=std::min(duration,std::max(0.,(window_cap-ego.speed_mps)/p_.max_acceleration_mps2));
    const double reachable=ego.speed_mps*accelerate+.5*p_.max_acceleration_mps2*accelerate*accelerate+
        window_cap*(duration-accelerate);
    const double planning_horizon_m=std::min(p_.trajectory_horizon_m,std::max(2*p_.horizon_m,
        reachable+brakingDistance(window_cap,0)+p_.vehicle.front_m+p_.vehicle.rear_m+2));
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
    const auto original_geometry=roadGeometry(reference_map,reference);
    const NativeRegionScope reentry_scope(active_native_region_,roadReentryRegion(ego,original_geometry));
    const auto geometry=roadGeometry(reference_map,reference);
    const auto& boundary=geometry->boundary;
    const std::vector<ProgressTime> unused;
    const Context c{reference,predictions,unused,boundary,0,offset,false,&geometry->index,geometry->native_region.get()};
    // Admission belongs to the scene, not to any single geometric proposal.
    // In particular, a hold-offset candidate must not bypass these guards.
    // A vehicle already traversing a legal avoidance lane can exceed the
    // nominal Frenet heading threshold. Admit a forward Cartesian connection
    // only while its measured full body is inside the existing permitted road.
    // The connection still has to recover lateral slope and pass every sweep.
    const double measured_heading=Geometry::wrap(ego.pose.yaw_rad-initial_road.yaw);
    const bool road_heading=c.native_region && roadClear(c,ego.pose,ego.pose) &&
        std::abs(measured_heading)<std::min(Geometry::kPi/2,
            p_.max_initial_heading_error_rad+std::atan(p_.max_lateral_slope));
    projection_distance_=origin.distance;projection_heading_=measured_heading;
    if(origin.distance>p_.max_initial_distance_m ||
       (!nominal_heading && !recovery_heading && !road_heading) ||
       reference.ambiguous(ego.pose.position,origin,ego.pose.yaw_rad)){failure_reason_="initial-projection";return {};}
    const auto schedule_key=[](const std::vector<ProgressTime>& timing) {
      GeometrySchedule key;for(const auto& point:timing)key.emplace_back(point.progress_m,point.time_sec);return key;
    };
    const auto generate_geometry=[&](const std::vector<ProgressTime>& timing,bool geometric_only,
                                     double horizon,bool retime=false) {
      const auto key=std::make_tuple(schedule_key(timing),geometric_only,horizon,retime);
      if(geometry_cache) {
        const auto found=geometry_cache->generated.find(key);
        if(found!=geometry_cache->generated.end())return found->second;
      }
      if(++geometry_searches_>p_.maximum_frenet_proposals*(extended_search_?3:1) && p_.maximum_frenet_proposals){++rejected_[0];return std::vector<PathPoint>{};}
      auto result=generatePath(ego,reference_map,predictions,timing,previous,geometric_only,horizon,retime);
      if(geometry_cache)geometry_cache->generated.emplace(key,result);
      return result;
    };
    const auto center_geometry=[&](const GeometrySchedule& goals,bool repair=false) {
      const auto key=std::make_pair(goals,repair);
      if(geometry_cache) {
        const auto found=geometry_cache->cartesian.find(key);
        if(found!=geometry_cache->cartesian.end())return found->second;
      }
      const bool footprint_goals=p_.preferred_reference_piece_m>0 && !pending_lane_change && !lane_change_ &&
          std::all_of(goals.begin(),goals.end(),[](const std::pair<double,double>& goal){return goal.second==0;});
      auto seed=ego;
      const bool tracking_target=p_.preferred_reference_piece_m>0 && !pending_lane_change && !lane_change_ &&
          (!c.native_region || !c.native_region->recovery_original);
      if(tracking_target) {
        double l=0,dl=0,ddl=0;
        if(!predictions.current.empty() && !previous.empty() && previous.front().spatial_reference) {
          l=previousL(reference,origin.s,previous);
          const double before=previousL(reference,std::max(0.,origin.s-.5),previous);
          const double after=previousL(reference,std::min(reference.length(),origin.s+.5),previous);
          dl=after-before;ddl=(after-2*l+before)/.25;
        }
        const auto point=lateralPoint(reference,origin.s,0,l,dl,ddl);
        seed.pose=point.pose;seed.curvature_inv_m=point.curvature_inv_m;
        PathPoint initial;initial.pose=seed.pose;
        if(p_.static_forward_shape_reserve_m>0 && collision(c,initial,initial))seed=ego;
      }
      auto result=cartesianPieces(c,seed,goals,repair,footprint_goals);
      if(tracking_target && !result.empty()) {
        auto target=std::make_shared<std::vector<Pose2>>();target->reserve(result.size());
        for(const auto& point:result)target->push_back(point.pose);
        result=matchInitialCurve(c,ego,result);
        if(!result.empty())result.front().spatial_reference=std::move(target);
      }
      if(geometry_cache)geometry_cache->cartesian.emplace(key,result);
      return result;
    };
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
    const bool align_spatial_curvature=pending_lane_change && c.native_region &&
        !c.native_region->recovery_original;
    const auto timed_connection=[&](const std::vector<PathPoint>& connected) -> std::vector<PathPoint> {
      if(connected.empty())return {};
      ArcPath connected_arc(connected);
      // #CONNECTED_CURVATURE_SEARCH
      // map 초기 곡선을 QP로 보정하면 명시 곡률 외에 공간 보간 곡률도 생긴다.
      // 차선 변경과 같은 기존 공간 곡률 계산을 보정 후보의 속도 탐색에도
      // 적용해, 최종 검사에서 같은 이유로 탈락할 속도 후보를 반복하지 않는다.
      // output_step_m과 실제 뒷차축 형상·v/a·객체 시각은 그대로 사용한다.
      // 정상 원본 C2의 선택과 도로 복귀 탐색은 유지하며, 보정 후에도 기존
      // 전체 도로/ROI·차체·예측 충돌·곡률·가감속·jerk 검사로 출력한다.
      if(c.native_region && !c.native_region->recovery_original)
        connected_arc.includeSpatialCurvature(p_.output_step_m);
      const double connected_stop=stopArc(ego,reference,connected_arc,limits);
      if(connected_stop<0 && limits.release_time_sec>0)return {};
      auto candidate=planSpeed(c,connected_arc,ego,limits,duration,connected_stop,reference_map,beam_width,truncated);
      if(!candidate.empty() && candidate.back().speed_mps>0 &&
         checkPlannedPath(ego,reference_map,predictions,limits,candidate))return candidate;
      return {};
    };
    // Search the actual reference/obstacle corridor before accepting a C2
    // shortcut. A safe braking prefix is not a failed search, but must not
    // hide a feasible DP/QP route around the same stationary obstacle.
    if(p_.preferred_reference_piece_m>0 && !pending_lane_change && !lane_change_ &&
       (!c.native_region || !c.native_region->recovery_original)) {
      bool stationary=true;
      for(std::size_t j=0;j<predictions.current.size();++j) {
        const auto& object=predictions.current[j];
        stationary=stationary && Geometry::norm(object.velocity_mps)<1e-10;
        for(const auto& slice:predictions.future)
          stationary=stationary && Geometry::norm(Geometry::sub(slice[j].pose.position,object.pose.position))<1e-10;
      }
      if(stationary) {
        const double horizon=std::min(planning_horizon_m,
            std::max(2*p_.horizon_m,ego.speed_mps*duration+.5*p_.max_acceleration_mps2*duration*duration+p_.dp_step_m));
        // Search a tracking target, rather than moving that target onto the
        // measured tracking error on every cycle. Timing is connected from
        // the actual rear axle below. Static avoidance keeps the last searched
        // target at this map station as its DP/QP initial condition.
        auto search_ego=ego;
        double l=0,dl=0,ddl=0;
        if(!predictions.current.empty() && !previous.empty() && previous.front().spatial_reference) {
          l=previousL(reference,origin.s,previous);
          const double before=previousL(reference,std::max(0.,origin.s-.5),previous);
          const double after=previousL(reference,std::min(reference.length(),origin.s+.5),previous);
          dl=(after-before);ddl=(after-2*l+before)/.25;
        }
        auto tracking=lateralPoint(reference,origin.s,0,l,dl,ddl);
        // A sharp turn may require a small rear-axle offset for the asymmetric
        // front footprint. Represent the nearest legal tracking seed instead
        // of rejecting the entire reference search at its first point.
        if(predictions.current.empty() && !roadClear(c,tracking.pose,tracking.pose)) {
          const auto r=reference.at(origin.s);
          const auto intervals=roadLateralIntervals(c,r,r.yaw,
              r.lower+p_.vehicle.width_m/2+p_.road_margin_m,
              r.upper-p_.vehicle.width_m/2-p_.road_margin_m);
          double distance=std::numeric_limits<double>::infinity();
          for(const auto& interval:intervals) {
            const double offset=Geometry::clamp(l,interval.first,interval.second);
            if(std::abs(offset-l)<distance) {
              distance=std::abs(offset-l);tracking=lateralPoint(reference,origin.s,0,offset,0,0);
            }
          }
        }
        search_ego.pose=tracking.pose;search_ego.curvature_inv_m=tracking.curvature_inv_m;
        // An updated LiDAR face can invalidate the previous optimizer seed.
        // Start the new search at the measured feasible state in that case;
        // do not repeatedly optimize from a now occupied historical target.
        PathPoint initial;initial.pose=search_ego.pose;
        if(p_.static_forward_shape_reserve_m>0 && collision(c,initial,initial))search_ego=ego;
        auto search_predictions=predictions;
        if(p_.static_forward_shape_reserve_m>0)for(std::size_t j=0;j<search_predictions.current.size();++j) {
          auto& object=search_predictions.current[j];
          const auto projection=reference.project(object.pose.position,origin.s,reference.length());
          const auto road=reference.at(projection.s);
          const auto forward=Geometry::unit(road.yaw),normal=Geometry::normal(road.yaw);
          double lo=std::numeric_limits<double>::infinity(),hi=-lo,left=lo,right=-lo;
          for(const auto& corner:Geometry::corners(Geometry::objectBox(object,0))) {
            const auto delta=Geometry::sub(corner,road.position);
            const double x=Geometry::dot(delta,forward),y=Geometry::dot(delta,normal);
            lo=std::min(lo,x);hi=std::max(hi,x);left=std::min(left,y);right=std::max(right,y);
          }
          hi+=p_.static_forward_shape_reserve_m;
          object.pose={Geometry::add(road.position,Geometry::add(Geometry::mul(forward,(lo+hi)/2),
              Geometry::mul(normal,(left+right)/2))),road.yaw};
          object.length_m=hi-lo;object.width_m=right-left;
          for(auto& slice:search_predictions.future)slice[j]=object;
        }
        // Stationary shapes do not consume a motion-forecast distance.
        // Parameterize the whole geometry in the available interval; this
        // proposal timing is discarded by planSpeed's measured-state solve.
        // Collision checks remain active at every geometric support.
        auto shape=generatePath(search_ego,reference_map,search_predictions,
            {{0,0},{horizon,duration}},previous,predictions.current.empty(),horizon,true);
        if(!shape.empty()) {
          auto connected=matchInitialCurve(c,ego,shape);
          if(!connected.empty()) {
            auto lateral=std::make_shared<std::vector<Pose2>>();lateral->reserve(shape.size());
            for(const auto& point:shape)lateral->push_back(point.pose);
            connected.front().spatial_reference=std::move(lateral);
          }
          auto candidate=timed_connection(connected);
          if(!candidate.empty() && candidate.back().speed_mps>.5) {
            selected_source_="native-frenet";
            if(p_.preferred_reference_piece_m>0) {
              // Keep the searched avoidance target as the lateral reference;
              // matching current steering into it is the timed connection,
              // not a reason to delay the avoidance target on every cycle.
              auto lateral=std::make_shared<std::vector<Pose2>>();lateral->reserve(shape.size());
              for(const auto& point:shape)lateral->push_back(point.pose);
              candidate.front().spatial_reference=std::move(lateral);
            }
            return candidate;
          }
        }
      }
    }
    // #CURVE_FOOTPRINT_RESERVE
    // 정상 곡선에서 뒷차축 경로가 차체 전체에 최소 여유만 남기면 수 cm의
    // 추종 오차에도 다음 재계획의 기하/횡가속도 연결이 막힐 수 있다.
    // 현재 차체가 충분히 안쪽에 있을 때만, 앞으로 두 속도 계획 간격의
    // 출력 자세에 road_margin_m의 두 배 여유가 있는 경로를 선호한다.
    // 이 값은 새 필수 안전 한도가 아니라 경계 접근을 미리 피하는 선택 기준이다.
    // 여유가 부족한 정상 이동 경로에 기존 15m 차체 중심 QP를 한 번 적용하고
    // 같은 map 뒷차축·실측 v/a/곡률에서 속도를 새로 계산한다. 새 경로가
    // 같은 여유 기준과 원본 객체·전체 도로·충돌·곡률·횡가속도·jerk 검사를
    // 모두 통과할 때만 선택한다. 실패하면 기존 검증 경로를 그대로 쓴다.
    // 실제 차선 변경·정적 회피·도로 복귀 선택과 MPC는 유지한다. QP 반복과
    // 원래 최소 여유를 바꾸지 않으며 경계 밖으로 도로를 확장하지 않는다.
    // GT ID·지도 구간에 의존하지 않고 LiDAR/ROI도 같은 map bbox·절대 속도·
    // 예측 시각 및 허용 도로 계약을 전달해야 한다.
    const auto prefer_body_reserve=[&](std::vector<PathPoint> path) -> std::vector<PathPoint> {
      if(path.empty() || path.back().speed_mps<=0 || pending_lane_change || lane_change_ || static_center_blocked ||
         !c.native_region || c.native_region->recovery_original ||
         !roadClearWithMargin(c,ego.pose,ego.pose,2*p_.road_margin_m))return path;
      const auto has_reserve=[&](const std::vector<PathPoint>& candidate) {
        return std::all_of(candidate.begin(),candidate.end(),[&](const PathPoint& point) {
          return point.time_sec>2*p_.speed_time_step_sec ||
                 roadClearWithMargin(c,point.pose,point.pose,2*p_.road_margin_m);
        });
      };
      if(has_reserve(path))return path;
      const auto settled=conditionInitialGeometryWithin(c,ego,path,15.0);
      auto preferred=timed_connection(settled);
      if(!preferred.empty() && has_reserve(preferred)) {
        return preferred;
      }
      return path;
    };
    unsigned center_attempts=0;
    const auto center_candidate=[&](double horizon,double segment_extent) -> std::vector<PathPoint> {
      if(++center_attempts>p_.maximum_center_proposals*(extended_search_?2:1) && p_.maximum_center_proposals){++rejected_[0];return {};}
      GeometrySchedule goals;
      if(segment_extent==p_.preferred_reference_piece_m && !pending_lane_change && !lane_change_) {
        // Keep curve joints in map coordinates across calls. Re-dividing the
        // whole horizon at the moving rear axle changes every downstream
        // quintic even when the road and obstacles have not changed.
        auto anchor=reference.project(curve_knot_anchor_,0,reference.length());
        double knot=anchor.s;
        if(!curve_knot_initialized_ || ego.stamp_sec<curve_knot_stamp_ ||
           ego.stamp_sec-curve_knot_stamp_>1 || anchor.distance>.05 ||
           std::abs(knot-origin.s)>3*segment_extent)
          knot=origin.s+segment_extent;
        while(knot<origin.s+segment_extent-1e-6)knot+=segment_extent;
        curve_knot_anchor_=reference.at(knot).position;
        curve_knot_stamp_=ego.stamp_sec;curve_knot_initialized_=true;
        double station=origin.s;
        while(knot<origin.s+horizon-1e-6) {
          goals.emplace_back(knot-station,0);station=knot;knot+=segment_extent;
        }
        if(origin.s+horizon-station>1e-6)goals.emplace_back(origin.s+horizon-station,0);
      } else {
        const int segments=static_cast<int>(std::ceil(horizon/segment_extent));
        goals.assign(segments,{horizon/segments,0});
      }
      auto smooth=center_geometry(goals);
      // 새 곡선이 시간 계획에서 실패해도 기존 짧은 창과 DP 후보를 계속
      // 검사해야 한다. 탐색 순서/정적 차단 표시는 원래 기하에서만 갱신한다.
      const bool original_geometry=!smooth.empty();
      if(!original_geometry) smooth=center_geometry(goals,true);
      if(smooth.empty()){++rejected_[1];return {};}
      if(original_geometry && segment_extent>=p_.horizon_m) long_geometry_ready.insert(horizon);
      ArcPath arc(smooth);
      if(align_spatial_curvature)arc.includeSpatialCurvature(p_.output_step_m);
      double alternate_clearance_progress=0;
      if(std::isfinite(staticStopArc(c,arc,original_geometry ? &static_clearance_progress : &alternate_clearance_progress))) {
        if(original_geometry) static_center_blocked=true;
      }
      // #OBSTACLE_APPROACH
      // 공간 창 안의 먼 정적 장애물 때문에 중앙 형상을 즉시 폐기하지 않는다.
      // planSpeed의 원래 장애물 앞 정지 거리·매 상태의 제동 여유·시간별 충돌
      // 검사로 안전한 접근 궤적을 계산하고, 아래에서 회피 궤적과 비용을 비교한다.
      // 장애물은 제거하지 않으며 시간 계획과 최종 검사를 통과해야 반환한다.
      const double stop_arc=stopArc(ego,reference,arc,limits);
      if(stop_arc<0 && limits.release_time_sec>0) return {};
      auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map,beam_width,truncated);
      if(!path.empty() && path.back().speed_mps>0 &&
         checkPlannedPath(ego,reference_map,predictions,limits,path)) return path;
      const auto connected=conditionInitialGeometry(c,ego,smooth,!pending_lane_change);
      const auto connected_path=timed_connection(connected);
      if(!connected_path.empty())return connected_path;
      // #HIGH_SPEED_AVOIDANCE_WINDOW
      // map 실측 v/a의 정지 가능 거리가 기존 짧은 복구 창(2*horizon_m)보다
      // 길면, 중앙의 분할 C2 시간 계획 실패 뒤 동일한 긴 창의 단일 C2도
      // 검사한다. 짧은 창에서는 정지 가능 공간이 모자라 빈 경로만 반복됐다.
      // 기존 Cartesian 연결식·네 공간 길이·초기 뒷차축 운동을 재사용한다.
      // GT ID·지도 좌표에 의존하지 않고 원본 객체 예측과 ROI/허용 도로,
      // 전체 차체·곡률·속도·가감속·jerk·5초 swept 충돌 검사를 유지한다.
      // 첫 분할 후보가 유효하면 기존 반환을 보존하고, 추가 후보 실패 시
      // 원래 DP/QP·횡 격자·짧은 복구 탐색을 계속한다. 입력 시각은 바꾸지 않는다.
      if(segment_extent==p_.horizon_m && !static_center_blocked &&
         brakingDistance(ego.speed_mps,ego.acceleration_mps2)>2*p_.horizon_m) {
        const auto single=cartesianRecovery(c,ego,horizon,0);
        if(!single.empty()) {
          ArcPath single_arc(single);
          if(align_spatial_curvature)single_arc.includeSpatialCurvature(p_.output_step_m);
          const double single_stop=stopArc(ego,reference,single_arc,limits);
          if(single_stop>=0 || limits.release_time_sec<=0) {
            auto candidate=planSpeed(c,single_arc,ego,limits,duration,single_stop,reference_map,beam_width,truncated);
            if(!candidate.empty() && candidate.back().speed_mps>0 &&
               checkPlannedPath(ego,reference_map,predictions,limits,candidate))return candidate;
          }
        }
      }
      return {};
    };
    // #SPEED_SCALED_C2_COMPARISON
    // 기존 25/50m C2 연결과 네 공간 창을 모두 유지한다. 고주로에서 현재
    // 속도로 기존 25m를 이동할 시간 안에 속도를 잃는 후보에만, 일반도로
    // 대비 요청 상한 비율로 늘린 연결 한 개를 추가 비교한다. 그 시간 동안
    // 초기 속도를 유지하고 원래 비용도 낮은 검증 궤적만 즉시 선택한다.
    // 새 후보가 실패하거나 이득이 없으면 원래 후보·비교 순서를 보존한다.
    // 표본 예산·실측 v/a·도로·충돌·곡률·횡가속도·가감속·jerk 검사는 그대로다.
    // 실제 100→60 제한 전환은 변경하지 않으며, 무한 입력 상한은 기존처럼
    // 차량 설정 상한으로 제한한다. 객체/ROI 계약과 MPC 설정은 바꾸지 않는다.
    const double long_segment_extent=2*p_.horizon_m*std::max(1.0,
        p_.normal_speed_limit_mps>0 ? std::min(p_.max_speed_mps,limits.maximum_speed_mps)/p_.normal_speed_limit_mps : 1.0);
    const auto timed_center=[&]() -> std::vector<PathPoint> {
    // #REACHABLE_LONG_CONNECTION
    // 고속 차선변경에서 기존 25/50m 연결이 모두 시간 계획에 실패하면,
    // 이미 계산한 고속 연결 길이도 같은 네 공간 창에서 검사한다. 짧은
    // 유효 경로가 있어야 긴 연결을 비교하던 조건은 빈 경로를 복구하지 못한다.
    // 기존 25/50m 성공 순서를 유지하고, 긴 길이가 실제로 더 클 때만 한 번
    // 추가한다. C2 계산식·초기 뒷차축 운동·목표 중앙·전체 예측과 차체/도로/
    // 충돌/곡률/횡가속도/가감속/jerk 검사는 유지하며 새 창·계수는 만들지 않는다.
    // GT ID나 지도 위치에 의존하지 않아 LiDAR도 같은 map 계약을 사용한다.
    std::vector<double> segment_extents{p_.horizon_m,2*p_.horizon_m};
    // Retain the reference curve through short C2 pieces before trying a
    // coarse connection that can cut a turn and bulge at its exit.
    const bool prefer_reference_curve=p_.preferred_reference_piece_m>0 && !pending_lane_change && !lane_change_ &&
        (!c.native_region || !c.native_region->recovery_original);
    if(prefer_reference_curve) {
      segment_extents.insert(segment_extents.begin(),p_.preferred_reference_piece_m/2);
      segment_extents.insert(segment_extents.begin(),p_.preferred_reference_piece_m);
    }
    if(long_segment_extent>2*p_.horizon_m)segment_extents.push_back(long_segment_extent);
    segment_extents.push_back(p_.horizon_m/2);segment_extents.push_back(p_.horizon_m/4);
    for(double segment_extent:segment_extents)
    for(double horizon:{planning_horizon_m,.75*planning_horizon_m,
                        .5*planning_horizon_m,.375*planning_horizon_m}) {
      if(static_center_blocked) break;
      if(segment_extent<p_.horizon_m && long_geometry_ready.count(horizon)) continue;
      if(horizon<=p_.horizon_m || horizon/p_.horizon_m>500) continue;
      auto path=center_candidate(horizon,segment_extent);
      if(path.empty()) continue;
      if(prefer_reference_curve && segment_extent==p_.preferred_reference_piece_m && path.back().speed_mps>0)
        return path;
      // #VERIFIED_BRAKING_CANDIDATE_COMPARISON
      // map 도로의 같은 전체 계산 범위에서도 짧은 C2 연결은 불필요한 곡률과
      // 급감속을 만들 수 있다. 첫 속도 계획 시간 격자 안에 제동이 있으면 기존 두 배 길이
      // 연결 한 개를 같은 초기 상태로 비교한다. 고속 후보의 비교 조건은 아래에서
      // 기존 25m 이동 시간 안의 속도 손실도 확인한다. 새 공간 창이나 다른 차선을
      // 추가하지 않고, 정적 차단·전체 차체·도로·동적 충돌·정지선·속도·가감속·
      // jerk 검사를 통과한 후보 중 기존 시간 궤적 비용이 낮은 결과를 선택한다.
      // 비교 후보가 실패하거나 비용이 같으면 원래 검증된 후보를 유지한다.
      // ROI/검출 입력이 바뀌어도 같은 map 객체·도로 계약과 검사를 유지해야 한다.
      const double comparison_time=ego.speed_mps>0 ? std::min(duration,p_.horizon_m/ego.speed_mps) : 0;
      const bool early_speed_loss=std::any_of(path.begin(),path.end(),[&](const PathPoint& point) {
        return point.time_sec<=comparison_time && point.speed_mps<ego.speed_mps-1e-6;
      });
      const bool compare_scaled=!static_center_blocked && long_segment_extent>2*p_.horizon_m &&
          segment_extent<=2*p_.horizon_m && early_speed_loss;
      if(compare_scaled || (segment_extent==p_.horizon_m && std::any_of(path.begin(),path.end(),
          [&](const PathPoint& point){
            return point.time_sec<=p_.speed_time_step_sec && point.acceleration_mps2<0;
          }))) {
        // #LEAD_FOLLOWING_GEOMETRY_CHOICE
        // 곡선 속도 목표보다 앞차 추종 목표가 낮은 직진에서는 검증된 중앙
        // 형상을 유지한다. 가감속의 원인이 앞차인데 같은 중앙 형상을 두 길이로
        // 다시 만드는 속도 탐색은 입력 만료를 일으켰다. 곡률이 목표를 제한하는
        // 지점이 있거나 앞차가 목표를 낮추지 않으면 기존 형상 비교를 계속한다.
        // 실제 변경/차선 유지 후보의 비용 비교는 아래에서 별도로 수행한다.
        const double following_ceiling=std::min({p_.max_speed_mps,limits.maximum_speed_mps,limits.preferred_speed_mps});
        const ArcPath timed_arc(path);
        bool following_limited=false,curve_limited=false;
        for(std::size_t i=0;i<path.size();++i) {
          const auto& point=path[i];
          const double following=followingSpeedTarget(c,point.pose,point.time_sec,following_ceiling,point.speed_mps,reference.at(origin.s+point.progress_m).yaw);
          following_limited=following_limited || following<following_ceiling;
          const double curvature=timed_arc.curvature(timed_arc.station[i],timed_arc.station[i]);
          const double acceleration=p_.preferred_lateral_accel_mps2>0 ?
              p_.preferred_lateral_accel_mps2 : p_.max_lateral_accel_mps2;
          curve_limited=curve_limited || (curvature>1e-12 && std::sqrt(acceleration/curvature)<following);
        }
        if(following_limited && !curve_limited)return path;
        if(compare_scaled) {
          auto scaled=center_candidate(horizon,long_segment_extent);
          if(!scaled.empty() && std::all_of(scaled.begin(),scaled.end(),[&](const PathPoint& point) {
                return point.time_sec>comparison_time || point.speed_mps>=ego.speed_mps-1e-6;
              }) && trajectoryCost(c,scaled,following_ceiling)<trajectoryCost(c,path,following_ceiling))
            return scaled;
          // #WIDENING_C2_CONNECTION
          // map에서 짧은 합법 변경 lanelet 뒤로 연결 도로가 계속 열려 있어도
          // 참조선의 짧은 전환을 그대로 따라가면 큰 곡률 때문에 접근부터 감속한다.
          // 원본 map 좌표의 차로 개방 지점까지 기존 분할 C2로 접근한 다음,
          // 같은 공간 창의 나머지를 하나의 C2로 연결해 완만한 변경도 비교한다.
          // 개방 지점은 뒤쪽 output_step 격자로 올림해 이르지 않게 잡고,
          // 접근 구간을 정수 격자로 나눠 기존 600구간 예산을 넘기지 않는다.
          // 빠른 탐색을 가로막지 않도록 전체 beam에서만 한 후보를 추가하며,
          // 기존 두 배 비교 시간 동안 초기 속도 유지와 더 낮은 비용을 요구한다.
          // 원본 v/a·전체 차체·합법 도로·객체 예측·곡률·횡가속도·가감속·jerk
          // 검사를 모두 통과해야 한다. GT ID나 특정 지도 좌표를 사용하지 않으며,
          // 같은 map 객체 계약을 공급하는 LiDAR에도 적용된다. 복귀 중에는 쓰지
          // 않고 실패하면 기존 후보를 보존한다. GlobalPath/경유점과 MPC는 유지한다.
          if(beam_width==p_.speed_beam_width && ego.acceleration_mps2>=0 &&
             c.native_region && !c.native_region->recovery_original &&
             geometry_cache && geometry_cache->independent_entries) {
            for(const auto& entry:*geometry_cache->independent_entries) {
              const double opening=reference.project(entry,origin.s,reference.length()).s-origin.s;
              if(opening<=p_.min_horizon_m || opening>=horizon-p_.min_horizon_m)continue;
              const int steps=static_cast<int>(std::ceil(opening/p_.output_step_m));
              const double approach=steps*p_.output_step_m;
              if(approach<=p_.min_horizon_m || approach>=horizon-p_.min_horizon_m)continue;
              const int count=static_cast<int>(std::ceil(approach/p_.horizon_m));
              GeometrySchedule goals;
              for(int i=0;i<count;++i)
                goals.push_back({(steps/count+(i<steps%count ? 1 : 0))*p_.output_step_m,0});
              goals.push_back({horizon-approach,0});
              auto direct=timed_connection(center_geometry(goals));
              if(!direct.empty() && std::all_of(direct.begin(),direct.end(),[&](const PathPoint& point) {
                    return point.time_sec>std::min(duration,2*comparison_time) || point.speed_mps>=ego.speed_mps-1e-6;
                  }) && trajectoryCost(c,direct,following_ceiling)<trajectoryCost(c,path,following_ceiling))
                return direct;
              break;
            }
          }
        }
        if(segment_extent!=p_.horizon_m) return path;
        auto alternative=center_candidate(horizon,2*p_.horizon_m);
        const double ceiling=std::min({p_.max_speed_mps,limits.maximum_speed_mps,limits.preferred_speed_mps});
        if(!alternative.empty() && trajectoryCost(c,alternative,ceiling)<trajectoryCost(c,path,ceiling))
          path=std::move(alternative);
        // #EXIT_PATH_CONTINUITY
        // 첫 긴 중앙 후보가 즉시 제동하면 기존 네 공간 창 중 바로 다음 창도
        // 같은 C2 연결기로 비교한다. 공간 분할의 끝점 차이 때문에 앞차가
        // 멀어져도 불필요한 초기 제동을 택하는 경우를 줄인다. 새 창이나 계수는
        // 추가하지 않고, 예측 시간 전체와 차체·도로·객체·동역학 검사 및 기존
        // 시간 궤적 비용을 유지한다. 더 짧은 창이 실패하거나 비용이 크면
        // 원래 후보를 보존하며, 정상 가속/순항 후보의 추가 탐색은 하지 않는다.
        for(double next_horizon:{.75*planning_horizon_m,.5*planning_horizon_m,
                                 .375*planning_horizon_m}) {
          if(next_horizon>=horizon || next_horizon<=p_.horizon_m) continue;
          auto shifted=center_candidate(next_horizon,2*p_.horizon_m);
          if(!shifted.empty() && trajectoryCost(c,shifted,ceiling)<trajectoryCost(c,path,ceiling))
            path=std::move(shifted);
          break;
        }
      }
      return path;
    }
    return {};
    }();
    // #LANE_CHANGE_SPEED_CHOICE
    // 현재 뒷차축이 합법 변경의 출발 lanelet 안에 있고 객체가 있을 때,
    // 즉시 목표 중앙으로 향하는 경로가 첫 시간 격자부터 감속한다면 기존
    // DP/QP·현재 횡 offset 유지 후보와 전체 시간 궤적 비용을 비교한다.
    // 옆차의 map 절대 속도와 예측은 원래 swept 충돌 검사에 사용한다.
    // 현재 차선에서 먼저 전진할 수 있어도 중앙 후보의 조기 반환 때문에
    // 뒤차를 보내는 감속만 택하던 누락을 없앤다. 감속 없는 변경이 가능하면
    // 원래 중앙 우선순위를 유지해 간격이 열린 뒤 변경을 계속 미루지 않는다.
    // 차선 경계를 건넌 뒤에는 출발 차선 유지 선택을 새로 시작하지 않는다.
    // 도로 합집합·합법 방향·차체·속도·가감속·jerk 한도는 유지하며,
    // GT ID를 사용하지 않아 같은 계약의 LiDAR 추적 객체에도 적용된다.
    // #MANEUVER_SCENE_RELEVANCE
    // 수 km 떨어진 객체 때문에 빈 도로와 다른 차선 후보를 비교하지 않는다.
    // 전체 허용 도로의 바깥 범위에 원래 회전/차체 및 추종 거리 여유까지 더해
    // 분리된 경우만 제외한다. 객체·ROI·예측 배열 자체는 그대로 사용한다.
    bool maneuver_objects=false;
    if(pending_lane_change && !predictions.current.empty()) {
      Geometry::Bounds road_bounds;
      for(const auto& point:boundary)road_bounds.include(point);
      if(c.native_region)for(const auto& polygon:c.native_region->polygons)
        for(const auto& point:polygon.outer())road_bounds.include({point.x(),point.y()});
      maneuver_objects=potentialCollision(c,road_bounds,
          p_.max_speed_mps*p_.following_time_headway_sec+p_.following_min_gap_m);
    }
    // #MEASURED_ACCELERATION_MERGE_CHOICE
    // 현재 차선에 차체 전체가 남아 있는 합법 변경에서, 계획이 실측보다
    // 큰 가속을 요구할 때의 뒤차 접근도 비교한다. R205에서는 계획 약 2m/s²,
    // 실측 약 0.8m/s²의 차이로 합류 여유가 사라졌다. 같은 map 형상을 실측의
    // 비음수 가속이 유지되는 시각표로 조회하고, 그 진행이 원래 계획보다
    // 항상 뒤에 있을 때만 원래 예측 객체와 기존 swept 충돌 검사를 수행한다.
    // 겹침이 예상되면 기존 현재 차선 후보도 비교해 간격을 확보하며 전진한다.
    // 새 기하/속도 후보·계수·안전 한도나 속도 offset은 추가하지 않는다.
    // 이 관찰 모델은 실행 가속의 보장이 아니라 후보 선택 기준이다. 반환
    // 경로는 실측 초기 운동량과 기존 차체/도로/충돌/곡률/가감속/jerk 검사를
    // 그대로 통과해야 한다. 현재 차선을 벗어난 뒤에는 새로운 유지 후보를
    // 시작하지 않고, 감속 중에는 기존 선택을 보존한다. GT ID/지도 위치를
    // 쓰지 않으므로 LiDAR도 동일한 map 절대 속도·bbox·예측 시각을 공급해야 한다.
    bool measured_acceleration_conflict=false;
    if(pending_lane_change && hold_reference && maneuver_objects &&
       !timed_center.empty() && ego.acceleration_mps2>=0) {
      const ArcPath arc(timed_center);
      const double duration=timed_center.back().time_sec;
      const SpeedEdge measured{0,ego.speed_mps,ego.acceleration_mps2,
          ego.speed_mps+ego.acceleration_mps2*duration,ego.acceleration_mps2,0,duration};
      bool behind_plan=false,bounded=true;
      for(std::size_t i=0;i<timed_center.size();++i) {
        const double station=measured.distance(timed_center[i].time_sec);
        if(station>arc.station[i]+1e-8)bounded=false;
        if(station<arc.station[i]-1e-6)behind_plan=true;
      }
      if(bounded && behind_plan)for(double time=0;time<duration;) {
        const double next=std::min(duration,time+p_.speed_time_step_sec);
        const SpeedEdge edge{measured.distance(time),measured.speed(time),ego.acceleration_mps2,
            measured.speed(next),ego.acceleration_mps2,time,next-time};
        if(!collisionEdgeClear(c,arc,edge)){measured_acceleration_conflict=true;break;}
        time=next;
      }
    }
    // #COMMITTED_LANE_CHANGE_TIMING
    // 차체가 이미 두 차선에 걸쳐 현재 차선을 유지할 수 없다면, 검증된
    // 중앙 변경 경로의 감속은 새로운 횡 후보 전체를 재탐색할 이유가 아니다.
    // R211에서는 그 탐색이 전부 실패한 뒤 같은 중앙 경로를 늦게 반환하여
    // 제어 입력 만료를 유발했다. 출발 차선 유지가 가능한 동안은 기존의
    // 가속/유지/감속 비교를 수행하고, 유효 중앙 경로가 없으면 기존 회피 탐색을
    // 계속한다. 실제 map 차체/객체·시간 계획과 모든 최종 검사는 유지한다.
    const bool compare_maneuvers=pending_lane_change && maneuver_objects &&
        (hold_reference || timed_center.empty()) && requested_limits.maximum_speed_mps>0 && (timed_center.empty() || measured_acceleration_conflict ||
        std::any_of(timed_center.begin(),timed_center.end(),[&](const PathPoint& point) {
          return point.time_sec<=p_.speed_time_step_sec && point.acceleration_mps2<0;
        }));
    if(!timed_center.empty() && !static_center_blocked && !compare_maneuvers) return prefer_body_reserve(timed_center);
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
      const double meaningful=std::min(p_.dp_step_m,
          duration*std::max(.5,ego.speed_mps));
      return path.back().progress_m>meaningful;
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
        if(attempt<4) for(int q=1;q<8;++q) {
          const double intermediate=span*(1-q/16.0);
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
          if(p_.static_forward_shape_reserve_m>0 && extent-covered>=p_.static_forward_shape_reserve_m) {
            goals.push_back({p_.static_forward_shape_reserve_m,lateral});
            covered+=p_.static_forward_shape_reserve_m;completed=covered;
          }
          if(extent-covered>=p_.min_horizon_m) {
            const double returning=std::min(span,extent-covered);
            goals.push_back({returning,0});covered+=returning;completed=covered;
            while(extent-covered>=p_.min_horizon_m) {
              const double next=std::min(p_.horizon_m,extent-covered);
              goals.push_back({next,0});covered+=next;
            }
          }
          const auto shape=center_geometry(goals);
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
          auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map,beam_width,truncated);
          if(!path.empty() && path.back().speed_mps>0 && has_initial_progress(path) &&
             checkPlannedPath(ego,reference_map,predictions,limits,path)) {
            // A clear avoidance geometry must not lose to the short-term
            // cost of approaching the blocking centerline. The time window
            // may finish before the obstacle while the full route still
            // needs the same detour and return.
            return path;
          }
        }
      }
    }
    std::vector<PathPoint> source_lane_shape;
    if(hold_reference && compare_maneuvers) {
      const Reference hold_road(*hold_reference);
      const Context hold_context{hold_road,predictions,unused,boundary,0,offset,true,
          &geometry->index,geometry->native_region.get()};
      const double start=hold_road.project(ego.pose.position,0,hold_road.length()).s;
      const double extent=std::min(planning_horizon_m,hold_road.length()-start-
          std::hypot(p_.vehicle.front_m,p_.vehicle.width_m/2)-p_.road_margin_m-1e-6);
      for(double fraction:{1.,.75,.5,.375}) {
        const double horizon=extent*fraction;
        if(horizon<p_.min_horizon_m)continue;
        const int count=static_cast<int>(std::ceil(horizon/p_.horizon_m));
        auto shape=cartesianPieces(hold_context,ego,
            std::vector<std::pair<double,double>>(count,{horizon/count,0}),true);
        if(!shape.empty()) {
          source_lane_shape=std::move(shape);break;
        }
      }
    }
    // #VALIDATED_SOURCE_LANE_PRIORITY
    // 차선 변경이 즉시 감속을 요구해도 현재 차선의 가속/유지 경로는 가능할 수
    // 있다. 느린 앞차별 목표속도 비용 때문에 이 후보를 늦은 QP 탐색 뒤에
    // 놓으면 불필요한 양보 감속과 입력 만료가 반복된다. 실제 출발 차선의
    // 기존 C2 후보를 먼저 시간 계획하고, 초기 가속도보다 제동을 더하지 않는
    // 경로만 선택한다. laneHoldReference는 전체 차체가 출발 차선에 포함될 때만
    // 제공된다. 이미 변경 중인 차선을 되돌리는 shortcut으로 사용하지 않는다.
    // 동일 map 예측·5초 시간축·차체·충돌·도로·속도·가감속·jerk 최종 검사를
    // 모두 통과해야 한다. 변경이 감속 없이 가능하면 앞의 중앙 우선 반환을
    // 유지하므로 빈 옆 차선 변경을 미루지 않는다. 실패 시 원래 후보 비교를
    // 계속하고, 같은 계산 안의 형상·시간 결과만 재사용해 이전 경로를 발행하지 않는다.
    std::vector<PathPoint> source_lane_plan;
    if(!source_lane_shape.empty()) {
      const ArcPath arc(source_lane_shape);
      const double stop_arc=stopArc(ego,reference,arc,limits);
      if(stop_arc>=0 || limits.release_time_sec<=0) {
        source_lane_plan=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map,beam_width,truncated);
        if(!source_lane_plan.empty() && source_lane_plan.back().speed_mps>0 &&
           std::all_of(source_lane_plan.begin(),source_lane_plan.end(),[&](const PathPoint& point) {
             return point.time_sec>p_.speed_time_step_sec ||
                 point.acceleration_mps2>=std::min(0.0,ego.acceleration_mps2)-1e-6;
           }) && checkPlannedPath(ego,reference_map,predictions,limits,source_lane_plan))return source_lane_plan;
      }
    }
    std::vector<std::vector<PathPoint>> proposals;
    std::size_t source_lane_proposal=std::numeric_limits<std::size_t>::max();
    proposals.push_back(generate_geometry(
                       {{0,0},{planning_horizon_m,duration}},true,planning_horizon_m));
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
    // #MERGE_SOURCE_LANE_CHOICE
    // 합류 참조선에 일정한 횡 offset을 주어도 그 선의 횡이동을 따라가므로
    // 현재 차선 유지 후보가 되지 않는다. 실제 출발 차선의 종방향 중심선으로
    // 기존 C2 형상을 하나 만들고, 목표 참조선 기준 비용으로 변경 후보와 비교한다.
    // 속도는 같은 예측 객체·실측 초기 v/a에서 다시 계획한다. 동일한 허용 도로,
    // 차체·충돌·곡률·가감속·jerk 검사에 통과한 경우에만 선택하며, 변경 경로가
    // 감속 없이 가능하면 위의 기존 중앙 우선 반환을 유지한다.
    if(!source_lane_shape.empty()) {
      source_lane_proposal=proposals.size();proposals.push_back(std::move(source_lane_shape));
    }
    // Timing guesses only choose geometric obstacle sides for DP/QP. They are
    // never published, nor treated as measured ego speed. Final timing is solved
    // anew for each candidate and checked against the original predictions.
    // #RELEVANT_AVOIDANCE_GEOMETRY
    // 먼 객체가 목록에만 있어도 추가 DP 후보가 짧은 감속 경로를 만들고,
    // 아래의 일반 C2 창 탐색을 선점했다. 실제 초기 속도에서 최대 가속도를
    // 즉시 적용한 예측 시간 전체의 이동 상한으로 자차 도달 범위를 감싼다.
    // 모든 객체의 미래 위치·회전·bbox 여유와 이 범위가 완전히 분리될 때만
    // 장애물용 추가 형상 탐색을 생략해 일반 도로와 같은 후보를 사용한다.
    // 객체 목록·예측을 자르지 않으며 기존 정지거리·도로·시간별 충돌 검사는
    // 모든 후보에 그대로 수행한다. GT ID·구간 번호에 의존하지 않아 같은
    // map/절대 속도/bbox 계약의 LiDAR 추적 결과에도 적용된다.
    Geometry::Bounds reachable_bounds;
    const double maximum_travel=std::max(0.0,ego.speed_mps)*duration+
        .5*p_.max_acceleration_mps2*duration*duration;
    for(double x:{-maximum_travel,maximum_travel})
      for(double y:{-maximum_travel,maximum_travel})
        reachable_bounds.include(Geometry::add(ego.pose.position,{x,y}));
    if(!predictions.current.empty() && potentialCollision(c,reachable_bounds,0)) {
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
        proposals.push_back(generate_geometry(
            timing,false,proposal_extent,true));
      }
    }
    double best=timed_center.empty() ? std::numeric_limits<double>::infinity() :
        trajectoryCost(c,timed_center,std::min(cap,limits.preferred_speed_mps));
    std::vector<PathPoint> result=timed_center;
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
      const auto matched=i==source_lane_proposal ? shape : matchInitialCurve(c,ego,shape);
      if(matched.empty()) continue;
      const ArcPath arc(matched);
      const double stop_arc=stopArc(ego,reference,arc,limits);
      if(stop_arc<0 && limits.release_time_sec>0) continue;
      auto path=i==source_lane_proposal ? source_lane_plan :
          planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map,beam_width,truncated);
      if(path.empty() || !checkPlannedPath(ego,reference_map,predictions,limits,path)) continue;
      // #VERIFIED_CENTER_PRIORITY
      // 중앙 복귀가 실제 시간 계획과 모든 안전 검사를 통과해 계속 전진할 수
      // 있으면, 더 빠르다는 이유만으로 이전 횡 offset의 직진 후보를 고르지 않는다.
      // 명목 경로가 장애물 때문에 예측 구간 끝에 정지해야 하는 경우는 기존
      // 회피 후보와 비용 비교를 계속한다. 이 우선순위는 빈 경로나 미검증 형상을
      // 발행하지 않으며 도로·방향·충돌·곡률·속도·가감속·jerk 검사를 모두 유지한다.
      if(i==0 && path.back().speed_mps>.5 && !static_center_blocked && !compare_maneuvers) return path;
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
    const bool seek_continuing_road=predictions.current.empty() && cap>0 &&
        !std::isfinite(limits.stop_line_progress_m);
    if(result.empty() || (cap>0 && !has_initial_progress(result)) ||
       (seek_continuing_road && result.back().speed_mps==0)) {
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
          const auto shape=generate_geometry(
              {{0,0},{horizon,duration}},true,horizon);
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
        auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map,beam_width,truncated);
        // A short return-to-road connection is a prefix, not a dead-end road.
        // Join its C2 endpoint to the remaining reference before retiming it;
        // otherwise stopping at this artificial geometry window can win even
        // though the continuing road is clear. Retain the original short
        // candidate when a continuous, fully validated extension is unavailable.
        if(cap>0 && (path.empty() || path.back().speed_mps==0) && matched.back().progress_m<planning_horizon_m) {
          EgoState next=ego;next.pose=matched.back().pose;
          next.curvature_inv_m=matched.back().curvature_inv_m;
          const auto tail_origin=reference.project(next.pose.position,0,reference.length());
          const double remaining=.99*std::min(planning_horizon_m-matched.back().progress_m,
                                             reference.length()-tail_origin.s);
          if(remaining>=p_.min_horizon_m) {
            const int pieces=static_cast<int>(std::ceil(remaining/p_.horizon_m));
            const auto tail=cartesianPieces(c,next,GeometrySchedule(pieces,{remaining/pieces,0}));
            if(!tail.empty()) {
              auto extended=matched;const double offset=matched.back().progress_m;
              for(std::size_t i=1;i<tail.size();++i) {
                auto point=tail[i];point.progress_m+=offset;point.time_sec=point.progress_m;
                extended.push_back(point);
              }
              const ArcPath extended_arc(extended);
              const double extended_stop=stopArc(ego,reference,extended_arc,limits);
              auto candidate=planSpeed(c,extended_arc,ego,limits,duration,extended_stop,reference_map,beam_width,truncated);
              if(!candidate.empty() && candidate.back().speed_mps>0 &&
                 checkPlannedPath(ego,reference_map,predictions,limits,candidate)) {
                path=std::move(candidate);
              }
            }
          }
        }
        // #SINGLE_CENTER_WINDOW_SEARCH
        // 이 창은 위에서 중앙 C2 형상을 먼저 검사했다. 형상이 없었으면
        // 같은 입력의 중앙 C2 재호출도 비고, 형상이 있었다면 같은 시간 계획을
        // 반복해도 결과가 같다. 이전 순서에서 필요했던 후행 재시도를 제거한다.
        // 다른 창·DP 횡 후보·전체 도로/충돌/동역학 최종 검사는 그대로 유지한다.
        if(path.empty() || !checkPlannedPath(ego,reference_map,predictions,limits,path)) continue;
        if(cap==0 || (has_initial_progress(path) && (!seek_continuing_road || path.back().speed_mps>0))) {
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
            auto path=planSpeed(c,arc,ego,limits,duration,stop_arc,reference_map,beam_width,truncated);
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
    // #FUTURE_LANE_CHANGE_CURVE_CONNECTION
    // map 경로에 앞으로의 합법 차선 변경이 있으면 hold_reference가 미리
    // 만들어진다. 아직 변경을 시작하지 않은 일반 곡선까지 변경 중으로
    // 분류하면 기존 15m 초기 연결이 차단되어, 실제 차체가 도로 안에 있고
    // 추종이 정상이어도 첫 0.5초의 곡률/속도 연결 실패로 경로가 비었다.
    // 원래 탐색이 전부 빈 경우에만 현재 map 뒷차축·실측 v/a/곡률에서
    // 기존 네 공간 창의 15m QP 연결을 검사한다. 앞서 계산한 기하는 같은
    // 호출의 cache에서 재사용하고 10m 시간 계획은 반복하지 않는다.
    // 정상 결과와 실제 변경 중의 선택은 보존한다. 객체 예측·차체 전체
    // 도로 포함·충돌·곡률·횡가속도·jerk와 최종 시간 경로 검사는 그대로다.
    // 새로운 차선이나 정지 후 복귀 영역을 허용하지 않으며, GT ID나
    // 구간 좌표에 의존하지 않아 LiDAR/ROI도 기존 map·시각 계약을 따른다.
    if(result.empty() && pending_lane_change && hold_reference && !lane_change_) {
      for(double horizon:{planning_horizon_m,.75*planning_horizon_m,.5*planning_horizon_m,.375*planning_horizon_m}) {
        const int segments=static_cast<int>(std::ceil(horizon/p_.horizon_m));
        const GeometrySchedule goals(segments,{horizon/segments,0});
        auto smooth=center_geometry(goals);
        if(smooth.empty())smooth=center_geometry(goals,true);
        const auto candidate=timed_connection(conditionInitialGeometryWithin(c,ego,smooth,15.0));
        if(!candidate.empty())return candidate;
      }
    }
    return prefer_body_reserve(std::move(result));
  }

  // Rechecks the cubic-speed / arc-interpolated representation above. This is
  // NOT checkTimedPath's legacy linear-in-time interpolation contract.
  bool checkPlannedPath(const EgoState& ego, const std::vector<ReferencePoint>& reference_map,
                        const Predictions& predictions, const PlanningConstraints& requested_limits,
                        const std::vector<PathPoint>& path,bool supplied_geometry=false) const {
    requireSceneDomain(ego,reference_map,predictions);
    requirePlanningDomain(ego,requested_limits);
    const auto limits=lateralRecoveryLimits(ego,requested_limits);
    if (path.size()<2) {path_check_reason_="check-1";++rejected_[9];return false;}
    if(ego.speed_mps*ego.speed_mps*std::abs(ego.curvature_inv_m)>p_.max_lateral_accel_mps2+1e-6 &&
       path.back().speed_mps>std::min(p_.max_speed_mps,limits.maximum_speed_mps)+1e-8){path_check_reason_="check-2";++rejected_[9];return false;}
    for (std::size_t i=0;i<path.size();++i) {
      const auto& q=path[i];
      if (!Geometry::finite(q.pose) || !std::isfinite(q.curvature_inv_m) ||
          !std::isfinite(q.speed_mps) || !std::isfinite(q.acceleration_mps2) ||
          !std::isfinite(q.progress_m) || !std::isfinite(q.time_sec) ||
          (i==0 && (q.progress_m!=0 || q.time_sec!=0)) ||
          (i>0 && (q.time_sec<=path[i-1].time_sec || q.progress_m<path[i-1].progress_m)))
        {path_check_reason_="trajectory-order";++rejected_[9];return false;}
    }
    const double duration=predictions.step_sec*kPredictionSteps-(ego.stamp_sec-predictions.reference_stamp_sec);
    if (ego.stamp_sec<predictions.reference_stamp_sec || path.back().time_sec>duration+1e-12 ||
        Geometry::norm(Geometry::sub(path.front().pose.position,ego.pose.position))>1e-6 ||
        std::abs(Geometry::wrap(path.front().pose.yaw_rad-ego.pose.yaw_rad))>1e-6 ||
        std::abs(path.front().speed_mps-ego.speed_mps)>1e-8 ||
        std::abs(path.front().acceleration_mps2-ego.acceleration_mps2)>1e-8 ||
        std::abs(path.front().curvature_inv_m-ego.curvature_inv_m)>1e-8) {path_check_reason_="check-3";++rejected_[9];return false;}
    const Reference reference(reference_map);
    const auto original_geometry=roadGeometry(reference_map,reference);
    const NativeRegionScope reentry_scope(active_native_region_,roadReentryRegion(ego,original_geometry));
    const auto geometry=roadGeometry(reference_map,reference);
    const auto& boundary=geometry->boundary;
    const std::vector<ProgressTime> unused;
    Context c{reference,predictions,unused,boundary,0,ego.stamp_sec-predictions.reference_stamp_sec,false,&geometry->index,
        supplied_geometry?nullptr:geometry->native_region.get()};
    c.supplied_geometry=supplied_geometry;
    if(!reentryProgressClear(c,path,true)){path_check_reason_="check-4";++rejected_[9];return false;}
    const auto recovery=ceilingRecovery(ego,limits);
    if (!roadClear(c,path.front().pose,path.front().pose) || collision(c,path.front(),path.front())) {path_check_reason_="check-5";++rejected_[9];return false;}
    const auto line=stopLine(ego,reference,limits);
    for (std::size_t i=1;i<path.size();++i) {
      const auto& a=path[i-1]; const auto& b=path[i];
      const double distance=Geometry::norm(Geometry::sub(b.pose.position,a.pose.position));
      const double angle=std::abs(Geometry::wrap(b.pose.yaw_rad-a.pose.yaw_rad));
      if (!std::isfinite(distance) || (distance==0 && (angle>1e-10 || a.progress_m!=b.progress_m)) ||
          (distance>0 && (b.progress_m==a.progress_m ||
             Geometry::dot(Geometry::sub(b.pose.position,a.pose.position),Geometry::unit(a.pose.yaw_rad))<=0 ||
             Geometry::dot(Geometry::sub(b.pose.position,a.pose.position),Geometry::unit(b.pose.yaw_rad))<=0))) {path_check_reason_="check-6";++rejected_[9];return false;}
      const SpeedEdge edge{0,a.speed_mps,a.acceleration_mps2,b.speed_mps,b.acceleration_mps2,
                           a.time_sec,b.time_sec-a.time_sec};
      const double curvature=supplied_geometry ? 0 : std::max({std::abs(a.curvature_inv_m),std::abs(b.curvature_inv_m),
                                      distance>0 ? angle/distance : 0});
      if (!speedAdmissible(edge,ego,limits,curvature,recovery) ||
          std::abs(edge.distance(edge.h)-distance)>2e-6 ||
          !roadClear(c,a.pose,b.pose)) {path_check_reason_="check-7";++rejected_[9];return false;}
      const std::vector<PathPoint> pair{a,b};
      const ArcPath arc(pair);
      if (!collisionEdgeClear(c,arc,edge)) {path_check_reason_="check-8";++rejected_[9];return false;}
      if (std::isfinite(limits.stop_line_progress_m) && a.time_sec<limits.release_time_sec) {
        const double h=std::min(edge.h,limits.release_time_sec-a.time_sec);
        const auto until=arc.at(edge.distance(h));
        if (!behindLine(line,a.pose,until.pose)) {path_check_reason_="check-9";++rejected_[9];return false;}
      }
    }
    const bool accepted=supplied_geometry || sampledGeometryClear(path,true);if(!accepted){path_check_reason_="sampled-geometry";++rejected_[9];}return accepted;
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
      for(double offset:staticSupportOffsets(c,road))
        if(offset>=lower && offset<=upper)offsets.push_back(offset);
      // Keep the existing avoidance grid, but also represent the smallest
      // rear-axle correction that fits the asymmetric Ioniq5 footprint.
      // A whole-body constraint, not a fixed inward shift or a body-center
      // change to the output coordinate contract, determines this candidate.
      for(const auto& interval:roadLateralIntervals(c,road,road.yaw,lower,upper)) {
        const double centered=Geometry::clamp(0.0,interval.first,interval.second);
        offsets.push_back(centered);
        offsets.push_back((centered+Geometry::clamp(origin.l,interval.first,interval.second))/2);
        if(interval.second-interval.first>.02) {
          offsets.push_back(interval.first+.01);
          offsets.push_back(interval.second-.01);
          offsets.push_back((interval.first+interval.second)/2);
        }
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
  mutable std::map<std::uint32_t,std::pair<double,double>> initial_obstacle_margins_;
  double obstacleMargin(std::uint32_t id,double progress) const {
    const auto found=initial_obstacle_margins_.find(id);
    if(found==initial_obstacle_margins_.end())return p_.obstacle_margin_m;
    const double phase=Geometry::clamp((progress-found->second.second)/p_.vehicle.wheelbase_m,0.,1.);
    return found->second.first+(p_.obstacle_margin_m-found->second.first)*phase;
  }
  mutable const char* failure_reason_="geometry";
  mutable double selected_extent_=0,selected_curvature_=0,projection_distance_=0,projection_heading_=0;
  mutable const char* selected_source_="none";
  mutable const char* geometry_reason_="none";
  mutable const char* road_reason_="none";
  mutable std::array<std::size_t,10> rejected_{};
  mutable const char* path_check_reason_="none";
  mutable bool extended_search_=false;
  mutable double failed_layer_=-1;
  mutable unsigned speed_searches_=0,geometry_searches_=0;
  mutable std::vector<PathPoint> validated_stop_;
  mutable Vec2 curve_knot_anchor_;
  mutable double curve_knot_stamp_=0;
  mutable bool curve_knot_initialized_=false;
  PathParameters p_;
  std::vector<std::shared_ptr<const NativeRoadRegion::IndexedRing>> physical_boundaries_;
  void preparePhysicalBoundaries() {
    physical_boundaries_.clear();
    for(const auto& line:p_.physical_boundaries)if(line.size()>1)
      physical_boundaries_.push_back(std::make_shared<const NativeRoadRegion::IndexedRing>(line));
  }
  std::vector<std::pair<double,double>> mission_speed_limits_;
  double missionSpeedTarget(double station,double fallback) const {
    if(mission_speed_limits_.empty())return fallback;
    auto after=std::upper_bound(mission_speed_limits_.begin(),mission_speed_limits_.end(),station,
        [](double s,const std::pair<double,double>& q){return s<q.first;});
    if(after==mission_speed_limits_.begin())return std::min(fallback,after->second);
    if(after==mission_speed_limits_.end())return std::min(fallback,mission_speed_limits_.back().second);
    const auto& a=*(after-1);const auto& b=*after;
    const double u=(station-a.first)/std::max(1e-9,b.first-a.first);
    return std::min(fallback,a.second+std::max(0.,std::min(1.,u))*(b.second-a.second));
  }

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
  mutable std::map<std::uint32_t,double> pedestrian_caution_distances_;
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
    Vec2 velocity;
  };
  struct Context {
    mutable std::map<double,std::vector<FollowingObject>> following_objects;
    mutable std::map<std::pair<double,double>,std::vector<ObjectBounds>> temporal_bounds;
    const Reference& reference;
    const Predictions& predictions;
    const std::vector<ProgressTime>& schedule;
    const std::vector<Vec2>& boundary;
    double start_s;
    double prediction_offset;
    bool geometric_only;
    bool proposal_timing;
    bool supplied_geometry=false;
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
    // Only a measured physical departure gets the inherited steering-response
    // sweep. Normal road and margin-only checks stay unchanged. The area bound,
    // original one-way boundaries and complete return within horizon_m remain.
    const auto footprint=Geometry::sweptEgoEnvelope(ego.pose,ego.pose,p_.vehicle,2*p_.road_margin_m);
    const auto direction=Geometry::mul(Geometry::unit(ego.pose.yaw_rad),p_.horizon_m);
    double outside_limit=outside;
    const bool margin_only=original->containsBox(Geometry::egoBox(ego.pose,p_.vehicle));
    const int response_steps=margin_only?0:std::max(1,int(std::ceil(p_.reentry_response_sec/.02)));
    for(int step=0;step<=response_steps;++step) {
      Pose2 inherited=ego.pose;
      if(step>0) {
        const double t=p_.reentry_response_sec*step/response_steps;
        const double distance=std::max(0.,std::max(ego.speed_mps,p_.reentry_minimum_speed_mps)*t+ego.acceleration_mps2*t*t/2);
        const double turn=ego.curvature_inv_m*distance;
        const double scale=std::abs(turn)>1e-8?2*std::sin(turn/2)/turn:1.;
        inherited.position=Geometry::add(ego.pose.position,
            Geometry::mul(Geometry::unit(ego.pose.yaw_rad+turn/2),distance*scale));
        inherited.yaw_rad+=turn;
        outside_limit=std::max(outside_limit,original->outsideArea(
            Geometry::sweptEgoEnvelope(inherited,inherited,p_.vehicle,p_.road_margin_m)));
      }
      const auto response_box=step==0?footprint:
          Geometry::sweptEgoEnvelope(inherited,inherited,p_.vehicle,2*p_.road_margin_m);
      for(const auto& corner:Geometry::corners(response_box)) {
        corners.emplace_back(corner.x,corner.y);
        corners.emplace_back(corner.x+direction.x,corner.y+direction.y);
      }
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
    region->reentry_outside_limit_m2=region->margin_only_recovery ? 0 : outside_limit;
    region->prepare();return region;
  }

  // A cubic velocity joins endpoint v/a exactly. Position is its quartic
  // integral. This bounded-control lattice uses the s,v,a state of Apollo's
  // arc-time formulation; it is a finite beam search, not Apollo's spline QP.
  struct SpeedEdge {
    double s0,v0,a0,v1,a1,t0,h;
    // Resolve a constant-jerk edge from endpoint accelerations. Subtracting
    // two ground speeds over a microsecond release event otherwise amplifies
    // their rounding error into a fictitious jerk violation.
    double velocityResidual() const {
      const long double residual=static_cast<long double>(v1)-v0-
          .5L*(static_cast<long double>(a0)+a1)*h;
      const double roundoff=8*std::numeric_limits<double>::epsilon()*
          (std::abs(v0)+std::abs(v1)+h*(std::abs(a0)+std::abs(a1)));
      return std::abs(residual)<=roundoff ? 0 : static_cast<double>(residual);
    }
    double c2() const { return (a1-a0)/(2*h)+3*velocityResidual()/(h*h); }
    double c3() const { return -2*velocityResidual()/(h*h*h); }
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
  void pruneSpeedLayer(std::vector<SpeedState>& next,std::size_t beam_width,bool& truncated) const {
    std::stable_sort(next.begin(),next.end(),[](const SpeedState& a,const SpeedState& b){return a.cost<b.cost;});
    if(next.size()<=beam_width) return;
    if(beam_width<p_.speed_beam_width) truncated=true;
    if(beam_width==1) { next.resize(1); return; }
    const auto braking=std::min_element(next.begin(),next.end(),[&](const SpeedState& a,const SpeedState& b) {
      return a.s+brakingDistance(a.v,a.a)<b.s+brakingDistance(b.v,b.a);
    });
    std::vector<bool> selected(next.size(),false); selected[0]=true;
    const std::size_t brake_index=static_cast<std::size_t>(braking-next.begin());
    selected[brake_index]=true;
    std::size_t count=brake_index==0 ? 1 : 2;
    const std::size_t budget=std::max<std::size_t>(1,beam_width/4);
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
    for(std::size_t i=0;count<beam_width;++i)
      if(!selected[i]) { selected[i]=true; ++count; }
    std::vector<SpeedState> retained; retained.reserve(beam_width);
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
    if(a<-d) append((-d-a)/j,j);
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
    std::vector<double> heading_rates;
    std::vector<double> interpolation_knots;
    Geometry::Bounds bounds;
    bool supplied_geometry=false;
    double heading_per_meter=0;
    double yaw_scale=2*Geometry::kPi;
    explicit ArcPath(const std::vector<PathPoint>& input):points(input),station(1,0) {
      for(std::size_t i=1;i<input.size();++i)
        station.push_back(station.back()+Geometry::norm(Geometry::sub(input[i].pose.position,input[i-1].pose.position)));
      for(const auto& point:input) { bounds.include(point.pose.position); yaw_scale=std::max(yaw_scale,std::abs(point.pose.yaw_rad)); }
      heading_rates.resize(input.size(),0);
      for(std::size_t i=1;i<input.size();++i) {
        const double distance=station[i]-station[i-1];
        const double angle=std::abs(Geometry::wrap(input[i].pose.yaw_rad-input[i-1].pose.yaw_rad));
        if(distance>0) heading_rates[i]=angle/distance;
        else if(angle>0) heading_rates[i]=std::numeric_limits<double>::infinity();
        heading_per_meter=std::max(heading_per_meter,heading_rates[i]);
      }
      position_curvature.resize(input.size(),0);
      for(std::size_t i=1;i+1<input.size();++i)
        position_curvature[i]=std::abs(Geometry::curvature(input[i-1].pose.position,input[i].pose.position,
                                                         input[i+1].pose.position));
      if(input.size()>2) {
        position_curvature.front()=position_curvature[1];
        position_curvature.back()=position_curvature[input.size()-2];
      }
      // Lateral control tracks the searched target, not only the measured-
      // state connection. Enforce both curvatures in longitudinal planning;
      // otherwise a gentle execution connection can authorize city speed
      // through a much tighter avoidance target.
      if(!input.empty() && input.front().spatial_reference && input.front().spatial_reference->size()>2) {
        const auto& target=*input.front().spatial_reference;
        std::vector<double> target_k(target.size(),0);
        for(std::size_t i=1;i+1<target.size();++i) {
          target_k[i]=std::abs(Geometry::curvature(target[i-1].position,target[i].position,target[i+1].position));
          const double ds=Geometry::norm(Geometry::sub(target[i+1].position,target[i-1].position));
          if(ds>1e-9)target_k[i]=std::max(target_k[i],
              std::abs(Geometry::wrap(target[i+1].yaw_rad-target[i-1].yaw_rad))/ds);
        }
        target_k.front()=target_k[1];target_k.back()=target_k[target.size()-2];
        for(std::size_t i=0;i<input.size();++i) {
          double best=std::numeric_limits<double>::infinity();std::size_t nearest=0;
          for(std::size_t j=1;j<target.size();++j) {
            const auto delta=Geometry::sub(target[j].position,target[j-1].position);
            const double norm=Geometry::dot(delta,delta);
            const double u=norm>1e-12 ? Geometry::clamp(Geometry::dot(
                Geometry::sub(input[i].pose.position,target[j-1].position),delta)/norm,0.,1.) : 0.;
            const double distance=Geometry::norm(Geometry::sub(input[i].pose.position,
                Geometry::add(target[j-1].position,Geometry::mul(delta,u))));
            if(distance<best){best=distance;nearest=j;}
          }
          position_curvature[i]=std::max({position_curvature[i],target_k[nearest],target_k[nearest-1]});
        }
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
    // #SPATIAL_CURVATURE_SEARCH
    // 실제 map 인접 차선 변경의 중앙 후보와 초기 기하 보정 후보에 최종
    // 검사와 같은 output_step_m 균일 공간 곡률을 먼저 반영한다. 시간 표본 복원 뒤에야 곡률 초과를
    // 발견하면 같은 속도 후보를 수백 번 거부해 제어 입력 만료를 유발한다.
    // 원래 위치의 3점 곡률을 그 범위와 겹치는 기존 arc 구간 상한에 넣고,
    // 속도 탐색 후 원래 전체 최종 검사를 계속 수행한다. 현재 후보만 바꾸며
    // 보정 전 일반 곡선·도로 복귀와 다른 형상의 기존 계산/선택은 유지한다.
    // 초기 v/a·차체/도로/충돌/곡률/횡가속도/jerk 한도와 시각은 유지한다.
    // 새 계수·후보·GT ID는 없으며 LiDAR/ROI 통합도 같은 map bbox·예측
    // 시각과 최종 전체 검사를 유지해야 한다.
    void includeSpatialCurvature(double step) {
      for(double s=step;s+step<=length();s+=step) {
        const double k=std::abs(Geometry::curvature(at(s-step).pose.position,
            at(s).pose.position,at(s+step).pose.position));
        auto it=std::upper_bound(station.begin(),station.end(),s-step);
        for(;it!=station.end() && *(it-1)<=s+step;++it) {
          const std::size_t i=static_cast<std::size_t>(it-station.begin());
          position_curvature[i-1]=std::max(position_curvature[i-1],k);
          position_curvature[i]=std::max(position_curvature[i],k);
        }
      }
    }
    double length() const { return station.back(); }
    std::pair<double,double> headingBounds(double lo,double hi) const {
      double rate=0,variation=0;
      auto it=std::upper_bound(station.begin(),station.end(),lo);
      for(;it!=station.end() && *(it-1)<=hi;++it) {
        const double local=heading_rates[static_cast<std::size_t>(it-station.begin())];
        rate=std::max(rate,local);
        const double span=std::max(0.0,std::min(hi,*it)-std::max(lo,*(it-1)));
        variation+=std::isfinite(local) ? span*local : std::numeric_limits<double>::infinity();
      }
      return {rate,variation};
    }
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
      if(supplied_geometry)return 0;
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
          const auto extrema=edge.extrema();
          const auto exceeds=[&](double t) {
            return std::pow(edge.speed(t),2)*k>p_.max_lateral_accel_mps2+1e-6;
          };
          if(!std::any_of(extrema.begin(),extrema.end(),exceeds) ||
             (recoveringLateralExcess(initial,k,recovery) && belowRecovery(edge,recovery)))continue;
          // #SPATIAL_TIME_ALIGNMENT
          // 균일 공간 재구성의 곡률 k는 map 뒷차축의 [lo,hi] 구간에 대한
          // 상한이다. 일부만 겹치는 시간 edge의 구간 밖 속도까지 곱하면
          // 감속 중인 정상 경로를 거부하고 같은 후보 탐색을 반복하게 된다.
          // 전진 속도 다항식의 거리 적분을 이분해 실제 겹치는 시간 구간을
          // 구하고, 그 양 끝과 구간 안의 모든 속도 극값을 검사한다.
          // 최종 거리 일치 허용값 2e-6m만큼 공간 구간을 넓히고 이분 탐색의
          // 바깥쪽 시각을 사용해 전체 구간을 보수적으로 포함한다. 기존
          // 곡률·횡가속도 한도와 허용 오차, 도로·충돌·가감속·jerk 검사는
          // 유지한다. ROI나 검출 원천과 무관하며 속도를 낮추거나 가산하지 않는다.
          const auto time_at=[&](double distance,bool upper) {
            if(distance<=0)return 0.0;
            if(distance>=edge.distance(edge.h))return edge.h;
            double left=0,right=edge.h;
            for(int iteration=0;iteration<45;++iteration) {
              const double middle=(left+right)/2;
              if(edge.distance(middle)<distance)left=middle;else right=middle;
            }
            return upper ? right : left;
          };
          const double first=time_at(lo-arc.station[j-1]-2e-6,false);
          const double last=time_at(hi-arc.station[j-1]+2e-6,true);
          if(exceeds(first) || exceeds(last))return false;
          for(double t:extrema)if(t>first && t<last && exceeds(t))return false;
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
                                         bool zero_initial_curvature_slope=false,
                                         bool footprint_goal=false) const {
    const auto origin=c.reference.project(ego.pose.position,0,c.reference.length());
    if(extent<=0 || origin.s+extent>c.reference.length()) return {};
    const auto end=cartesianEndpoint(c.reference,origin.s+extent);
    if(footprint_goal && c.native_region && !c.native_region->recovery_original) {
      const Pose2 nominal{end.position,end.yaw};
      if(!roadClear(c,nominal,nominal)) {
        const double lower=end.lower+p_.vehicle.width_m/2+p_.road_margin_m;
        const double upper=end.upper-p_.vehicle.width_m/2-p_.road_margin_m;
        if(lower>upper)return {};
        const auto intervals=roadLateralIntervals(c,end,end.yaw,lower,upper);
        if(intervals.empty())return {};
        double best=std::numeric_limits<double>::infinity();
        for(const auto& interval:intervals) {
          const double offset=Geometry::clamp(terminal_l,interval.first,interval.second);
          if(std::abs(offset)<best){best=std::abs(offset);terminal_l=offset;}
        }
      }
    }
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
      if(!out.empty() && progress<=out.back().progress_m){geometry_reason_="projection-order";return {};}
      out.push_back({{position,std::atan2(first.y,first.x)},progress,progress,
                     Geometry::cross(first,second)/(norm*norm*norm)});
    }
    out.front().pose=ego.pose; out.front().curvature_inv_m=ego.curvature_inv_m;
    const Context geometric{c.reference,c.predictions,c.schedule,c.boundary,origin.s,c.prediction_offset,true,&c.boundary_index,c.native_region,c.proposal_timing};
    if(!lateralSlopeClear(geometric,out)){geometry_reason_="lateral-slope";return {};}
    if(!checkPath(geometric,out))return {};
    return out;
  }

  std::vector<PathPoint> cartesianPieces(const Context& c,const EgoState& ego,
                                        const std::vector<std::pair<double,double>>& goals,
                                        bool repair_initial_slope=false,bool footprint_goals=false) const {
    double intervals=0;
    for(const auto& goal:goals) intervals+=std::max(2.0,std::ceil(goal.first/p_.output_step_m));
    if(intervals>600) return {};
    std::vector<PathPoint> shape;
    EgoState next=ego;
    for(const auto& goal:goals) {
      auto part=cartesianRecovery(c,next,goal.first,goal.second,false,footprint_goals);
      if(part.empty() && repair_initial_slope)
        part=cartesianRecovery(c,next,goal.first,goal.second,true,footprint_goals);
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

  std::vector<double> staticSupportOffsets(const Context& c,const RoadSample& road) const {
    std::vector<double> result;
    const auto normal=Geometry::normal(road.yaw),forward=Geometry::unit(road.yaw);
    for(const auto& object:c.predictions.current) {
      if(Geometry::norm(object.velocity_mps)>1e-10)continue;
      const auto box=Geometry::objectBox(object,p_.obstacle_margin_m);
      const auto delta=Geometry::sub(box.pose.position,road.position);
      if(std::abs(Geometry::dot(delta,forward))>
         Geometry::extent(box,forward)+p_.vehicle.front_m+2*p_.dp_step_m)continue;
      const double lateral=Geometry::dot(delta,normal);
      const double support=Geometry::extent(box,normal)+p_.vehicle.width_m/2+.01;
      result.push_back(lateral-support);result.push_back(lateral+support);
    }
    return result;
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
    for(double offset:staticSupportOffsets(c,end))
      if(offset>=lower && offset<=upper)offsets.push_back(offset);
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
  // #REACHABLE_INITIAL_GEOMETRY
  // 속도 계획이 실패한 C2 형상의 초기 10m만 같은 map 뒷차축 좌표로 다시
  // 최적화한다. 시작 위치·방향·실측 곡률과 접속 끝의 위치·방향·곡률은
  // 등식으로 고정한다. 제어기나 입력 시각을 바꾸지 않는다.
  // ego의 전방축을 x로 둔 cubic B-spline y(x)의 계수를 기존 QP 계산기로
  // 구한다. 현재 차체 방향의 실제 도로 합집합에서, 원래 형상 주변의 같은
  // corridor 안에 있는 연결 성분만 사용한다. 첫 공간 표본을 두 번 더
  // 나누어 초기 곡률 변화가 한 개의 25cm 표본에서 급증하지 않게 한다.
  // 양의 초기 가속도를 기존 최대 jerk로 해제할 때의 불가피한 최고 속도까지
  // 유지 가능한 곡률을 사용한다. |y''| 제한은 실제 곡률보다 보수적이며,
  // QP 잔차 여유를 뺀다. 해가 있어도 전체 차체·진행 방향·공간 형상을 먼저
  // 검사하고, 호출부에서 새 시간 계획·예측 충돌·가감속·jerk를 다시 검사한다.
  // 뒤쪽 경로·속도 프로파일을 재사용하지 않는다. 표본 수와 QP 반복 수를
  // 제한하며 GT ID·구간 번호를 쓰지 않아 같은 map 계약의 LiDAR에도 적용된다.
  std::vector<PathPoint> conditionInitialGeometry(const Context& c,const EgoState& ego,
                                                   const std::vector<PathPoint>& shape,bool extend_connection) const {
    // #INITIAL_CONNECTION_REACHABILITY
    // S자 도로에서 실측 곡률과 10m 뒤의 자세·곡률을 동시에 고정하면,
    // 기존 jerk 제한 감속으로도 그 연결의 횡가속도를 만족하지 못할 수 있다.
    // 먼저 기존 10m QP의 성공 결과를 그대로 유지한다. 일반 곡선 주행에서
    // 연결이 없을 때만 같은 map 형상의 15m 끝점으로 같은 QP를 한 번 검사한다.
    // 차선변경 중에는 기존 연결·후보 선택을 유지해 합류 탐색 비용을 늘리지 않는다.
    // 64개 spline 분할·QP 반복 한도는 늘리지 않으며, 시작/끝 위치·방향·
    // 곡률 등식과 실제 차체·도로 corridor를 보존한다. 연결 뒤의 형상은
    // 그대로 잇고 호출부에서 원본 객체 예측으로 전체 시간 계획·swept 충돌·
    // 횡가속도·가감속·jerk를 다시 검증한다. 실패한 경로를 발행하거나
    // 이전 속도 계획을 재사용하지 않는다. GT ID·구간 번호에 의존하지 않아
    // LiDAR/ROI 통합에서도 같은 map 객체·좌표·시각 계약을 사용해야 한다.
    auto result=conditionInitialGeometryWithin(c,ego,shape,10.0);
    if(!result.empty() || !extend_connection)return result;
    return conditionInitialGeometryWithin(c,ego,shape,15.0);
  }
  std::vector<PathPoint> conditionInitialGeometryWithin(const Context& c,const EgoState& ego,
                                                   const std::vector<PathPoint>& shape,double extent) const {
    if(shape.size()<4)return {};
    const auto endpoint=std::lower_bound(shape.begin(),shape.end(),std::min(extent,shape.back().progress_m),
        [](const PathPoint& q,double s){return q.progress_m<s;});
    if(endpoint==shape.end())return {};
    const auto tangent=Geometry::unit(ego.pose.yaw_rad),normal=Geometry::normal(ego.pose.yaw_rad);
    std::vector<double> gx,gy,gyaw;
    for(auto it=shape.begin();it!=endpoint+1;++it) {
      const auto delta=Geometry::sub(it->pose.position,ego.pose.position);
      const double x=Geometry::dot(delta,tangent),y=Geometry::dot(delta,normal);
      if(!gx.empty() && x<=gx.back()+1e-8)return {};
      gx.push_back(x);gy.push_back(y);gyaw.push_back(Geometry::wrap(it->pose.yaw_rad-ego.pose.yaw_rad));
    }
    const double length=gx.back();
    if(length<1 || std::abs(gyaw.back())>1)return {};
    using Spline=Eigen::Spline<double,1,3>;
    std::vector<double> breaks{0,.1,.2};
    for(double x=p_.output_step_m;x<length-1e-8;x+=p_.output_step_m)breaks.push_back(x);
    breaks.push_back(length);
    if(breaks.size()>64)return {};
    std::sort(breaks.begin(),breaks.end());
    breaks.erase(std::unique(breaks.begin(),breaks.end(),[](double a,double b){return std::abs(a-b)<1e-8;}),breaks.end());
    typename Spline::KnotVectorType knots(breaks.size()+6);
    int k=0;for(int i=0;i<4;++i)knots(k++)=0;
    for(std::size_t i=1;i+1<breaks.size();++i)knots(k++)=breaks[i]/length;
    for(int i=0;i<4;++i)knots(k++)=1;
    const int controls=knots.size()-4,m=breaks.size();
    const auto basis=[&](double x,int derivative) {
      Eigen::RowVectorXd row=Eigen::RowVectorXd::Zero(controls);
      const double u=x/length;
      const auto span=Spline::Span(u,3,knots);
      const auto values=Spline::BasisFunctionDerivatives(u,derivative,3,knots);
      row.segment(span-3,4)=values.row(derivative)/std::pow(length,derivative);
      return row;
    };
    Eigen::MatrixXd b(m,controls),d1(m,controls),d2(m,controls),d3(m,controls);
    Eigen::VectorXd guide(m),preferred(m),lower(m),upper(m),curvature(m);
    const double peak_speed=ego.speed_mps+std::pow(std::max(0.0,ego.acceleration_mps2),2)/(2*p_.max_longitudinal_jerk_mps3);
    const double hold_curvature=std::min(p_.max_curvature_inv_m,p_.max_lateral_accel_mps2/std::max(1e-8,peak_speed*peak_speed));
    for(int i=0;i<m;++i) {
      const double x=breaks[i];b.row(i)=basis(x,0);d1.row(i)=basis(x,1);d2.row(i)=basis(x,2);d3.row(i)=basis(x,3);
      auto it=std::upper_bound(gx.begin(),gx.end(),x);const auto j=std::max<std::size_t>(1,std::min<std::size_t>(gx.size()-1,it-gx.begin()));
      const double u=Geometry::clamp((x-gx[j-1])/(gx[j]-gx[j-1]),0.,1.);
      guide[i]=Geometry::lerp(gy[j-1],gy[j],u);
      const double yaw=Geometry::yawLerp(gyaw[j-1],gyaw[j],u);
      RoadSample sample;sample.position=Geometry::add(ego.pose.position,Geometry::mul(tangent,x));sample.yaw=ego.pose.yaw_rad;
      const auto intervals=roadLateralIntervals(c,sample,Geometry::wrap(ego.pose.yaw_rad+yaw),
          guide[i]-p_.corridor_half_width_m,guide[i]+p_.corridor_half_width_m);
      if(intervals.empty())return {};
      // A geometric seed is an objective, not a feasibility certificate.
      // Project an out-of-corridor seed onto the nearest actual footprint
      // interval; the spline and its continuous road sweep are checked below.
      const auto distance=[&](const std::pair<double,double>& r) {
        return std::max({0.,r.first-guide[i],guide[i]-r.second});
      };
      const auto range=std::min_element(intervals.begin(),intervals.end(),
          [&](const std::pair<double,double>& a,const std::pair<double,double>& b){return distance(a)<distance(b);});
      lower[i]=range->first;upper[i]=range->second;
      // #FOOTPRINT_RESERVE
      // 정상 C2 시간 계획은 호출부에서 즉시 반환해 기존 경로를 유지한다.
      // 실패한 후보를 다시 최적화할 때만 도로 경계에 붙은 원래 형상 대신,
      // 앞 범퍼·뒤 범퍼를 모두 포함하는 실제 허용 구간의 중앙을 비용 목표로
      // 둔다. 뒷차축을 차체 중심으로 바꾸지 않고 같은 corridor 안에서만
      // 횡 위치를 보정한다. 최종 실제 자세의 도로 포함 검사는 다시 수행한다.
      preferred[i]=(lower[i]+upper[i])/2;
      const double limit=hold_curvature;
      curvature[i]=std::max(0.,limit-4*p_.qp_tolerance);
    }
    Eigen::MatrixXd equality(6,controls);
    equality.row(0)=b.row(0);equality.row(1)=d1.row(0);equality.row(2)=d2.row(0);
    equality.row(3)=b.row(m-1);equality.row(4)=d1.row(m-1);equality.row(5)=d2.row(m-1);
    Eigen::VectorXd value(6);const double finish_slope=std::tan(gyaw.back());
    value<<0,0,ego.curvature_inv_m,gy.back(),finish_slope,endpoint->curvature_inv_m*std::pow(1+finish_slope*finish_slope,1.5);
    Eigen::FullPivLU<Eigen::MatrixXd> decomposition(equality);
    if(decomposition.rank()!=6)return {};
    const Eigen::VectorXd fixed=decomposition.solve(value);
    const Eigen::MatrixXd kernel=decomposition.kernel();
    Eigen::MatrixXd h=2*((p_.reference_weight+p_.dp_tracking_weight+p_.previous_weight)*b.transpose()*b+
        p_.slope_weight*d1.transpose()*d1+p_.bending_weight*d2.transpose()*d2+p_.jerk_weight*d3.transpose()*d3);
    Eigen::VectorXd q=-2*(p_.dp_tracking_weight+p_.previous_weight)*b.transpose()*preferred;
    Eigen::MatrixXd a(2*m,controls);a.topRows(m)=b;a.bottomRows(m)=d2;
    Eigen::VectorXd lo(2*m),hi(2*m);lo.head(m)=lower;hi.head(m)=upper;lo.tail(m)=-curvature;hi.tail(m)=curvature;
    q=(kernel.transpose()*(q+h*fixed)).eval();h=(kernel.transpose()*h*kernel).eval();
    lo-=a*fixed;hi-=a*fixed;a=(a*kernel).eval();
    Eigen::VectorXd x=Eigen::VectorXd::Zero(kernel.cols());
    // #DISTANCE_DEPENDENT_CURVATURE
    // 기존의 초기 최고 속도 기준 QP가 성공하면 같은 해를 그대로 유지한다.
    // 실패했을 때만 기존 최대 jerk·감속 정지 포락선에서 거리별 속도를 구해
    // 곡률 제안 구간을 넓히고 같은 QP를 한 번 더 푼다. 이를 먼저 적용하면
    // 기존 R182 연결이 실패하는 회귀가 있어 기존 성공 순서를 보존한다.
    // 초기 실측 v/a·C2 등식·도로 corridor·QP 오차는 유지한다. 전방 x와
    // 실제 arc 거리는 다르므로 이 제안 자체를 주행 승인으로 쓰지 않는다.
    // 아래의 전체 형상 검사와 호출부의 새 시간 계획·원본 객체 예측·충돌·
    // 횡가속도·가감속·jerk 검사를 모두 통과해야 발행한다. 반복은 두 번으로
    // 제한하며 GT ID나 특정 구간에 의존하지 않아 LiDAR 추적에도 적용된다.
    if(!solveBoundedQp(h,q,a,lo,hi,x)) {
      PlanningConstraints stop_limits;stop_limits.maximum_speed_mps=0;
      const auto braking=ceilingRecovery(ego,stop_limits);
      const auto reachable_speed=[&](double distance) {
        double station=0;
        for(std::size_t phase_index=0;phase_index<braking.count;++phase_index) {
          const auto& phase=braking.phase[phase_index];
          const auto travelled=[&](double t){return phase.v*t+phase.a*t*t/2+phase.j*t*t*t/6;};
          const double end=station+travelled(phase.h);
          if(distance<=end) {
            double left=0,right=phase.h;
            for(int iteration=0;iteration<45;++iteration) {
              const double middle=(left+right)/2;
              if(station+travelled(middle)<distance)left=middle;else right=middle;
            }
            return std::max(0.0,phase.speed((left+right)/2));
          }
          station=end;
        }
        return 0.0;
      };
      bool expanded=false;
      for(int i=0;i<m;++i) {
        const double speed=reachable_speed(breaks[i]);
        const double limit=std::min(p_.max_curvature_inv_m,p_.max_lateral_accel_mps2/std::max(1e-8,speed*speed));
        const double increase=std::max(0.0,std::max(0.0,limit-4*p_.qp_tolerance)-curvature[i]);
        lo[m+i]-=increase;hi[m+i]+=increase;expanded=expanded || increase>0;
      }
      if(!expanded)return {};
      x.setZero();
      if(!solveBoundedQp(h,q,a,lo,hi,x))return {};
    }
    const Eigen::VectorXd coefficients=fixed+kernel*x;
    std::vector<PathPoint> output;
    for(double station:breaks) {
      const double y=(basis(station,0)*coefficients)[0],dy=(basis(station,1)*coefficients)[0],ddy=(basis(station,2)*coefficients)[0];
      const auto position=Geometry::add(ego.pose.position,Geometry::add(Geometry::mul(tangent,station),Geometry::mul(normal,y)));
      const double progress=station/length*endpoint->progress_m;
      output.push_back({{position,Geometry::wrap(ego.pose.yaw_rad+std::atan(dy))},progress,progress,ddy/std::pow(1+dy*dy,1.5)});
    }
    output.front().pose=ego.pose;output.front().curvature_inv_m=ego.curvature_inv_m;
    output.back()=*endpoint;output.back().time_sec=output.back().progress_m;
    for(auto it=endpoint+1;it!=shape.end();++it){auto point=*it;point.time_sec=point.progress_m;output.push_back(point);}
    const auto origin=c.reference.project(ego.pose.position,0,c.reference.length());
    const Context geometric{c.reference,c.predictions,c.schedule,c.boundary,origin.s,c.prediction_offset,true,&c.boundary_index,c.native_region,c.proposal_timing};
    if(!lateralSlopeClear(geometric,output) || !checkPath(geometric,output))return {};
    return output;
  }
  double followingSpeedTarget(const Context& c,const Pose2& pose,double time,double cap,double ego_speed,double road_yaw) const {
    // #LEAD_VEHICLE_HEADWAY
    // 고정 0.25m 충돌 여유만으로 속도를 최적화하면 느린 앞차의 범퍼까지
    // 접근하는 계획이 선택된다. map 경로상의 각 예정 자세에서 같은 방향의
    // 차량 예측을 조회하고, 실제 차체의 횡 투영이 겹치는 앞차만 추종한다.
    // 뒷차축→앞범퍼와 상대 OBB의 전방 투영을 빼 실제 범퍼 간격을 구한다.
    // 목표속도에는 아래 상대속도 보정을 적용한다. 옆 차선·뒤차·
    // 교차 차량은 이 추종 목표에서 제외하며 원래 시간별 충돌 검사가 담당한다.
    // 이는 기존 속도 비용의 목표값이지 새 물리 속도 제약이 아니다. 따라서
    // 현재 속도를 순간 삭제하지 않고 원래 가감속/jerk 한도에서 감속한다.
    // headway는 추종 여유이며 급제동·모델 오차에 대한 안전 보증은 아니다.
    // ROI는 앞차를 임의 제거하면 안 되며 bbox와 예측 시각 계약을 유지한다.
    // #RELATIVE_GAP_PREVIEW
    // 현재 간격만 쓰면 더 빠르게 멀어지는 앞차에도 제동하고, 느린 앞차에는
    // 여유가 부족해진 뒤 감속한다. 같은 map 예측의 절대 속도를 경로 전방으로
    // 투영하고 후보 자차 속도를 빼, 기존 응답 시간 동안의 간격 변화를 구한다.
    // 이 변화만 기존 headway 목표에 더한다. 벌어지는 간격은 불필요한 감속을
    // 줄이고, 좁아지는 간격은 속도에 비례해 감속 시점을 앞당긴다. 현재 정지
    // 여유 이하의 간격은 이 보정으로 풀지 않는다. 예측·bbox·hard 충돌 검사,
    // 물리 한도와 제어기는 유지하며 GT ID나 검출 원천에 의존하지 않는다.
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
        const auto object=samplePrediction(c.predictions,j,c.prediction_offset+time);
        // LiDAR tracks carry label=unknown. A moving body travelling along
        // our lane is still a lead object; semantic vehicle labels must not
        // be required for ordinary following. Stationary unknown clutter and
        // pedestrians keep their existing obstacle treatment.
        if(object.kind==ObjectKind::Pedestrian ||
           (object.kind!=ObjectKind::Vehicle && Geometry::norm(object.velocity_mps)<.3))continue;
        const double speed=Geometry::norm(object.velocity_mps);
        // A fitted rectangular box can flip by pi without changing its shape.
        // Following direction comes from measured motion when it is moving.
        const auto heading=speed>=.3 ? Geometry::mul(object.velocity_mps,1./speed) :
            Geometry::unit(object.pose.yaw_rad);
        objects.push_back({Geometry::objectBox(object,p_.obstacle_margin_m),
                           heading,object.velocity_mps});
      }
      found=c.following_objects.emplace(time,std::move(objects)).first;
    }
    // #LOW_SPEED_FOLLOWING
    // 저속에서 일정한 1.5초 목표를 강제하면 동일 속도의 앞차 뒤에서도
    // 여유를 다시 벌리기 위한 감속이 반복된다. 후보 자차 속도가 0→설정
    // 전환 속도로 증가할 때 저속 headway→기존 headway를 연속 보간한다.
    // 전환 속도 이상은 기존 식과 같다. map 예측·범퍼 간격·상대속도 응답은
    // 그대로 사용하며 2m 정지 여유, swept 충돌, 가감속/jerk 검사는 줄이지 않는다.
    // 검출 원천이나 객체 ID에 의존하지 않는다. ROI 통합 시 앞차/예측을
    // 누락시키지 말고 실제 bbox·속도·시각 계약을 유지해야 한다.
    const double headway=Geometry::lerp(p_.following_low_speed_headway_sec>0 ?
        p_.following_low_speed_headway_sec : p_.following_time_headway_sec,
        p_.following_time_headway_sec,Geometry::clamp(ego_speed/p_.following_headway_transition_mps,0,1));
    // #ROAD_RELATIVE_SPEED
    // 같은 map 참조선 접선으로 두 차량의 속도를 투영한다. 차선 변경 중
    // 자차 경로 yaw가 기울어도 옆 차 절대속도의 크기로 추월 여부를 판단하지
    // 않는다. 실제 회전 차체의 횡 중첩과 앞범퍼 투영을 사용하고, 도로축의
    // 목표는 경로축 속도로 환산해 같은 속도 비용·충돌 검사에 전달한다.
    const auto forward=Geometry::unit(road_yaw),left=Geometry::normal(road_yaw);
    const double along=Geometry::dot(Geometry::unit(pose.yaw_rad),forward);
    if(along<=.5)return cap;
    const auto own_box=Geometry::egoBox(pose,p_.vehicle);
    const double front=Geometry::dot(Geometry::sub(own_box.pose.position,pose.position),forward)+
        Geometry::extent(own_box,forward);
    const double road_speed=ego_speed*along;
    for(const auto& object:found->second) {
      if(Geometry::dot(forward,object.heading)<=.5) continue;
      const auto delta=Geometry::sub(object.box.pose.position,pose.position);
      const double longitudinal=Geometry::dot(delta,forward);
      if(longitudinal<=0) continue;
      const auto& box=object.box;
      if(std::abs(Geometry::dot(Geometry::sub(box.pose.position,own_box.pose.position),left))>
         Geometry::extent(own_box,left)+Geometry::extent(box,left)) continue;
      const double gap=longitudinal-front-Geometry::extent(box,forward);
      if(gap<=p_.following_min_gap_m) { cap=0; continue; }
      const double lead_speed=Geometry::dot(object.velocity,forward);
      const double gap_target=std::max(0.0,(gap-p_.following_min_gap_m+
          (lead_speed-road_speed)*p_.speed_limit_response_sec)/headway);
      // #SEPARATING_LEAD_SPEED
      // 차체가 분리되어 있고 앞차가 더 빨라 간격이 벌어질 때에는 현재의
      // 짧은 간격만으로 목표속도를 낮추지 않는다. 그 앞차 속도까지의 가속을
      // 비용 목표로 허용하며 후보의 실제 가속·jerk와 전 시간 충돌 검사는
      // 그대로 적용한다. 상대속도가 닫히면 기존 간격 목표로 돌아가므로
      // 빠른 앞차를 관측했다는 이유로 뒤따라 충돌하는 후보를 허용하지 않는다.
      const double target=lead_speed>road_speed ? std::max(gap_target,lead_speed) : gap_target;
      cap=std::min(cap,target/along);
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
      const double following_target=missionSpeedTarget(q.progress_m,followingSpeedTarget(c,q.pose,edge.t0+t,cap,edge.speed(t),r.yaw));
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

  // #MEASURED_BRAKING_RELEASE
  // IMU가 전달한 실제 종가속도는 map 경로를 따라가는 뒷차축의 초기 상태다.
  // 실제 제동이 설정 감속 한도를 넘은 순간도 값을 잘라내지 않고 보존한다.
  // 이미 발생한 초과 구간에서만 최대 양의 jerk로 즉시 제동을 해제한
  // a(t)=a0+J*t를 하한으로 사용한다. |jerk|<=J와 초기 상태 일치 검사 때문에
  // 초과 제동을 유지하거나 더 강하게 만드는 후보는 허용되지 않는다.
  // -max_deceleration에 도달한 뒤에는 원래 하한을 그대로 적용한다.
  // 같은 해제 구간을 속도 탐색 시간 격자·정지 거리·속도 상한 복귀에도
  // 반영한다. 도로 전체 차체·예측 충돌·정지선·횡가속도와 가감속/jerk
  // 최종 검사는 유지하며, 해제 전에 역주행이 불가피하면 경로를 내지 않는다.
  // 새 감속 한도나 제어 명령을 추가하지 않으며 GT/LiDAR·ROI와 무관하다.
  // Preserve measured acceleration above the configured comfort bound and
  // return to that bound at maximum negative jerk; never clip the initial state.
  double maximumPlannedAcceleration(const EgoState& ego,double time) const {
    return std::max(p_.max_acceleration_mps2,
                    ego.acceleration_mps2-p_.max_longitudinal_jerk_mps3*time);
  }
  double minimumPlannedAcceleration(const EgoState& ego,double time) const {
    return std::min(-p_.max_deceleration_mps2,
                    ego.acceleration_mps2+p_.max_longitudinal_jerk_mps3*time);
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
    auto acceleration_times=edge.extrema();
    if(ego.acceleration_mps2<-p_.max_deceleration_mps2) {
      const auto add=[&](double t){if(t>0 && t<edge.h)acceleration_times.push_back(t);};
      add((-p_.max_deceleration_mps2-ego.acceleration_mps2)/p_.max_longitudinal_jerk_mps3-edge.t0);
      if(std::abs(edge.c3())>1e-12)
        add((p_.max_longitudinal_jerk_mps3-2*edge.c2())/(6*edge.c3()));
    }
    for(double t:acceleration_times) {
      const double v=edge.speed(t),a=edge.acceleration(t),j=edge.jerk(t);
      if(!std::isfinite(v) || !std::isfinite(a) || !std::isfinite(j) || v<-1e-8 ||
         v>ceiling+1e-8 || a>maximumPlannedAcceleration(ego,edge.t0+t)+1e-8 ||
         a<minimumPlannedAcceleration(ego,edge.t0+t)-1e-8 || std::abs(j)>p_.max_longitudinal_jerk_mps3+1e-8 ||
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
      for(int k=1;k<=count;++k)
        refined.push_back(k==count ? times[i] : times[i-1]+span*k/count);
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
    // #PREDICTION_SWEEP_REUSE
    // 같은 속도 layer의 모든 후보는 동일한 예측 시간 구간을 검사한다.
    // 객체 중심의 시간 구간 바깥 범위만 Context 안에 재사용한다. 후보별 자차
    // 위치·회전·속도 보간 여유와 정밀 SAT는 그대로이며 다음 관측과 공유하지 않는다.
    const std::vector<ObjectBounds>* bounds=&c.object_bounds;
    const double horizon=c.predictions.step_sec*kPredictionSteps;
    if(std::isfinite(time0) && std::isfinite(time1) && time0>=0 && time1>=time0 && time1<=horizon) {
      const auto key=std::make_pair(time0,time1);
      auto found=c.temporal_bounds.find(key);
      if(found==c.temporal_bounds.end()) {
        auto objects=c.object_bounds;
        for(std::size_t j=0;j<objects.size();++j) {
          const auto center=[&](double time) {
            const double scaled=time/c.predictions.step_sec;
            const auto lo=std::min(static_cast<std::size_t>(scaled),kPredictionSteps-1);
            const auto& a=(lo==0?c.predictions.current:c.predictions.future[lo-1])[j];
            const auto& b=c.predictions.future[lo][j];
            return Geometry::lerp(a.pose.position,b.pose.position,scaled-lo);
          };
          auto& box=objects[j];box.centers=Geometry::Bounds{};
          box.centers.include(center(time0));box.centers.include(center(time1));
          for(std::size_t k=1;k<kPredictionSteps;++k) {
            const double time=k*c.predictions.step_sec;
            if(time>time0 && time<time1)box.centers.include(c.predictions.future[k-1][j].pose.position);
          }
        }
        found=c.temporal_bounds.emplace(key,std::move(objects)).first;
      }
      bounds=&found->second;
    }
    std::vector<std::size_t> indices;const double radius=egoCollisionRadius(motion_pad,angle);
    for(std::size_t j=0;j<c.object_bounds.size();++j) {
      if(!potentialObjectCollision(c.object_bounds[j],rear_axle,radius))continue;
      const auto& bounded=(*bounds)[j];
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
    if(c.supplied_geometry)return true;
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
      // 실제 차체가 이미 밖이면 초기 침범 면적을 상한으로 유지한다.
      // 복귀 중 면적이 잠시 감소했다는 이유로 더 작은 새 상한을 만들지 않는다.
      // 초기 전방 통로와 완전 복귀 후의 원래 도로 검사는 계속 적용한다.
      // 객체/동역학/차선 방향 검사는 생략하지 않으며 GT 전용 정보가 없다.
      const bool margin_only=c.native_region->margin_only_recovery;
      double previous=c.native_region->reentry_outside_limit_m2;
      bool returned=false;
      for(std::size_t i=0;i<path.size();++i) {
        const auto full_margin=Geometry::sweptEgoEnvelope(path[i].pose,path[i].pose,p_.vehicle,p_.road_margin_m);
        const auto box=margin_only ? Geometry::egoBox(path[i].pose,p_.vehicle) : full_margin;
        const double outside=margin_only && physical_proven ? 0 : c.native_region->reentryOutsideArea(box);
        if(outside>previous+1e-7)return false;
        if(margin_only)previous=outside;
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
    const auto candidates=[&](double lo,double hi) {
      const double begin=edge.s0+edge.distance(lo),end=edge.s0+edge.distance(hi);
      const double span=hi-lo;
      const double radius=std::hypot(std::max(p_.vehicle.front_m,p_.vehicle.rear_m),p_.vehicle.width_m/2);
      // 각 시간 구간의 각도/거리 비는 선형 arc의 상한과 반올림 오차로 감싼다.
      // 기존 d>1e-12 분기와 같은 최소 분모를 사용한다. 큰 좌표의 오차는
      // 범위를 넓혀 공간 선별의 이득만 줄이며, 충돌 검사 범위를 줄이지 않는다.
      const double scale=std::max({1.0,std::abs(arc.bounds.min_x),std::abs(arc.bounds.min_y),
                                  std::abs(arc.bounds.max_x),std::abs(arc.bounds.max_y)});
      const double error=128*std::numeric_limits<double>::epsilon();
      // #LOCAL_SWEEP_ROTATION_BOUND
      // 짧은 시간 edge에 150m 뒤 커브의 회전율을 적용하면 직선 옆차도 정밀
      // 검사 대상으로 팽창해 경로가 제어 입력 수명보다 늦어진다. 비음수 속도일
      // 때 실제 이동 arc 구간의 yaw/거리 비와 yaw 총변동 상한을 사용한다.
      // headingBounds()는 그 사이 모든 knot 구간을 포함한다. 부호가 바뀌는 이동은
      // 기존 전체 상한을 쓰고, 반올림·객체 회전 여유와 최종 swept SAT는 유지한다.
      const auto heading=min_speed>=0 ? arc.headingBounds(begin,end) :
          std::make_pair(arc.heading_per_meter,Geometry::kPi);
      const double heading_rate=heading.first;
      const double heading_bound=heading_rate+
          (heading_rate*error*scale+error*arc.yaw_scale)/1e-12;
      const double motion_bound=max_acceleration*span*span/8*(1+radius*heading_bound);
      // #SPEED_EDGE_BOUNDS
      // 짧은 속도 edge와 관계없는 전체 경로를 사용하면 멀리 있는 장애물 때문에도
      // 매 후보마다 정밀 SAT가 반복된다. 비음수 속도 edge의 실제 arc 시작·끝과
      // 그 사이 모든 꺾임점을 포함한다. 선형 arc 보간은 이 AABB 밖으로 나가지
      // 않으며 곡선 중간 돌출도 빠지지 않는다. 부호가 바뀌는 비단조 구간은 기존
      // 전체 범위를 유지한다. 객체 예측, 회전 팽창 및 정밀 시간 샘플은 유지한다.
      Geometry::Bounds edge_bounds=arc.bounds;
      if(min_speed>=0) {
        edge_bounds=Geometry::Bounds{};
        edge_bounds.include(arc.at(begin).pose.position);
        edge_bounds.include(arc.at(end).pose.position);
        auto knot=std::upper_bound(arc.station.begin(),arc.station.end(),begin);
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
      const double angle=std::min(Geometry::kPi,heading.second+error*arc.yaw_scale+1e-10);
      return collisionObjectIndices(c,edge_bounds,motion_bound,min_speed>=0?angle:Geometry::kPi,
          c.prediction_offset+edge.t0+lo,c.prediction_offset+edge.t0+hi,arc.at(begin).pose.yaw_rad);
    };
    auto indices=candidates(0,edge.h);
    if(indices.empty())return true;
    // #COLLIDING_ENDPOINT_PRUNE
    // edge 끝에서 원래 OBB 검사상 이미 충돌하면 중간 knot의 시간 역산을
    // 먼저 할 필요가 없다. 끝점은 기존 연속 sweep에 포함되므로 이 후보만
    // 조기 거부한다. 끝점이 비충돌이라는 이유로 통과시키지 않으며, 나머지는
    // 동일한 시간 분할·불확실성·swept SAT로 전체 이동을 계속 검사한다.
    auto last=arc.at(edge.s0+edge.distance(edge.h));last.time_sec=edge.t0+edge.h;
    if(collisionNarrow(c,last,last,0,&indices))return false;
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

  // #SPEED_DEPENDENT_OBSTACLE_APPROACH
  // map 관측의 접근 판단은 일반도로 기준 속도에서 설정한 거리(시험값 40m)를
  // 현재 속도에 비례시킨다. 다만 기존 근거리 공간 창 안의 보행자 접근과
  // 실측 v/a에서 jerk 한도로 정지할 거리, 기존 응답 시간 동안의 이동 및
  // 뒷차축→앞범퍼/객체 여유보다 짧아지지 않게 한다. 가속 중이면 정지거리도
  // 커진다. 이는 제동 모델의 시험 여유이며 실제 모든 상황의 정지를 보장하지 않는다.
  // 보행자 접근 상한에만 사용한다. 시작한 접근 범위는 같은 ID에 보존해
  // 감속 자체로 범위가 줄어 가속/제동을 반복하지 않게 한다. 객체가 뒤로
  // 지나가거나 관측에서 없어지거나 범위를 실제 벗어나면 기존처럼 해제한다.
  // 객체/예측/ROI를 자르지 않으며 거리 밖에서도 원래 swept 충돌 검사와
  // 매 상태의 남은 거리 대비 정지 가능성 검사는 그대로 수행한다.
  double approachDistance(const EgoState& ego) const {
    const double reference_speed=p_.normal_speed_limit_mps>0 ?
        p_.normal_speed_limit_mps : p_.max_speed_mps;
    // #FINITE_APPROACH_DISTANCE
    // brakingDistance의 무한대는 현재 v/a에서 jerk를 지키며 전진 정지가
    // 불가능하다는 계산 결과다. 공간 감지 반경의 무한대를 뜻하지 않는다.
    // 이를 객체별 접근 거리로 저장하면 먼 보행자가 사라질 때까지 10km/h
    // 상한이 남는다. 정상 정지 거리는 그대로 사용하고, 정지 해가 없을 때만
    // 기존 공간 창·속도 비례 거리·응답 이동 거리로 유한한 감지 범위를 정한다.
    // 속도 계획의 정지 가능성·jerk·충돌 검사는 별개로 그대로 수행하므로
    // 불가능한 초기 상태의 경로를 통과시키거나 관측 객체를 제거하지 않는다.
    const double stopping_distance=brakingDistance(ego.speed_mps,ego.acceleration_mps2);
    const double preview=ego.speed_mps*p_.speed_limit_response_sec+
        p_.vehicle.front_m+p_.obstacle_margin_m+
        (std::isfinite(stopping_distance) ? stopping_distance : 0.0);
    return std::max({p_.horizon_m,p_.pedestrian_caution_distance_m*ego.speed_mps/reference_speed,preview});
  }

  // Minimum jerk-limited stopping distance on this fixed arc. Ramp to negative
  // peak acceleration, optionally hold -D, then ramp to a=0 at v=0. States
  // below v=a²/(2J) with a<0 cannot reach rest without reversing in this model.
  double brakingDistance(double v,double a) const {
    const double j=p_.max_longitudinal_jerk_mps3,d=p_.max_deceleration_mps2;
    if(a<0 && v<a*a/(2*j)-1e-8) return std::numeric_limits<double>::infinity();
    if(a<-d) {
      const double t=(-d-a)/j;
      const double distance=v*t+a*t*t/2+j*t*t*t/6;
      return distance+brakingDistance(v+a*t+j*t*t/2,-d);
    }
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
  bool staticSweepOverlap(Pose2 a,Pose2 b,const Geometry::Box& obstacle,int depth=0) const {
    const auto envelope=Geometry::sweptEgoEnvelope(a,b,p_.vehicle,0);
    if(!Geometry::sweptOverlap(envelope,envelope,obstacle,obstacle))return false;
    const auto first=Geometry::egoBox(a,p_.vehicle),last=Geometry::egoBox(b,p_.vehicle);
    if(Geometry::sweptOverlap(first,first,obstacle,obstacle) ||
       Geometry::sweptOverlap(last,last,obstacle,obstacle))return true;
    if(depth>=16)return true;
    const Pose2 middle{Geometry::lerp(a.position,b.position,.5),Geometry::yawLerp(a.yaw_rad,b.yaw_rad,.5)};
    return staticSweepOverlap(a,middle,obstacle,depth+1) ||
           staticSweepOverlap(middle,b,obstacle,depth+1);
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
        const auto box=Geometry::objectBox(inflated,obstacleMargin(inflated.id,0));
        return p_.recover_static_obstacle_margin ? staticSweepOverlap(a,b,box) :
            Geometry::sweptOverlap(envelope,envelope,box,box);
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
                                 const std::vector<ReferencePoint>& reference_map,std::size_t beam_width,bool& truncated) const {
    const bool interacting=potentialCollision(c,arc.bounds,0);
    // Empty or spatially disjoint scenes do not need an obstacle-search cap.
    // Cutting these cheap retimings off discarded valid free-road paths.
    const bool physical_reentry=c.native_region && c.native_region->recovery_original &&
        !c.native_region->margin_only_recovery;
    auto start=arc.points.front(); start.time_sec=0;
    if(!roadClear(c,start.pose,start.pose)){failure_reason_="initial-road";return {};}
    if(collision(c,start,start)){failure_reason_="initial-collision";return {};}
    // #FREE_ROAD_SEARCH_INVARIANCE
    // GT의 먼 객체가 존재한다는 이유만으로 빈 도로의 속도 탐색을 줄이지 않는다.
    // 전체 경로·예측 swept 범위가 기존 보수적 선별에서도 분리되면 원래 beam을
    // 유지한다. 객체는 삭제하지 않으며 LiDAR도 같은 입력/충돌 계약을 따른다.
    const std::size_t active_beam_width=beam_width<p_.speed_beam_width &&
        !interacting ? p_.speed_beam_width : beam_width;
    const double cap=std::min(p_.max_speed_mps,limits.maximum_speed_mps);

    const double origin_s=c.reference.project(start.pose.position,0,c.reference.length()).s;
    const auto recovery=ceilingRecovery(ego,limits);
    // #STATIC_STANDSTILL_MARGIN
    // 정적 객체의 실제 bbox 앞 충돌 여유만 남기던 정지 위치에도 기존 앞차
    // 추종의 정지 여유를 적용한다. 분류·GT ID와 관계없이 정지한 예측을 쓴다.
    // 기존 jerk/가감속 한도로 그 추가 여유를 확보할 수 있을 때만 정지 위치를
    // 앞당긴다. 이미 가까운 객체나 정지 상태에서는 원래 정지 공간을 보존해
    // 추가 여유 때문에 가능했던 정지 경로가 사라지지 않게 한다. 회피 형상과
    // 속도 비용 목표는 바꾸지 않으며 정지선·전체 차체 충돌 검사는 유지한다.
    const double static_stop=staticStopArc(c,arc);
    const double buffered_static_stop=static_stop-p_.following_min_gap_m>=
        brakingDistance(ego.speed_mps,ego.acceleration_mps2) ?
        static_stop-p_.following_min_gap_m : static_stop;
    const double persistent_stop=std::min(buffered_static_stop,
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
    if(available_space<0 || brakingDistance(ego.speed_mps,ego.acceleration_mps2)>available_space){failure_reason_="stopping-space";++rejected_[2];return {};}
    // Reject impossible initial/terminal geometry before charging the expensive
    // speed-search budget. A failed distance check did not run a speed search.
    if((interacting || physical_reentry) &&
       ++speed_searches_>p_.maximum_speed_searches*(extended_search_?3:1) &&
       p_.maximum_speed_searches){++rejected_[0];return {};}
    // A static obstacle constrains position and braking reserve, not the
    // arbitrary end time of this prediction window. Every state still keeps
    // s + brakingDistance <= available_space and passes swept collision checks.
    // Explicit stop requests and reachable persistent stop lines retain waits.
    const bool finish_stopped=cap==0 ||
        (std::isfinite(stop_arc) && limits.release_time_sec>=duration &&
         stop_arc<std::max(ego.speed_mps*duration,p_.following_min_gap_m));
    std::vector<double> clock{0};
    for(double t=p_.speed_time_step_sec;t<duration-1e-10;t+=p_.speed_time_step_sec) clock.push_back(t);
    // Prioritize analytic events over almost coincident regular samples.
    // Absolute-time subtraction on ~1e-9 s Hermite intervals amplifies ulp
    // error into spurious acceleration/jerk during the independent recheck.
    std::vector<double> events;
    const auto event=[&](double t) { if(t>0 && t<duration) events.push_back(t); };
    const double measured_brake_release_sec=ego.acceleration_mps2<-p_.max_deceleration_mps2 ?
        (-p_.max_deceleration_mps2-ego.acceleration_mps2)/p_.max_longitudinal_jerk_mps3 : 0;
    if(measured_brake_release_sec>0)event(measured_brake_release_sec);
    const double measured_acceleration_release_sec=std::max(0.,
        (ego.acceleration_mps2-p_.max_acceleration_mps2)/p_.max_longitudinal_jerk_mps3);
    event(measured_acceleration_release_sec);
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
    // #BRAKING_EVENT_CLOCK
    // 실제 초과 제동을 해제하는 동안 a(t)=a0+J*t가 강제되므로 정규 격자는
    // 새로운 제어 선택을 추가하지 않는다. 해제 경계가 0.500254초일 때
    // 0.5초 정규 점도 남기면 0.254ms Hermite 구간에서 반올림만으로
    // jerk 4가 4.000000025가 되어 가능한 전진 경로를 전부 거부했다.
    // 기존 출력 knot의 1ms 안정화 범위와 같이, 해제 경계 직전의 불필요한
    // 정규 점만 생략한다. 작은 정규 간격에는 그 간격의 1/4까지만 쓴다.
    // 실제 해제·정지·상한 전환 등 모든 해석적 이벤트와 초기 상태·전체
    // 예측 시간은 보존한다. 원래 jerk 허용 오차나 가감속 한도는 바꾸지
    // 않고 새 다항식도 전체 차체/도로·예측 충돌·속도·jerk로 재검증한다.
    // 역주행이 불가피한 상태는 여전히 거부하며 GT/LiDAR·ROI와 무관하다.
    clock.erase(std::remove_if(clock.begin(),clock.end(),[&](double t) {
      if(t<=0)return false;
      if(t<measured_brake_release_sec &&
         measured_brake_release_sec-t<std::min(.001,p_.speed_time_step_sec/4))return true;
      return std::any_of(events.begin(),events.end(),[&](double q){return std::abs(t-q)<std::min(.001,p_.speed_time_step_sec/4);});
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
    // #REENTRY_INTERVAL_GEOMETRY_REUSE
    // 실제 차체는 도로 안에 있고 계획 여유만 소모된 경우, 같은 ArcPath를
    // 이동하는 속도 후보마다 원래 map 도로의 포함 검사를 반복하지 않는다.
    // 각 공간 구간의 연속 차체 sweep이 road_margin_m까지 포함됨을 먼저
    // 원래 검사로 증명하고, 빈틈 없이 이어진 검증 구간만 합친다. 속도의
    // 모든 극값이 비음수이고 이동 전체가 그 안에 있을 때만 도로 복귀 검사가
    // 자명하게 성립한다. 나머지 이동과 실제 도로 밖의 초기 상태는 기존
    // 후보별 검사를 유지한다. 이 증명은 현재 planSpeed 호출 안에서만 쓰며,
    // 객체 예측 시각에 따른 collisionEdgeClear와 최종 전체 경로의 복귀·
    // 차체·충돌·가감속·jerk 검사를 계속 수행한다. 경계/여유/시각 한도를
    // 바꾸지 않고, GT·구간 ID 없이 같은 map/ROI 계약의 LiDAR에도 적용한다.
    std::vector<std::pair<double,double>> reentry_clear_intervals;
    if(c.native_region && c.native_region->margin_only_recovery) {
      const auto& original=*c.native_region->recovery_original;
      for(std::size_t i=1;i<arc.points.size();++i) {
        if(!roadClearWithMargin(c,arc.points[i-1].pose,arc.points[i].pose,
                                p_.road_margin_m,0,&original))continue;
        if(!reentry_clear_intervals.empty() && reentry_clear_intervals.back().second==arc.station[i-1])
          reentry_clear_intervals.back().second=arc.station[i];
        else reentry_clear_intervals.emplace_back(arc.station[i-1],arc.station[i]);
      }
    }
    // Geometry is shared by every speed candidate. Precheck a dense spatial
    // return once, then defer duplicate prefix tests to checkPlannedPath's
    // unchanged exact output-sample checks. Nothing bypasses final validation.
    bool reentry_shape_prechecked=false;
    if(physical_reentry) {
      const auto& original=*c.native_region->recovery_original;
      std::vector<PathPoint> prefix;double return_station=-1;
      const double spacing=std::min(p_.output_step_m,.05);
      for(double station=0;station<=arc.length();station+=spacing) {
        const auto point=arc.at(station);prefix.push_back(point);
        if(original.containsBox(Geometry::sweptEgoEnvelope(
              point.pose,point.pose,p_.vehicle,p_.road_margin_m))) {
          return_station=station;break;
        }
        if(point.progress_m>p_.horizon_m)break;
      }
      if(return_station>=0 && reentryProgressClear(c,prefix,true)) {
        reentry_shape_prechecked=true;auto previous=prefix.back().pose;
        for(std::size_t i=1;i<arc.points.size() && reentry_shape_prechecked;++i) {
          if(arc.station[i]<=return_station)continue;
          reentry_shape_prechecked=roadClearWithMargin(c,previous,arc.points[i].pose,
              p_.road_margin_m,0,&original);
          previous=arc.points[i].pose;
        }
      }
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
        const double a_lo=std::max(minimumPlannedAcceleration(ego,clock[layer]),q.a-jerk*dt);
        const double a_hi=std::min(maximumPlannedAcceleration(ego,clock[layer]),q.a+jerk*dt);
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
        const double preferred_acceleration=p_.preferred_lateral_accel_mps2>0 ?
            p_.preferred_lateral_accel_mps2 : p_.max_lateral_accel_mps2;
        const double preferred_curve_cap=nearby_curvature>1e-12 ?
            std::min(cap,std::sqrt(preferred_acceleration/nearby_curvature)) : cap;
        for(double a:accelerations) {
          // Endpoint jerk of cubic Hermite v(t) is affine in v1. Intersect
          // both exact jerk intervals BEFORE allocating/checking an arc edge.
          const double lo=std::max(0.0,q.v+std::max((4*q.a+2*a)*dt-jerk*dt*dt,
                                                   (2*q.a+4*a)*dt-jerk*dt*dt)/6);
          const double hi=std::min(recovery_peak,q.v+std::min((4*q.a+2*a)*dt+jerk*dt*dt,
                                                                 (2*q.a+4*a)*dt+jerk*dt*dt)/6);
          if(lo>hi+1e-12){++rejected_[3];continue;}
          const double upper=std::max(lo,hi);
          std::vector<double> velocities{lo,upper};
          for(double v : {q.v+(q.a+a)*dt/2,0.0,cap,q.v,curve_cap,preferred_curve_cap,missionSpeedTarget(q.s,cap)})
            velocities.push_back(Geometry::clamp(v,lo,upper));
          std::sort(velocities.begin(),velocities.end());
          velocities.erase(std::unique(velocities.begin(),velocities.end()),velocities.end());
          for(double v:velocities) {
            if(v<0 || v>recovery_peak) continue;
            const SpeedEdge edge{q.s,q.v,q.a,v,a,clock[layer-1],dt};
            // Interior acceleration/speed and lowered-ceiling recovery still
            // require the full polynomial check. Do cheap dynamics before
            // curvature traversal, temporal footprint sampling or SAT.
            if(!speedAdmissible(edge,ego,limits,0,recovery)){++rejected_[4];continue;}
            const double s=q.s+edge.distance(dt);
            if(!std::isfinite(s) || s<q.s-1e-9 || s>arc.length()+1e-9 ||
               s+brakingDistance(v,a)>available_space ||
               (clock[layer-1]<limits.release_time_sec &&
                q.s+edge.distance(std::min(dt,limits.release_time_sec-clock[layer-1]))>stop_arc+1e-9)){++rejected_[2];continue;}
            const double curvature=arc.curvature(q.s,s);
            if(curvature>p_.max_curvature_inv_m+1e-6){++rejected_[5];continue;}
            bool lateral_clear=true,forward_only=true;
            for(double t:edge.extrema()) {
              const double speed=edge.speed(t);
              if(speed<0)forward_only=false;
              if(std::pow(speed,2)*curvature>p_.max_lateral_accel_mps2+1e-6) lateral_clear=false;
            }
            if(!lateral_clear && !lateralAccelerationClear(arc,edge,ego,recovery)){++rejected_[6];continue;}
            const bool reentry_proven=forward_only && s>=q.s &&
                (reentry_shape_prechecked ||
                 std::any_of(reentry_clear_intervals.begin(),reentry_clear_intervals.end(),
                    [&](const std::pair<double,double>& interval) {return q.s>=interval.first && s<=interval.second;}));
            if(!(reentry_proven ? collisionEdgeClear(c,arc,edge) : edgeClear(c,arc,edge,physical_proven))){++rejected_[7];continue;}
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
      if(next.empty()){failed_layer_=clock[layer];failure_reason_="speed-search";return {};}
      pruneSpeedLayer(next,active_beam_width,truncated);
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
      if(checkPlannedPath(ego,reference_map,c.predictions,limits,out,arc.supplied_geometry)) {
        auto spatial=std::make_shared<std::vector<Pose2>>();spatial->reserve(arc.points.size());
        for(const auto& point:arc.points)spatial->push_back(point.pose);
        out.front().spatial_reference=arc.points.front().spatial_reference ?
            arc.points.front().spatial_reference : std::move(spatial);
        selected_extent_=arc.length();selected_curvature_=arc.curvature(0,arc.length());
        if(out.back().speed_mps==0)validated_stop_=out;
        return out;
      }
    }
    return {};
  }
  void validateParameters() const {
    for(double x:{p_.lane_change_confirmation_sec,p_.lane_change_observation_gap_sec,
                  p_.lane_change_heading_tolerance_rad,p_.lane_change_steering_tolerance_rad})
      if(!std::isfinite(x) || x<=0)throw std::invalid_argument("invalid lane change confirmation parameter");
    if(p_.lane_change_observation_gap_sec>p_.lane_change_confirmation_sec ||
       p_.lane_change_heading_tolerance_rad>=p_.max_initial_heading_error_rad ||
       p_.lane_change_steering_tolerance_rad>=1)
      throw std::invalid_argument("inconsistent lane change confirmation parameter");

    for (double x : {p_.vehicle.front_m,p_.vehicle.rear_m,p_.vehicle.width_m,p_.vehicle.wheelbase_m,p_.horizon_m,p_.dp_step_m,
                    p_.lateral_step_m,p_.dp_slope_sample,p_.output_step_m,p_.corridor_half_width_m,p_.max_curvature_inv_m,
                    p_.max_lateral_accel_mps2,p_.max_speed_mps,p_.max_lateral_slope,p_.max_initial_heading_error_rad,
                    p_.max_initial_distance_m,p_.qp_rho,p_.qp_tolerance,p_.min_horizon_m,p_.road_boundary_step_m,
                    p_.max_acceleration_mps2,p_.max_deceleration_mps2,p_.max_longitudinal_jerk_mps3,
                    p_.speed_time_step_sec,p_.trajectory_output_step_sec,p_.trajectory_horizon_m,
                    p_.following_time_headway_sec,p_.following_headway_transition_mps})
      if (!std::isfinite(x) || x<=0.0) throw std::invalid_argument("nonpositive/nonfinite path parameter");
    for (double x : {p_.obstacle_margin_m,p_.road_margin_m,p_.reference_weight,p_.dp_tracking_weight,
                    p_.slope_weight,p_.bending_weight,p_.jerk_weight,p_.previous_weight,
                    p_.speed_tracking_weight,p_.acceleration_weight,p_.longitudinal_jerk_weight,
                    p_.following_min_gap_m,p_.following_low_speed_headway_sec,p_.preferred_lateral_accel_mps2,
                    p_.pedestrian_caution_speed_mps,p_.pedestrian_caution_distance_m,
                    p_.pedestrian_caution_lateral_margin_m,p_.pedestrian_near_preferred_speed_mps})
      if (!std::isfinite(x) || x<0.0) throw std::invalid_argument("negative/nonfinite path weight or margin");
    if(!std::isfinite(p_.preferred_reference_piece_m) || p_.preferred_reference_piece_m<0 ||
       (p_.preferred_reference_piece_m>0 && p_.preferred_reference_piece_m<2) || p_.preferred_reference_piece_m>25)
      throw std::invalid_argument("invalid preferred reference piece length");
    if(!std::isfinite(p_.reentry_response_sec) || p_.reentry_response_sec<0 || p_.reentry_response_sec>.3 ||
       !std::isfinite(p_.reentry_minimum_speed_mps) || p_.reentry_minimum_speed_mps<0 || p_.reentry_minimum_speed_mps>3)
      throw std::invalid_argument("invalid measured reentry response interval");
    if (p_.following_low_speed_headway_sec>p_.following_time_headway_sec ||
        p_.preferred_lateral_accel_mps2>p_.max_lateral_accel_mps2 ||
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
    if(path.empty())return 0;
    const auto target=ref.at(s);
    const auto direction=Geometry::unit(target.yaw),normal=Geometry::normal(target.yaw);
    double best=1.,lateral=0;
    const auto consider=[&](const Pose2& a,const Pose2& b) {
      const auto da=Geometry::sub(a.position,target.position),db=Geometry::sub(b.position,target.position);
      const double x0=Geometry::dot(da,direction),x1=Geometry::dot(db,direction);
      const double u=std::abs(x1-x0)>1e-10 ? Geometry::clamp(-x0/(x1-x0),0.,1.) : 0.;
      const auto pose=Pose2{Geometry::lerp(a.position,b.position,u),Geometry::yawLerp(a.yaw_rad,b.yaw_rad,u)};
      const auto d=Geometry::sub(pose.position,target.position);
      const double along=std::abs(Geometry::dot(d,direction));
      if(along<best && std::abs(Geometry::wrap(pose.yaw_rad-target.yaw))<.8) {
        best=along;lateral=Geometry::dot(d,normal);
      }
    };
    // Warm-start from the complete searched geometry. The five-second speed
    // window can end before avoidance/return finishes and is not its geometry.
    if(path.front().spatial_reference && path.front().spatial_reference->size()>1) {
      const auto& spatial=*path.front().spatial_reference;
      for(std::size_t i=1;i<spatial.size();++i)consider(spatial[i-1],spatial[i]);
    } else {
      consider(path.front().pose,path.front().pose);
      for(std::size_t i=1;i<path.size();++i)consider(path[i-1].pose,path[i].pose);
    }
    return lateral;
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
    if(path.empty())return true;
    const auto initial=c.reference.at(c.start_s+path.front().progress_m);
    const double l0=Geometry::dot(Geometry::sub(path.front().pose.position,initial.position),
                                Geometry::normal(initial.yaw));
    const double error=Geometry::wrap(path.front().pose.yaw_rad-initial.yaw);
    if(std::abs(error)>=Geometry::kPi/2)return false;
    // A bound on newly planned lateral motion cannot require the measured
    // initial tangent to change instantaneously. An existing excess must
    // monotonically recover; it cannot be increased or carried to the end.
    double allowed=std::max(p_.max_lateral_slope,
        std::abs(initial.metric*(1-initial.curvature*l0)*std::tan(error)));
    double previous=0;
    for(std::size_t i=0;i<path.size();++i) {
      const auto r=c.reference.at(c.start_s+path[i].progress_m);
      const double offset=Geometry::dot(Geometry::sub(path[i].pose.position,r.position),Geometry::normal(r.yaw));
      if(i>0) {
        const double step=path[i].progress_m-path[i-1].progress_m;
        if(step<=0)return false;
        const double slope=std::abs(offset-previous)/step;
        if(slope>allowed+2*p_.qp_tolerance)return false;
        allowed=std::max(p_.max_lateral_slope,slope);
      }
      previous=offset;
    }
    return allowed<=p_.max_lateral_slope+2*p_.qp_tolerance;
  }
  bool roadContainsBox(const Context& c,const Geometry::Box& box) const {
    // 실제 map polygon이 있으면 중심선의 자기 교차로 생긴 가상 경계로
    // 차체를 다시 거부하지 않는다. 두 경로 모두 전체 차체 포함을 검사한다.
    return c.native_region ? c.native_region->containsBox(box) :
                             c.boundary_index.containsBox(box);
  }
  bool roadClearWithMargin(const Context& c,Pose2 first,Pose2 last,double margin_m,int depth=0,
                           const NativeRoadRegion* region=nullptr) const {
    if(c.supplied_geometry)return true;  // Lane feasibility belongs to the supplied spatial planner.
    // 도로 검사만 원래 polygon으로 수행한다. 객체 예측 경계를 담은 Context를
    // 다시 만들지 않으며, 충돌 검사는 기존 Context와 시간별 예측을 그대로 쓴다.
    const auto* native=region?region:c.native_region;
    const auto contains=[&](const Geometry::Box& box) {
      if(!(native?native->containsBox(box):c.boundary_index.containsBox(box))){road_reason_="polygon";return false;}
      Geometry::Bounds bounds;for(const auto& point:Geometry::corners(box))bounds.include(point);
      for(const auto& line:physical_boundaries_)for(const auto i:line->index.queryBounds(bounds))
        if(i>0 && Geometry::segmentIntersectsBox(line->points[i-1],line->points[i],box)){road_reason_="physical-paint";return false;}
      return true;
    };
    if(native && !native->allowsMotion(first.position,last.position)){road_reason_="one-way-boundary";return false;}
    if (contains(
          Geometry::sweptEgoEnvelope(first,last,p_.vehicle,margin_m))) return true;
    // #ROAD_SWEEP_SUBDIVISION
    // 회전하는 차체 sweep의 보수적 사각형은 실제 차체보다 넓다. 도로의
    // 오목한 경계와 5cm 여유가 가까울 때 6회 분할만으로는 끝점이 모두
    // 포함되는 안전한 곡선도 거부될 수 있다. 같은 시작/끝 자세를 선형
    // 보간해 최대 12회까지 enclosure를 정밀화한다. 도로 polygon·차체·여유는
    // 바꾸지 않으며 각 작은 sweep 전체가 포함될 때만 통과한다. 분할 한도에서도
    // 증명되지 않으면 기존처럼 거부하므로 점 검사로 연속 검사를 대신하지 않는다.
    if (depth>=12 || (Geometry::norm(Geometry::sub(first.position,last.position))<1e-9 &&
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
    // SAT encloses both endpoint box sizes by their maximum. Split where a
    // temporary margin starts/restores, so the larger future margin cannot
    // falsely overlap an earlier, still-parallel footprint.
    if(b.progress_m>a.progress_m && t1>t0)for(const auto& entry:initial_obstacle_margins_) {
      for(double station:{entry.second.second,entry.second.second+p_.vehicle.wheelbase_m}) {
        const double u=(station-a.progress_m)/(b.progress_m-a.progress_m);
        if(u>0 && u<1)cuts.push_back(t0+u*(t1-t0));
      }
    }
    std::sort(cuts.begin(),cuts.end());
    cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
    const auto progress_at=[&](double t) {
      return t1>t0 ? Geometry::lerp(a.progress_m,b.progress_m,(t-t0)/(t1-t0)) : a.progress_m;
    };
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
        const double margin0=obstacleMargin(o0.id,progress_at(cuts[k-1]));
        const double margin1=obstacleMargin(o1.id,progress_at(cuts[k]));
        if(p_.recover_static_obstacle_margin && b.progress_m>=a.progress_m &&
           Geometry::norm(Geometry::sub(o1.pose.position,o0.pose.position))<1e-10 &&
           std::abs(Geometry::wrap(o1.pose.yaw_rad-o0.pose.yaw_rad))<1e-10) {
          const auto rear_at=[&](double t) {
            const double u=t1>t0?(t-t0)/(t1-t0):0.;
            return Pose2{Geometry::lerp(a.pose.position,b.pose.position,u),Geometry::yawLerp(a.pose.yaw_rad,b.pose.yaw_rad,u)};
          };
          // A stationary object's collision depends on spatial sweep only.
          // Speed interpolation changes traversal time, not the represented
          // arc. Refine rotation enclosures instead of treating their coarse
          // overlap (or temporal acceleration padding) as a physical contact.
          if(staticSweepOverlap(rear_at(cuts[k-1]),rear_at(cuts[k]),
              Geometry::objectBox(o0,std::max(margin0,margin1))))return true;
        } else if(Geometry::sweptOverlap(e0,e1,Geometry::objectBox(o0,margin0),Geometry::objectBox(o1,margin1)))return true;
      }
    }
    return false;
  }
  bool checkPath(const Context& c,const std::vector<PathPoint>& path) const {
    for (std::size_t i=0;i<path.size();++i) {
      if (!Geometry::finite(path[i].pose) || !std::isfinite(path[i].curvature_inv_m) ||
          std::abs(path[i].curvature_inv_m)>p_.max_curvature_inv_m+1e-6){geometry_reason_="sampled-curvature";return false;}
      if (i==0) {
        if(!roadClear(c,path[i].pose,path[i].pose)){geometry_reason_="initial-road";return false;}
        if(!c.geometric_only && collision(c,path[i],path[i])){geometry_reason_="initial-object";return false;}
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
      if(!roadClear(c,a.pose,b.pose)){geometry_reason_="swept-road";return false;}
      if(!c.geometric_only && collision(c,a,b)){geometry_reason_="swept-object";return false;}
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
    Eigen::VectorXd x=guide.tail(n-2);
    if(!solveBoundedQp(h,q,a,lower,upper,x))return false;
    output=fixed;
    output.tail(n-2)=x;
    return true;
  }

  // #QP_LINEAR_ALGEBRA_REUSE
  // map 경로의 동일한 QP에서 H/q/A는 ADMM의 rho·활성 제약 변경과 무관하다.
  // 정확히 0인 행렬 원소만 희소 표현에서 생략하고 같은 선형식을 계산한다.
  // H^-1 A^T와 A H^-1 A^T는 한 QP 안에서만 재사용하며 다음 관측/후보와
  // 공유하지 않는다. 양의 정부호 Schur 계는 Cholesky로 풀고 수치적으로
  // 불확실한 계는 기존 FullPivLU로 검사한다. KKT의 primal/dual 잔차는
  // 큰 블록 행렬을 만들지 않고 같은 두 식으로 확인한다. 비용·반복 한도·
  // 허용 오차·활성 제약 순서와 최종 도로/충돌/가감속 검사는 유지한다.
  struct QpRefinementCache {
    bool prepared=false,valid=false;
    Eigen::VectorXd unconstrained;
    Eigen::MatrixXd inverse_constraints,schur;
  };
  // 초기 연결과 기존 Frenet QP는 같은 ADMM·active-set 계산을 공유한다.
  bool solveBoundedQp(const Eigen::MatrixXd& h,const Eigen::VectorXd& q,
                      const Eigen::MatrixXd& a,const Eigen::VectorXd& lower,
                      const Eigen::VectorXd& upper,Eigen::VectorXd& x) const {
    const Eigen::SparseMatrix<double> sparse_a=a.sparseView(0.0,0.0);
    const Eigen::SparseMatrix<double> sparse_h=h.sparseView(0.0,0.0);
    const Eigen::SparseMatrix<double> ata=sparse_a.transpose()*sparse_a;
    double rho=p_.qp_rho;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>,Eigen::Lower,Eigen::NaturalOrdering<int>> factor(sparse_h+rho*ata);
    if (factor.info()!=Eigen::Success) return false;
    QpRefinementCache refinement;
    Eigen::VectorXd z=a*x,u=Eigen::VectorXd::Zero(a.rows());
    for (Eigen::Index j=0;j<z.size();++j) z[j]=Geometry::clamp(z[j],lower[j],upper[j]);
    for (int iteration=1;iteration<=p_.qp_max_iterations;++iteration) {
      x=factor.solve(-q+rho*sparse_a.transpose()*(z-u));
      if (!x.allFinite()) return false;
      const Eigen::VectorXd ax=sparse_a*x;
      const Eigen::VectorXd relaxed=1.6*ax-0.6*z;
      z=relaxed+u;
      for (Eigen::Index j=0;j<z.size();++j) z[j]=Geometry::clamp(z[j],lower[j],upper[j]);
      u+=relaxed-z;
      const double primal_residual=(ax-z).lpNorm<Eigen::Infinity>();
      const double dual_residual=(sparse_h*x+q+rho*sparse_a.transpose()*u).lpNorm<Eigen::Infinity>();
      if (primal_residual<=p_.qp_tolerance && dual_residual<=p_.qp_tolerance) {
        return true;
      }
      if (iteration%100==0 && refineActiveSet(h,q,a,lower,upper,z,x,&refinement)) {
        return true;
      }
      if (iteration%50==0) {
        const double ratio=primal_residual/std::max(dual_residual,1e-15);
        if (ratio>3.0 || ratio<1.0/3.0) {
          const double next=Geometry::clamp(rho*std::sqrt(ratio),0.1,1e4);
          u*=rho/next; rho=next;
          factor.compute(sparse_h+rho*ata);
          if (factor.info()!=Eigen::Success) return false;
        }
      }
    }
    // Iteration limit is not proof of physical infeasibility.
    return false;
  }
  bool refineActiveSet(const Eigen::MatrixXd& h,const Eigen::VectorXd& q,const Eigen::MatrixXd& a,
                       const Eigen::VectorXd& lower,const Eigen::VectorXd& upper,const Eigen::VectorXd& z,
                       Eigen::VectorXd& x,QpRefinementCache* cache=nullptr) const {
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
    QpRefinementCache local;auto& matrices=cache?*cache:local;
    if(!matrices.prepared) {
      matrices.prepared=true;
      const Eigen::SparseMatrix<double> sparse_h=h.sparseView(0.0,0.0);
      const Eigen::SparseMatrix<double> sparse_a=a.sparseView(0.0,0.0);
      const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>,Eigen::Lower,Eigen::NaturalOrdering<int>> h_factor(sparse_h);
      if(h_factor.info()!=Eigen::Success || h_factor.vectorD().minCoeff()<=0)return false;
      matrices.unconstrained=h_factor.solve(-q);
      const Eigen::MatrixXd transpose=a.transpose();
      matrices.inverse_constraints=h_factor.solve(transpose);
      matrices.schur=sparse_a*matrices.inverse_constraints;matrices.valid=true;
    }
    if(!matrices.valid)return false;
    const auto& unconstrained=matrices.unconstrained;
    for (int iteration=0;iteration<40;++iteration) {
      const Eigen::Index m=static_cast<Eigen::Index>(active.size());
      Eigen::MatrixXd constraints(m,n);
      Eigen::VectorXd bound(m);
      for (Eigen::Index j=0;j<m;++j) {
        const auto item=active[static_cast<std::size_t>(j)];
        constraints.row(j)=a.row(item.row);
        bound[j]=item.upper ? upper[item.row] : lower[item.row];
      }
      Eigen::VectorXd candidate(n+m);
      candidate.head(n)=unconstrained;
      if(m>0) {
        Eigen::MatrixXd projected(n,m),schur(m,m);
        for(Eigen::Index j=0;j<m;++j) {
          projected.col(j)=matrices.inverse_constraints.col(active[static_cast<std::size_t>(j)].row);
          for(Eigen::Index i=0;i<m;++i)schur(i,j)=matrices.schur(active[static_cast<std::size_t>(i)].row,active[static_cast<std::size_t>(j)].row);
        }
        const Eigen::VectorXd residual=constraints*unconstrained-bound;
        const Eigen::LLT<Eigen::MatrixXd> positive(schur);
        bool solved=false;
        if(positive.info()==Eigen::Success) {
          const auto diagonal=positive.matrixL().toDenseMatrix().diagonal().eval();
          const double threshold=std::numeric_limits<double>::epsilon()*m*schur.cwiseAbs().maxCoeff();
          if(diagonal.array().square().minCoeff()>threshold) {
            candidate.tail(m)=positive.solve(residual);
            solved=true;
          }
        }
        if(!solved) {
          const Eigen::FullPivLU<Eigen::MatrixXd> factor(schur);
          if(!factor.isInvertible())return false;
          candidate.tail(m)=factor.solve(residual);
        }
        candidate.head(n)-=projected*candidate.tail(m);
      }
      Eigen::VectorXd residual(n+m);
      residual.head(n)=h*candidate.head(n)+constraints.transpose()*candidate.tail(m)+q;
      residual.tail(m)=constraints*candidate.head(n)-bound;
      if(!candidate.allFinite() || residual.lpNorm<Eigen::Infinity>()>p_.qp_tolerance)return false;
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
      const Eigen::VectorXd dual=h*candidate.head(n)+q+constraints.transpose()*candidate.tail(m);
      if (dual.lpNorm<Eigen::Infinity>()>p_.qp_tolerance) return false;
      x=candidate.head(n);
      return true;
    }
    return false;
  }

 public:
  const char* failureReason() const {return failure_reason_;}
  std::string diagnostics() const {
    std::ostringstream out;out<<"source="<<selected_source_<<" extent="<<selected_extent_<<" peak_k="<<selected_curvature_<<" projection="<<projection_distance_<<" heading="<<projection_heading_<<" geometry_reason="<<geometry_reason_<<" road_reason="<<road_reason_<<" retry="<<extended_search_<<" layer="<<failed_layer_<<" check="<<path_check_reason_;
    const char* names[]={"budget","geometry","space","endpoint","dynamics","curvature","lateral","sweep","terminal","check"};
    for(std::size_t i=0;i<rejected_.size();++i)out<<" "<<names[i]<<"="<<rejected_[i];
    return out.str();
  }
  std::vector<ReferencePoint> mapReference(const EgoState& ego,const Route& route,bool smooth=true) {
    const auto native=nativeReference(route);
    const auto& points=native->points;
    std::vector<double> stations(points.size(),0.);
    // The route begins with predecessor/current. A later crossing of the same
    // road is not the current projection, even if it is centimetres closer.
    std::size_t lane_index=0;double lane_distance=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<std::min<std::size_t>(2,route->size());++i) {
      const double distance=lanelet::geometry::distance2d((*route)[i].centerline2d(),
          lanelet::BasicPoint2d(ego.pose.position.x,ego.pose.position.y));
      if(distance<lane_distance){lane_distance=distance;lane_index=i;}
    }
    std::size_t origin_begin=0,origin_end=points.size();
    for(const auto& group:native->groups)if(lane_index>=group.route_begin && lane_index<=group.route_end) {
      origin_begin=group.point_begin>0?group.point_begin-1:0;origin_end=group.point_end;break;
    }
    double best=std::numeric_limits<double>::infinity(),origin=0;
    for(std::size_t i=1;i<points.size();++i) {
      const auto delta=Geometry::sub(points[i].position,points[i-1].position);
      const double length=Geometry::norm(delta);stations[i]=stations[i-1]+length;
      if(length<1e-9 || i<=origin_begin || i>=origin_end)continue;
      const double u=Geometry::clamp(Geometry::dot(Geometry::sub(ego.pose.position,points[i-1].position),delta)/(length*length),0,1);
      const auto q=Geometry::add(points[i-1].position,Geometry::mul(delta,u));
      const double distance=Geometry::norm(Geometry::sub(q,ego.pose.position));
      if(distance<best-1e-6) {best=distance;origin=stations[i-1]+u*length;}
    }
    const double begin=std::max(0.,origin-8),end=origin+p_.trajectory_horizon_m+p_.vehicle.front_m+8;
    std::vector<ReferencePoint> result;
    const auto first=std::lower_bound(stations.begin(),stations.end(),begin);
    const std::size_t lo=first==stations.begin()?0:std::size_t(first-stations.begin()-1);
    if(p_.preferred_reference_piece_m>0 && stations.back()>3) {
      // Uniform, map-fixed supports suppress centimetre-scale Lanelet
      // midpoint teeth before differentiating the reference in Frenet DP/QP.
      // Road permission still comes from the original Lanelet polygons and
      // physical paint. This changes the search reference by at most 10 cm.
      const auto at=[&](double station) {
        station=Geometry::clamp(station,0.,stations.back());
        const auto right=std::max<std::size_t>(1,std::min<std::size_t>(points.size()-1,
            std::upper_bound(stations.begin(),stations.end(),station)-stations.begin()));
        const double length=stations[right]-stations[right-1];
        const double u=length>0?(station-stations[right-1])/length:0;
        return ReferencePoint{Geometry::lerp(points[right-1].position,points[right].position,u),
            Geometry::lerp(points[right-1].min_l_m,points[right].min_l_m,u),
            Geometry::lerp(points[right-1].max_l_m,points[right].max_l_m,u)};
      };
      const double limit=std::min(end,stations.back());
      const auto append=[&](double station) {
        auto point=at(station);
        if(smooth) {
          Vec2 smoothed{};double weight=0;
          for(int j=-12;j<=12;++j) {
            const double w=std::exp(-.5*std::pow(j*.25,2));
            smoothed=Geometry::add(smoothed,Geometry::mul(at(station+j*.25).position,w));weight+=w;
          }
          const auto delta=Geometry::sub(Geometry::mul(smoothed,1/weight),point.position);
          const double factor=.1/std::sqrt(.01+Geometry::dot(delta,delta));
          point.position=Geometry::add(point.position,Geometry::mul(delta,factor));
        }
        result.push_back(point);
      };
      const double step=smooth?1.:.25;
      for(double station=std::floor(begin/step)*step;station<limit-1e-6;station+=step)append(station);
      append(limit);
      return result;
    }
    for(std::size_t i=lo;i<points.size();++i) {
      result.push_back(points[i]);if(stations[i]>=end)break;
    }
    return result;
  }
  std::vector<PathPoint> planInMap(const EgoState& ego,
      const std::vector<ReferencePoint>& reference, const Predictions& predictions,
      const PlanningConstraints& limits, const Route& route,
      const std::vector<PathPoint>& previous = {},
      const std::vector<std::pair<double,double>>& speed_limits = {},
      const std::vector<bool>& physical_coverage = {},
      const std::vector<PathPoint>* fixed_geometry=nullptr,
      const std::vector<ReferencePoint>* physical_reference=nullptr) {
    mission_speed_limits_=speed_limits;
    speed_searches_=0;geometry_searches_=0;
    auto road=nativeRoadRegion(rearFootprintRoute(route));
    const auto& physical=physical_reference?*physical_reference:reference;
    if(physical_coverage.size()==physical.size() && !physical_boundaries_.empty()) {
      auto corrected=std::make_shared<NativeRoadRegion>(*road);
      corrected->original_lanelets=road;
      const Reference guide(physical);
      for(std::size_t begin=0;begin<physical.size();) {
        if(!physical_coverage[begin]){++begin;continue;}
        auto end=begin;while(end+1<physical.size() && physical_coverage[end+1])++end;
        if(end>begin) {
          RoadPolygon patch;std::vector<Vec2> left,right;double station=0;
          for(std::size_t i=0;i<=end;++i) {
            if(i)station+=Geometry::norm(Geometry::sub(physical[i].position,physical[i-1].position));
            if(i<begin)continue;
            const auto normal=Geometry::normal(guide.at(station).yaw);
            left.push_back(Geometry::add(physical[i].position,Geometry::mul(normal,physical[i].max_l_m)));
            right.push_back(Geometry::add(physical[i].position,Geometry::mul(normal,physical[i].min_l_m)));
          }
          for(const auto& p:right)patch.outer().emplace_back(p.x,p.y);
          for(auto i=left.rbegin();i!=left.rend();++i)patch.outer().emplace_back(i->x,i->y);
          boost::geometry::correct(patch);
          if(boost::geometry::is_valid(patch)) {
            RoadPolygons merged;boost::geometry::union_(corrected->polygons,patch,merged);
            if(!merged.empty())corrected->polygons=std::move(merged);
          }
        }
        begin=end+1;
      }
      corrected->prepare();road=std::move(corrected);
    }
    const NativeRegionScope region(active_native_region_,road);
    if(fixed_geometry)return timeReference(ego,reference,predictions,limits,*fixed_geometry);
    return plan(ego,reference,predictions,limits,previous);
  }


 private:
  // Retime a spatial path selected by the source lanelet planner. Do not solve
  // lateral feasibility again on its cropped map mesh. Source curvature preview
  // supplies the cruising cost; DP still checks measured v/a, jerk, braking
  // reserve, requested stops and swept, time-aligned obstacle footprints.
  std::vector<PathPoint> timeReference(const EgoState& observed,
      const std::vector<ReferencePoint>& reference_map,const Predictions& predictions,
      const PlanningConstraints& limits,const std::vector<PathPoint>& fixed) const {
    failure_reason_="speed-search";path_check_reason_="none";failed_layer_=-1;
    rejected_.fill(0);validated_stop_.clear();extended_search_=false;
    if(fixed.size()<2){failure_reason_="reference-geometry";return {};}
    requireSceneDomain(observed,reference_map,predictions);
    const Reference reference(reference_map);
    const auto geometry=roadGeometry(reference_map,reference);
    const std::vector<ProgressTime> unused;
    Context c{reference,predictions,unused,geometry->boundary,0,
        observed.stamp_sec-predictions.reference_stamp_sec,false,&geometry->index,nullptr};
    c.supplied_geometry=true;
    PathPoint actual;actual.pose=observed.pose;
    if(collision(c,actual,actual)){failure_reason_="initial-collision";return {};}
    ArcPath arc(fixed);arc.supplied_geometry=true;
    EgoState nominal=observed;nominal.pose=fixed.front().pose;nominal.curvature_inv_m=0;
    if(Geometry::norm(Geometry::sub(nominal.pose.position,observed.pose.position))>p_.max_initial_distance_m) {
      failure_reason_="initial-projection";return {};
    }
    const double duration=predictions.step_sec*kPredictionSteps-(observed.stamp_sec-predictions.reference_stamp_sec);
    if(observed.stamp_sec<predictions.reference_stamp_sec || duration<=0) {
      failure_reason_="forecast-time";return {};
    }
    requirePlanningDomain(nominal,limits);
    if(!std::isfinite(brakingDistance(nominal.speed_mps,nominal.acceleration_mps2))) {
      failure_reason_="initial-longitudinal";return {};
    }
    const double stop=stopArc(nominal,reference,arc,limits);
    bool truncated=false;
    auto result=planSpeed(c,arc,nominal,limits,duration,stop,reference_map,p_.speed_beam_width,truncated);
    if(!result.empty() && checkPlannedPath(nominal,reference_map,predictions,limits,result,true))return result;
    if(p_.recovery_speed_beam_width>p_.speed_beam_width) {
      speed_searches_=0;extended_search_=true;
      result=planSpeed(c,arc,nominal,limits,duration,stop,reference_map,p_.recovery_speed_beam_width,truncated);
      if(!result.empty() && checkPlannedPath(nominal,reference_map,predictions,limits,result,true))return result;
    }
    return {};
  }
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
    struct Group {std::size_t route_begin,route_end,point_begin,point_end;};
    std::vector<Group> groups;
    std::vector<LaneKey> route_ids;
    std::vector<ReferencePoint> points;
    std::vector<Eigen::Vector3d> heights;
    std::vector<Vec2> independent_entries;
  };
  std::shared_ptr<const NativeReference> native_reference_cache_,mission_reference_cache_;
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
    bool previous_group_transition=false;
    for(std::size_t begin=0;begin<route->size();) {
      const auto point_begin=result->points.size();
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
        // #EXIT_LANE_CHANGE_CONNECTOR
        // 짧은 source를 합법적인 종방향 source/target 쌍으로 연장한 경우,
        // 남은 도로를 이어 놓고도 횡이동을 25m에 압축하면 고속 접근 경로가
        // 불필요하게 감속한다. 이 경우만 기존 C2 연결 상한 2*horizon_m까지
        // 전환에 사용한다. 전환 시작점은 map에 고정하고 긴 일반 차선 변경의
        // 기존 horizon_m을 유지하고, 바로 앞 그룹도 차선 변경이면 연속 변경의
        // 기존 완료 거리를 유지한다. 실선·분기·허용 도로,
        // 차체·객체·곡률·횡가속도·가감속·시간 검사는 그대로 적용한다.
        const bool independent_connector=connected_end>end && !previous_group_transition;
        if(independent_connector)result->independent_entries.push_back({source.front().x(),source.front().y()});
        const double transition_extent=std::min(total,(independent_connector ? 2 : 1)*p_.horizon_m);
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
      result->groups.push_back({begin,connected_end,point_begin,result->points.size()});
      previous_group_transition=begin!=end;
      begin=connected_end+1;
    }
    native_reference_cache_=result;
    return result;
  }

  // #SEQUENTIAL_LANE_CHANGE
  // GlobalPath는 경유 방향을 제공하고, 실제 조작은 인접한 한 차선만 대상으로 한다.
  // 연속 left/right edge를 두 차선 폭의 한 곡선으로 합치지 않는다. 목표 lanelet이
  // 종방향으로 잘게 나뉘어도 같은 목표 차선을 이어서 유지하며, 매 계산에서 기존
  // DP/QP·예측 객체·swept 차체·곡률·속도·가감속 검사를 그대로 적용한다.
  struct LaneChange {
    lanelet::ConstLanelets source,target;
    bool left=false;
    std::shared_ptr<const NativeRoadRegion> target_region,corridor;
    Route reference;
    std::shared_ptr<const NativeReference> planned_reference;
    std::shared_ptr<const Reference> target_road;
    double last_observation=-1,stable_since=-1,stable_steering=0;
    Pose2 stable_pose;
  };
  lanelet::Optional<LaneChange> lane_change_;

  std::shared_ptr<const NativeReference> laneHoldReference(const Route& mission,const EgoState& ego) {
    if(!routing_graph_)return nullptr;
    const lanelet::BasicPoint2d position(ego.pose.position.x,ego.pose.position.y);
    for(std::size_t current=0;current<mission->size();++current) {
      if(!lanelet::geometry::inside((*mission)[current],position))continue;
      for(std::size_t i=current;i+1<mission->size();++i) {
        const auto& source=(*mission)[i];const auto& target=(*mission)[i+1];
        const auto left=legalNeighbor(source,true),right=legalNeighbor(source,false);
        if(!((left && laneKey(*left)==laneKey(target)) || (right && laneKey(*right)==laneKey(target))))continue;
        const double duration=kPredictionSteps*.5;
        const double reachable=ego.speed_mps*duration+.5*p_.max_acceleration_mps2*duration*duration;
        if(lanelet::geometry::distance2d(source.polygon2d(),position)>
           std::min(p_.trajectory_horizon_m,reachable))return nullptr;
        lanelet::ConstLanelets lanes(mission->begin()+current,mission->begin()+i+1);
        std::set<LaneKey> visited;for(const auto& lane:lanes)visited.insert(laneKey(lane));
        while((lanes.back().centerline3d().back().basicPoint2d()-position).norm()<
              p_.trajectory_horizon_m+p_.vehicle.front_m+p_.vehicle.rear_m) {
          const auto next=routing_graph_->following(lanes.back(),false);
          lanelet::ConstLanelets planned;
          for(const auto& lane:*mission)if(std::find(next.begin(),next.end(),lane)!=next.end())planned.push_back(lane);
          const auto choices=planned.size()==1 ? planned : next;
          if(choices.size()!=1 || !visited.insert(laneKey(choices.front())).second)break;
          lanes.push_back(choices.front());
        }
        const auto hold_route=rearFootprintRoute(lanelet::routing::LaneletPath(std::move(lanes)));
        // 실제 출발 차선의 종방향 polygon만 사용한다. 인접 차선은 이 포함
        // 판정에 합치지 않으며, 전파된 이번 계획 자세의 전체 차체를 검사한다.
        if(!nativeRoadRegion(hold_route,false)->containsBox(Geometry::sweptEgoEnvelope(
            ego.pose,ego.pose,p_.vehicle,p_.road_margin_m)))return nullptr;
        auto result=std::make_shared<NativeReference>(*nativeReference(hold_route));
        // #SOURCE_LANE_CENTER_JOIN
        // 공유 경계의 끝점이 같은 두 lanelet도 접속 단면의 중심선 표본 사이에
        // 13cm 가로 선분을 담을 수 있다. 이 선분은 직진 유지 형상의 Frenet
        // 횡기울기를 잘못 키운다. 이 후보에만 접속점에서 기존 출력 공간 간격
        // 이내의 표본을 공유 경계 양 끝의 실제 중앙으로 합친다.
        // 지도·임무 참조·도로 polygon은 그대로이며 C2/차체/동역학 검사를 유지한다.
        for(std::size_t j=1;j<hold_route->size();++j) {
          const auto& before=(*hold_route)[j-1];const auto& after=(*hold_route)[j];
          if(before.leftBound().back()!=after.leftBound().front() ||
             before.rightBound().back()!=after.rightBound().front())continue;
          const Eigen::Vector3d joint=(after.leftBound3d().front().basicPoint()+
                                       after.rightBound3d().front().basicPoint())/2;
          for(std::size_t k=0;k<result->points.size();++k) {
            if((result->heights[k]-joint).head<2>().norm()>p_.output_step_m)continue;
            result->points[k].position={joint.x(),joint.y()};result->heights[k]=joint;
          }
        }
        for(std::size_t k=1;k<result->points.size();) {
          if(Geometry::norm(Geometry::sub(result->points[k].position,result->points[k-1].position))>1e-9) {++k;continue;}
          result->points.erase(result->points.begin()+k);result->heights.erase(result->heights.begin()+k);
        }
        return result;
      }
      break;
    }
    return nullptr;
  }

  void extendLaneChange(const Route& mission,const EgoState& ego) {
    auto& change=*lane_change_;
    const auto old_size=change.target.size();
    std::set<LaneKey> visited;
    for(const auto& lane:change.target)visited.insert(laneKey(lane));
    // Euclidean distance cannot exceed the road arc distance: extending until the
    // tail is outside this preview never shortens the required forward coverage.
    while((change.target.back().centerline3d().back().basicPoint2d()-
           lanelet::BasicPoint2d(ego.pose.position.x,ego.pose.position.y)).norm()<
          p_.trajectory_horizon_m+p_.vehicle.front_m+p_.vehicle.rear_m) {
      const auto next=routing_graph_->following(change.target.back(),false);
      lanelet::ConstLanelets planned;
      for(const auto& lane:*mission)
        if(std::find(next.begin(),next.end(),lane)!=next.end())planned.push_back(lane);
      const auto choices=planned.size()==1 ? planned : next;
      if(choices.size()!=1 || !visited.insert(laneKey(choices.front())).second)break;
      const auto target=choices.front();
      lanelet::ConstLanelets sources;
      for(const auto& candidate:routing_graph_->following(change.source.back(),false)) {
        const auto neighbor=legalNeighbor(candidate,change.left);
        if(neighbor && laneKey(*neighbor)==laneKey(target))sources.push_back(candidate);
      }
      if(sources.size()==1)change.source.push_back(sources.front());
      change.target.push_back(target);
    }
    if(change.reference && old_size==change.target.size())return;
    const auto target_route=rearFootprintRoute(lanelet::routing::LaneletPath(change.target));
    change.target_region=nativeRoadRegion(target_route,false);
    change.target_road.reset(new Reference(nativeReference(target_route)->points));
    lanelet::ConstLanelets region=change.source;
    region.insert(region.end(),target_route->begin(),target_route->end());
    change.corridor=nativeRoadRegion(rearFootprintRoute(lanelet::routing::LaneletPath(region)),false);
    // The first source/target pair anchors the existing connector in map. Later
    // lateral edges are not appended until the observed maneuver has completed.
    lanelet::ConstLanelets reference{change.source.front()};
    reference.insert(reference.end(),change.target.begin(),change.target.end());
    change.reference=rearFootprintRoute(lanelet::routing::LaneletPath(std::move(reference)));
    change.planned_reference=nativeReference(change.reference);
  }
  // #LANE_CHANGE_COMPLETION_CONFIRMATION
  // 뒷차축만 경계를 넘거나 조향이 순간 0을 지나는 것을 완료로 보지 않는다.
  // 전체 차체와 기존 도로 여유가 목표 차선 안에 있고, 목표 중심선에 가깝고,
  // 도로 방향 및 도로 곡률에 필요한 타이어 조향과 정합된 전진 관측을 누적한다.
  // 핸들 회전각이나 벽시계는 쓰지 않는다. 같은 ego stamp는 시간이 쌓이지 않고,
  // 정지·관측 공백·정렬 이탈 시 1초 누적을 다시 시작한다. 목표 차선의 실제
  // 종방향 연결 polygon을 사용하므로 lanelet ID가 바뀌어도 판정이 끊기지 않는다.
  bool laneChangeComplete(const EgoState& ego,double steering,double stamp) {
    auto& change=*lane_change_;
    const double gap=stamp-change.last_observation;
    if(gap<0 || gap>p_.lane_change_observation_gap_sec)change.stable_since=-1;
    const auto& road=*change.target_road;
    const auto projection=road.project(ego.pose.position,0,road.length());
    // 기존 C2 끝점 계산과 같은 공간 표본을 쓴다. 조밀한 지도 knot의 순간
    // 곡률 부호 변화가 정상 곡선 주행의 완료 판정을 계속 초기화하지 않게 한다.
    const auto sample=cartesianEndpoint(road,projection.s);
    // 실제 조향 피드백의 부호에 의존하지 않는다. MPC와 동일한 yaw/v 등가
    // 타이어 조향으로 도로 곡률과의 잔차를 비교하고, 원본 조향은 변화량만 본다.
    // 0.5m/s 미만의 yaw/v는 안정되지 않으므로 그 관측으로 완료를 누적하지 않는다.
    const double effective_steering=std::atan(p_.vehicle.wheelbase_m*ego.curvature_inv_m);
    const bool stable=ego.speed_mps>=.5 &&
        change.target_region->containsBox(Geometry::sweptEgoEnvelope(
            ego.pose,ego.pose,p_.vehicle,p_.road_margin_m)) &&
        projection.distance<=p_.corridor_half_width_m &&
        std::abs(Geometry::wrap(ego.pose.yaw_rad-sample.yaw))<=p_.lane_change_heading_tolerance_rad &&
        std::abs(effective_steering-std::atan(p_.vehicle.wheelbase_m*sample.curvature))<=
            p_.lane_change_steering_tolerance_rad &&
        (change.stable_since<0 || std::abs(steering-change.stable_steering)<=p_.lane_change_steering_tolerance_rad);
    // 같은 ego 시각이어도 새 조향 관측에서 불안정이 보이면 누적은 초기화한다.
    if(!stable) {change.stable_since=-1;change.last_observation=stamp;return false;}
    if(gap<=0) {change.last_observation=stamp;return false;}
    change.last_observation=stamp;
    if(change.stable_since<0) {
      change.stable_since=stamp;change.stable_pose=ego.pose;change.stable_steering=steering;
    }
    const double progress=Geometry::dot(Geometry::sub(ego.pose.position,change.stable_pose.position),
                                       Geometry::unit(change.stable_pose.yaw_rad));
    return stamp-change.stable_since>=p_.lane_change_confirmation_sec-1e-9 &&
           progress>std::max(1e-6,p_.road_margin_m);
  }
  Route laneChangeRoute(const Route& mission,const EgoState& ego,
                        double steering,double stamp) {
    if(!routing_graph_)return mission;
    if(lane_change_) {
      extendLaneChange(mission,ego);
      // A reset to another part of the map abandons the old maneuver, rather
      // than steering back to an obsolete target after a scenario reload.
      double distance=std::numeric_limits<double>::infinity();
      for(const auto* lanes:{&lane_change_->source,&lane_change_->target})
        for(const auto& lane:*lanes)distance=std::min(distance,lanelet::geometry::distance2d(
            lane.polygon2d(),lanelet::BasicPoint2d(ego.pose.position.x,ego.pose.position.y)));
      if(distance>p_.max_initial_distance_m)lane_change_.reset();
      else if(laneChangeComplete(ego,steering,stamp)) {
        ROS_INFO_STREAM("lane_change completed target=" << lane_change_->target.front().id()
            << " ego_sec=" << std::setprecision(17) << stamp
            << " stable_since=" << lane_change_->stable_since);
        lane_change_.reset();
      } else return lane_change_->reference;
    }
    for(std::size_t i=0;i+1<mission->size();++i) {
      const auto& source=(*mission)[i];const auto& target=(*mission)[i+1];
      const auto left=legalNeighbor(source,true),right=legalNeighbor(source,false);
      const bool move_left=left && laneKey(*left)==laneKey(target);
      if(!move_left && !(right && laneKey(*right)==laneKey(target)))continue;
      // #LANE_CHANGE_MISSION_ORDER
      // GlobalPath는 필수 경유 구간을 보존한 채 첫 합법 변경 edge를 앞당긴다.
      // LocalPath가 그 source보다 앞의 종방향 구간까지 다시 당기면, 먼저 목표
      // 차선에 도착한 뒤 아직 남은 경유 차선으로 되돌아가는 왕복이 생긴다.
      // 전달된 edge의 source에서 바로 옆 target만 시작한다. 실제 간격·변경
      // 시각·속도와 1초 완료 판정은 기존 계산이 결정하며, 경유 조건은 버리지 않는다.
      if(!lanelet::geometry::inside(source,lanelet::BasicPoint2d(ego.pose.position.x,ego.pose.position.y)))continue;
      lane_change_=LaneChange{};
      lane_change_->source={source};lane_change_->target={target};lane_change_->left=move_left;
      extendLaneChange(mission,ego);
      ROS_INFO_STREAM("lane_change begin source=" << source.id() << " target=" << target.id()
          << " ego_sec=" << std::setprecision(17) << stamp);
      return lane_change_->reference;
    }
    return mission;
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
namespace path_planner {
struct IntegratedDynamicTrajectoryParameters {
  integrated_native::PathParameters solver;
  bool publish_lateral_reference=false;
  double curve_preview_m=45.,curve_spacing_m=2.,curve_deceleration_mps2=1.7;
  double minimum_curve_speed_mps=15./3.6;
  double corridor_half_width_m=1.8;
  double horizon_sec=5.0;
  double recovery_deceleration_mps2=8.,recovery_jerk_mps3=8.,recovery_response_sec=.3,recovery_minimum_speed_mps=0;
  double recovery_lateral_acceleration_mps2=6.;
  double dynamic_zone_speed_mps=25./3.6,dynamic_zone_approach_m=30;
  // Dynamic-zone emergency hold applies only to a body measured moving at
  // least this fast; slower bodies and future conflicts go to the solver.
  double dynamic_zone_moving_speed_mps=.8;
  // Footprints up to this extent are treated as vulnerable road users whose
  // forecast body grows with prediction time, capped at the maximum.
  double vulnerable_extent_m=1.5,vulnerable_uncertainty_mps=.3,vulnerable_uncertainty_max_m=1.;
  // Inside the dynamic zone a small body whose motion is not yet measured or
  // inferred may be walking: its forecast body grows at this speed, capped.
  double unknown_walker_speed_mps=0,unknown_walker_max_m=0;
};


struct IntegratedDynamicParameters {
  PathPlannerMissionParameters mission;
  IntegratedDynamicTrajectoryParameters trajectory;
};
class IntegratedDynamic {
class IntegratedTrajectory {
 public:
  IntegratedTrajectory(IntegratedDynamicTrajectoryParameters p,const lanelet::LaneletMap& map)
      : p_(std::move(p)),solver_(p_.solver,map),recovery_solver_(solver_,recoveryParameters(p_)),
        initial_solver_(solver_),initial_recovery_solver_(recovery_solver_) {}
  const char* outcome()const{return outcome_;}
  std::string diagnostics()const{return used_recovery_ ? "dynamics-recovery "+recovery_solver_.diagnostics():solver_.diagnostics();}
  Message calculate(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& status,
      const interfaces::Objects& objects,const sensor_msgs::Imu& imu,const Route& route,
      const PredictedObjects& predictions,const ros::Time& stamp,double ceiling,bool dynamic_zone,
      double detection,double clear_sec) {
    used_recovery_=false;outcome_="native-search";
    try {
      integrated_native::EgoState state;
      const auto& q=ego.pose.pose.orientation;
      state.pose={{ego.pose.pose.position.x,ego.pose.pose.position.y},heading(q)};
      state.stamp_sec=stamp.toSec();
      state.speed_mps=std::max(0.,double(status.velocity_x_mps));
      const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
      state.acceleration_mps2=imu.linear_acceleration.x-
          (rotation.conjugate()*Eigen::Vector3d(0,0,9.81)).x();
      // At measured standstill, brake/contact force is not a continuing
      // backwards acceleration. Preserve ground speed and restart the model
      // from zero acceleration when the vehicle is held by its brake.
      if(state.speed_mps<.001 && std::abs(state.acceleration_mps2)<.05)
        state.acceleration_mps2=0;
      if(status.brake>0 && state.acceleration_mps2<0) {
        // Brake force cannot be extended through rest into reverse motion.
        // Project only an infeasible braking seed onto the forward-only
        // reference model. Measured speed and all road/obstacle limits stay
        // unchanged; assuming less braking is conservative for stopping space.
        const double release_bound=std::sqrt(2*p_.solver.max_longitudinal_jerk_mps3*state.speed_mps);
        state.acceleration_mps2=state.speed_mps<.05 ? 0. :
            std::max(state.acceleration_mps2,-release_bound*(1-1e-8));
      }
      // MORAI status angular velocity uses the opposite yaw convention to
      // ROS IMU/map heading. Use the IMU for curve and pose propagation.
      state.curvature_inv_m=state.speed_mps>.5?imu.angular_velocity.z/state.speed_mps:0;
      if(stamp.toSec()<last_stamp_)previous_.clear();
      last_stamp_=stamp.toSec();
      const double ego_dt=std::max(0.,stamp.toSec()-ego.header.stamp.toSec());
      const double rate=imu.angular_velocity.z,angle=rate*ego_dt;
      const double forward=ego.twist.twist.linear.x,lateral=ego.twist.twist.linear.y;
      const double a=std::abs(rate)>1e-9?std::sin(angle)/rate:ego_dt;
      const double b=std::abs(rate)>1e-9?(1-std::cos(angle))/rate:0.;
      const double dx=forward*a-lateral*b,dy=forward*b+lateral*a;
      state.pose.position.x+=std::cos(state.pose.yaw_rad)*dx-std::sin(state.pose.yaw_rad)*dy;
      state.pose.position.y+=std::sin(state.pose.yaw_rad)*dx+std::cos(state.pose.yaw_rad)*dy;
      state.pose.yaw_rad+=angle;
      // A checkpoint return, or this Core taking over after another section's
      // Core drove on, moves the vehicle away from the place of the last call.
      // Previous plans, maneuvers and per-object memories belong to that place:
      // restart both solvers from their constructed state and replan from here.
      const double moved=std::hypot(state.pose.position.x-last_position_.x,state.pose.position.y-last_position_.y);
      // Planner stalls of up to 2 s at full speed stay below this bound.
      const double elapsed=std::max(0.,std::min(2.,stamp.toSec()-last_position_stamp_));
      if(has_last_position_ && (stamp.toSec()<last_position_stamp_ ||
          moved>p_.solver.max_initial_distance_m+state.speed_mps*elapsed)) {
        solver_=initial_solver_;recovery_solver_=initial_recovery_solver_;previous_.clear();
        last_blocked_sec_=-std::numeric_limits<double>::infinity();
        ROS_INFO("[native planner] planning state restarted after a %.1f m jump",moved);
      }
      has_last_position_=true;last_position_=state.pose.position;last_position_stamp_=stamp.toSec();

      auto reference=solver_.mapReference(state,route);
      Message guide;
      // The numerical reference includes supports behind the rear axle.
      // Curvature speed preview starts at the vehicle, not eight metres back.
      std::size_t guide_begin=0;double guide_distance=std::numeric_limits<double>::infinity();
      for(std::size_t i=0;i<reference.size();++i) {
        const double d=std::hypot(reference[i].position.x-state.pose.position.x,
            reference[i].position.y-state.pose.position.y);
        if(d<guide_distance){guide_distance=d;guide_begin=i;}
      }
      for(std::size_t i=guide_begin;i<reference.size();++i) {
        interfaces::PathPoint point;point.pose.pose.position.x=reference[i].position.x;point.pose.pose.position.y=reference[i].position.y;
        const auto a=i==0?0:i-1,b=i+1<reference.size()?i+1:i;
        const double yaw=std::atan2(reference[b].position.y-reference[a].position.y,reference[b].position.x-reference[a].position.x);
        point.pose.pose.orientation.z=std::sin(yaw/2);point.pose.pose.orientation.w=std::cos(yaw/2);guide.points.push_back(point);
      }
      if(dynamic_zone) {
        if(dynamicBlocked(reference,state,objects,predictions,detection))last_blocked_sec_=stamp.toSec();
        if(stamp.toSec()-last_blocked_sec_<clear_sec) {
          outcome_="dynamic-zone-stop";previous_.clear();return hold(ego);
        }
      } else last_blocked_sec_=-std::numeric_limits<double>::infinity();
      auto native=convert(objects,predictions,p_,dynamic_zone);

      std::vector<bool> coverage;

      integrated_native::PlanningConstraints limits;
      limits.maximum_speed_mps=std::min(p_.solver.max_speed_mps,ceiling);
      limits.preferred_speed_mps=referenceCurveTarget(guide,limits.maximum_speed_mps);
      auto planned=solver_.planInMap(state,reference,native,limits,route,previous_,{},coverage);
      const bool stopped_free=!planned.empty() && planned.back().speed_mps==0 && objects.objects.empty();
      if((planned.empty() || stopped_free) && std::string(solver_.failureReason())!="initial-collision") {
        auto alternative=recovery_solver_.planInMap(state,reference,native,limits,route,previous_,{},coverage);
        if(!alternative.empty() && (planned.empty() || alternative.back().speed_mps>planned.back().speed_mps)) {
          planned=std::move(alternative);used_recovery_=true;
        }
      }
      if(planned.empty()) {
        outcome_=used_recovery_?recovery_solver_.failureReason():solver_.failureReason();
        ROS_WARN_THROTTLE(1.,"[native planner] no trajectory: %s objects=%zu %s",outcome_,objects.objects.size(),diagnostics().c_str());
        return hold(ego);
      }
      previous_=planned;outcome_=used_recovery_?"native-generated-recovery":"native-generated";
      Message output;
      for(const auto& q:planned) {
        interfaces::PathPoint point;point.pose.pose.position.x=q.pose.position.x;point.pose.pose.position.y=q.pose.position.y;
        point.pose.pose.position.z=ego.pose.pose.position.z;point.pose.pose.orientation.z=std::sin(q.pose.yaw_rad/2);
        point.pose.pose.orientation.w=std::cos(q.pose.yaw_rad/2);point.velocity=q.speed_mps;point.acceleration=q.acceleration_mps2;
        point.time_from_start=ros::Duration(q.time_sec);
        if(!output.points.empty() && point.time_from_start<=output.points.back().time_from_start) {
          if(!output.points.back().time_from_start.isZero())output.points.back()=point;
        } else output.points.push_back(point);
      }
      if(p_.publish_lateral_reference && planned.front().spatial_reference)for(const auto& q:*planned.front().spatial_reference) {
        geometry_msgs::Pose point;point.position.x=q.position.x;point.position.y=q.position.y;point.position.z=ego.pose.pose.position.z;
        point.orientation.z=std::sin(q.yaw_rad/2);point.orientation.w=std::cos(q.yaw_rad/2);output.lateral_reference.push_back(point);
      }
      return output;
    } catch(const std::exception& e) {
      outcome_="calculation-domain";ROS_WARN_THROTTLE(1.,"[native planner] %s",e.what());previous_.clear();return hold(ego);
    }
  }
 private:
  static Message hold(const nav_msgs::Odometry& ego) {
    Message output;interfaces::PathPoint point;point.pose.pose=ego.pose.pose;point.velocity=0;point.acceleration=0;
    output.points.push_back(point);point.time_from_start=ros::Duration(5);output.points.push_back(point);return output;
  }
  bool dynamicBlocked(const std::vector<integrated_native::ReferencePoint>& reference,
      const integrated_native::EgoState& state,const interfaces::Objects& current,
      const PredictedObjects& future,double detection)const {
    double station=0,best=std::numeric_limits<double>::infinity(),origin=0;
    std::vector<double> stations(reference.size(),0);
    for(std::size_t i=1;i<reference.size();++i) {
      const auto a=reference[i-1].position,b=reference[i].position;const double length=std::hypot(b.x-a.x,b.y-a.y);
      stations[i]=stations[i-1]+length;
      const double d=std::hypot(a.x-state.pose.position.x,a.y-state.pose.position.y);
      if(d<best){best=d;origin=stations[i-1];}
    }
    const auto blocked=[&](const interfaces::Objects& objects) {
      for(const auto& object:objects.objects) {
        double nearest=std::numeric_limits<double>::infinity(),along=0,lateral=0,road_yaw=0,nearest_x=0,nearest_y=0;
        for(std::size_t i=1;i<reference.size();++i) {
          const auto a=reference[i-1].position,b=reference[i].position;const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
          if(length<1e-9)continue;
          const double raw=((object.bbox_pose.position.x-a.x)*dx+(object.bbox_pose.position.y-a.y)*dy)/(length*length);
          const double u=std::max(0.,std::min(1.,raw));
          const double ex=object.bbox_pose.position.x-a.x-u*dx,ey=object.bbox_pose.position.y-a.y-u*dy;
          const double distance=ex*ex+ey*ey;
          if(distance>=nearest)continue;
          // Keep true signed distance outside the finite reference ends. A
          // short predecessor must not project a distant rear object under us.
          const double signed_u=(i==1 && raw<0) || (i+1==reference.size() && raw>1) ? raw : u;
          nearest=distance;along=stations[i-1]+signed_u*length-origin;road_yaw=std::atan2(dy,dx);
          nearest_x=a.x+u*dx;nearest_y=a.y+u*dy;
          lateral=std::abs(-ex*dy+ey*dx)/length;
        }
        if(along < -p_.solver.vehicle.rear_m || along>detection)continue;
        const double relative=heading(object.bbox_pose.orientation)-road_yaw;
        const double half=(std::abs(std::sin(relative))*object.bbox_size.x+std::abs(std::cos(relative))*object.bbox_size.y)/2;
        if(lateral>p_.solver.vehicle.width_m/2+half+.25)continue;
        // Only a measured moving body already in the corridor inside the
        // stopping envelope forces a hold. Static bodies and later conflicts
        // are timed by the solver, which slows only as far as they require.
        double vx=object.velocity.x,vy=object.velocity.y;bool measured=object.velocity_available;
        if(!measured) {
          const auto inferred=std::find_if(future[0].objects.begin(),future[0].objects.end(),
              [&](const interfaces::Object& q){return q.id==object.id && q.velocity_available;});
          if(inferred!=future[0].objects.end()){vx=inferred->velocity.x;vy=inferred->velocity.y;measured=true;}
        }
        if(!measured || std::hypot(vx,vy)<p_.dynamic_zone_moving_speed_mps)continue;
        // A body clear of our footprint and moving out of the corridor has passed.
        const double side=-std::sin(road_yaw)*(object.bbox_pose.position.x-nearest_x)+
            std::cos(road_yaw)*(object.bbox_pose.position.y-nearest_y);
        const double outward=-std::sin(road_yaw)*vx+std::cos(road_yaw)*vy;
        if(lateral>p_.solver.vehicle.width_m/2+half && side*outward>0)continue;
        return true;
      }
      return false;
    };
    (void)station;
    return blocked(current);
  }

  static integrated_native::PathParameters recoveryParameters(const IntegratedDynamicTrajectoryParameters& p) {
    auto result=p.solver;
    // A measured departure must recover with the original hard 5 cm margin;
    // the larger nominal tracking reserve must not turn it into a crawl.
    result.road_margin_m=std::min(result.road_margin_m,.05);
    result.max_deceleration_mps2=p.recovery_deceleration_mps2;
    result.max_longitudinal_jerk_mps3=p.recovery_jerk_mps3;
    result.reentry_response_sec=p.recovery_response_sec;
    result.reentry_minimum_speed_mps=p.recovery_minimum_speed_mps;
    result.max_lateral_accel_mps2=p.recovery_lateral_acceleration_mps2;
    result.corridor_half_width_m=std::max(result.corridor_half_width_m,p.corridor_half_width_m);
    return result;
  }

  double referenceCurveTarget(const Message& path,double target) const {
    // Match the source curvature preview against the geometry actually tracked
    // laterally. The numerical trajectory may be straighter than this reference.
    // This is a cost target: measured v/a and hard collision/road limits stay intact.
    struct Sample {double x,y,s;};
    const double spacing=p_.curve_spacing_m;
    std::vector<Sample> samples;
    if(path.points.empty())return target;
    const auto& first=path.points.front().pose.pose.position;samples.push_back({first.x,first.y,0});
    double station=0,next=spacing;
    for(std::size_t i=1;i<path.points.size() && next<=p_.curve_preview_m+spacing+1e-9;++i) {
      const auto& a=path.points[i-1].pose.pose.position;const auto& b=path.points[i].pose.pose.position;
      const double length=std::hypot(b.x-a.x,b.y-a.y);
      if(length<1e-9)continue;
      while(next<=station+length+1e-9 && next<=p_.curve_preview_m+spacing+1e-9) {
        const double u=std::max(0.,std::min(1.,(next-station)/length));
        samples.push_back({a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),next});next+=spacing;
      }
      station+=length;
    }
    for(std::size_t i=1;i+1<samples.size();++i) {
      const auto& a=samples[i-1];const auto& b=samples[i];const auto& c=samples[i+1];
      const double denominator=std::hypot(b.x-a.x,b.y-a.y)*std::hypot(c.x-b.x,c.y-b.y)*std::hypot(c.x-a.x,c.y-a.y);
      if(denominator<1e-9)continue;
      const double curvature=2*std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/denominator;
      if(curvature<=.001)continue;
      const double curve=std::max(p_.minimum_curve_speed_mps,std::sqrt(p_.solver.preferred_lateral_accel_mps2/curvature));
      target=std::min(target,std::sqrt(curve*curve+2*p_.curve_deceleration_mps2*b.s));
    }
    return target;
  }

  static double heading(const geometry_msgs::Quaternion& q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  }
  static integrated_native::PredictedObject object(const interfaces::Object& input) {
    integrated_native::PredictedObject o;o.id=input.id;
    o.kind=input.label=="vehicle"?integrated_native::ObjectKind::Vehicle:
        input.label=="pedestrian"?integrated_native::ObjectKind::Pedestrian:integrated_native::ObjectKind::Unknown;
    o.pose={{input.position.x,input.position.y},heading(input.bbox_pose.orientation)};
    o.velocity_mps=input.velocity_available?integrated_native::Vec2{input.velocity.x,input.velocity.y}:integrated_native::Vec2{};
    o.length_m=input.bbox_size.x;o.width_m=input.bbox_size.y;
    return o;
  }
  static integrated_native::Predictions convert(const interfaces::Objects& current,const PredictedObjects& future,
      const IntegratedDynamicTrajectoryParameters& parameters,bool dynamic_zone) {
    integrated_native::Predictions result;result.reference_stamp_sec=current.header.stamp.toSec();
    for(const auto& input:current.objects) {
      auto o=object(input);o.source_stamp_sec=result.reference_stamp_sec;
      bool measured=input.velocity_available;
      if(!input.velocity_available) {
        const auto inferred=std::find_if(future[0].objects.begin(),future[0].objects.end(),
            [&](const interfaces::Object& q){return q.id==input.id && q.velocity_available;});
        if(inferred!=future[0].objects.end()){o.velocity_mps={inferred->velocity.x,inferred->velocity.y};measured=true;}
      }
      const bool unknown_walker=dynamic_zone && !measured && std::max(o.length_m,o.width_m)<=parameters.vulnerable_extent_m;
      result.current.push_back(o);
      for(std::size_t i=0;i<10;++i) {
        const auto found=std::find_if(future[i].objects.begin(),future[i].objects.end(),
            [&](const interfaces::Object& q){return q.id==input.id;});
        auto p=found==future[i].objects.end()?o:object(*found);
        p.source_stamp_sec=result.reference_stamp_sec;
        // The upstream solver models one rigid physical body. An enclosing
        // forecast OBB is uncertainty, not a different car at each time slice.
        p.radius_error_m=std::max({0.,(p.length_m-o.length_m)/2,(p.width_m-o.width_m)/2});
        p.length_m=o.length_m;p.width_m=o.width_m;p.kind=o.kind;p.model=o.model;
        // A walker can start, stop or turn within the forecast. Grow a moving
        // small body with prediction time so a pass is planned with clearance.
        if(std::max(o.length_m,o.width_m)<=parameters.vulnerable_extent_m && std::hypot(o.velocity_mps.x,o.velocity_mps.y)>=.3)
          p.radius_error_m+=std::min(parameters.vulnerable_uncertainty_max_m,parameters.vulnerable_uncertainty_mps*.5*(i+1));
        // A walker seen standing before its motion is measured is not a static
        // obstacle to steer around: it may step into either side of the lane.
        if(unknown_walker)p.radius_error_m+=std::min(parameters.unknown_walker_max_m,parameters.unknown_walker_speed_mps*.5*(i+1));
        result.future[i].push_back(p);
      }
    }
    return result;
  }

  const char* outcome_="native-search";
  IntegratedDynamicTrajectoryParameters p_;
  integrated_native::IntegratedSolver solver_,recovery_solver_;
  const integrated_native::IntegratedSolver initial_solver_,initial_recovery_solver_;
  bool used_recovery_=false;
  std::vector<integrated_native::PathPoint> previous_;
  double last_stamp_=0,last_blocked_sec_=-std::numeric_limits<double>::infinity();
  bool has_last_position_=false;
  integrated_native::Vec2 last_position_;
  double last_position_stamp_=0;
};
 public:
  IntegratedDynamic(IntegratedDynamicParameters p,SharedMap& map):mission_parameters_(p.mission),
      dynamic_zone_speed_mps_(p.trajectory.dynamic_zone_speed_mps),dynamic_zone_approach_m_(p.trajectory.dynamic_zone_approach_m),
      trajectory_(p.trajectory,*map.map) {}
  const char* planningOutcome()const{return trajectory_.outcome();}
  std::string planningDiagnostics()const{return trajectory_.diagnostics();}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& status,
      const interfaces::Objects& objects,const tf2_msgs::TFMessage& tf,const sensor_msgs::Imu& imu,
      const Route& route,const PredictedObjects& predictions,const ros::Time& stamp) {
    (void)tf;
    const auto& p=mission_parameters_;const auto& q=ego.pose.pose.position;
    const bool inside=p.dynamic_zone_enabled && q.x>=p.dynamic_zone_min_x_m && q.x<=p.dynamic_zone_max_x_m &&
        q.y>=p.dynamic_zone_min_y_m && q.y<=p.dynamic_zone_max_y_m;
    const bool approach=p.dynamic_zone_enabled && q.x>=p.dynamic_zone_min_x_m && q.x<=p.dynamic_zone_max_x_m &&
        q.y>=p.dynamic_zone_min_y_m-dynamic_zone_approach_m_ && q.y<=p.dynamic_zone_max_y_m;
    const double speed=std::abs(status.velocity_x_mps);
    const double detection=std::max(p.dynamic_zone_minimum_detection_m,speed*p.dynamic_zone_reaction_sec+
        speed*speed/(2*p.dynamic_zone_braking_mps2)+p.vehicle_front_m+p.stop_buffer_m);
    return trajectory_.calculate(ego,status,objects,imu,route,predictions,stamp,
        approach?dynamic_zone_speed_mps_:std::numeric_limits<double>::infinity(),inside,detection,p.dynamic_zone_clear_sec);
  }
 private:
  PathPlannerMissionParameters mission_parameters_;
  double dynamic_zone_speed_mps_,dynamic_zone_approach_m_;
  IntegratedTrajectory trajectory_;
};
}
}  // namespace integrated_dynamic_detail

namespace path_planner {
using IntegratedDynamic = ::integrated_dynamic_detail::path_planner::IntegratedDynamic;
}  // namespace path_planner
