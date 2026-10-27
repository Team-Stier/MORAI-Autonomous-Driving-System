// Frozen mission behavior from local workspace 69419e3.
#pragma once
#include <lanelet2_core/LaneletMap.h>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <ros/console.h>
#include <node_pipeline/extension.hpp>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <array>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace path_planner {
struct PathPlannerStaticMissionParameters {
  bool reference_smoothing_enabled=false;
  bool dynamic_zone_enabled = false;
  double dynamic_zone_min_x_m=-75, dynamic_zone_max_x_m=-40;
  double dynamic_zone_min_y_m=-100, dynamic_zone_max_y_m=85;
  double dynamic_zone_minimum_detection_m=20, dynamic_zone_reaction_sec=.3;
  double dynamic_zone_braking_mps2=6, dynamic_zone_clear_sec=.5;
  double tracking_margin_m=.3;
  double minimum_obstacle_extent_m=0;
  std::vector<lanelet::BasicLineString3d> solid_boundaries;
  bool use_boundary_midpoint_centerline = false;
  double centerline_step_m = 0.25;
  double local_path_length_m = 50;
  double sample_step_m = 0.5;
  double wheelbase_m = 3.0;
  double steering_gain = .77;
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

// Static-obstacle route extraction and Lanelet lane changes for section 4.
class PathPlannerStaticMission {
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
  PathPlannerStaticMission(PathPlannerStaticMissionParameters parameters, lanelet::LaneletMap& map)
      : parameters_(std::move(parameters)), planning_length_m_(parameters_.local_path_length_m),
        rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle)),
        graph_(lanelet::routing::RoutingGraph::build(map, *rules_)) {}

  Message process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                  const interfaces::Objects& objects, const Route& route) {
    const double speed=std::abs(competition.velocity_x_mps);
    planning_length_m_=std::max(parameters_.local_path_length_m,
        speed*std::sqrt(6*parameters_.maximum_lane_offset_m/parameters_.maximum_lateral_acceleration_mps2)+
        parameters_.vehicle_front_m+parameters_.stop_buffer_m);
    reference_lane_change_=false;
    updateTracks(objects, ego.header.stamp.toSec());
    publishMotion(objects);
    Message current;
    if (!lane_change_.points.empty()) {
      current = trim(lane_change_, ego.pose.pose.position);
      const auto& q=current.points.front().pose.pose.position;
      const auto& p=ego.pose.pose.position;
      if(std::hypot(q.x-p.x,q.y-p.y)>parameters_.maximum_lane_offset_m) {
        lane_change_.points.clear();normal_lane_change_=false;continuation_route_=Route{};
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
        ROS_INFO_STREAM("[path_planner_static] " <<
                        (normal_lane_change_?"route lane change completed":"avoidance return completed"));
        normal_lane_change_=false;
      } else if (distances(current).back()<parameters_.stop_buffer_m) {
        report("avoidance_end_stop");
        return stopped(std::move(current),0);
      }
    }
    if (lane_change_.points.empty()) current = reference(ego,route);
    const auto& position=ego.pose.pose.position;
    const bool dynamic_zone=parameters_.dynamic_zone_enabled &&
        position.x>=parameters_.dynamic_zone_min_x_m && position.x<=parameters_.dynamic_zone_max_x_m &&
        position.y>=parameters_.dynamic_zone_min_y_m && position.y<=parameters_.dynamic_zone_max_y_m;
    if(dynamic_zone) {
      const double detection=std::max(parameters_.dynamic_zone_minimum_detection_m,
          speed*parameters_.dynamic_zone_reaction_sec+speed*speed/(2*parameters_.dynamic_zone_braking_mps2)+
          parameters_.vehicle_front_m+parameters_.stop_buffer_m);
      bool blocked=false;
      for(const auto& entry:tracks_)if(firstCollision(current,entry.second)<=detection){blocked=true;break;}
      const double now=ego.header.stamp.toSec();
      if(blocked)dynamic_zone_last_blocked_sec_=now;
      if(blocked || now-dynamic_zone_last_blocked_sec_<parameters_.dynamic_zone_clear_sec) {
        for(auto& point:current.points){point.velocity=0;point.acceleration=0;}
        report("dynamic_zone_emergency_stop");return current;
      }
      report("dynamic_zone_clear");return current;
    }
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
    // Refit a committed maneuver only when a new scan consumes its reserved
    // clearance. Keep the measured box and the existing collision test intact.
    if(!lane_change_.points.empty() && !normal_lane_change_ &&
        objects.header.stamp.toSec()!=last_maneuver_check_stamp_) {
      last_maneuver_check_stamp_=objects.header.stamp.toSec();
      bool tight=false;
      for(const auto& entry:tracks_) {
        auto envelope=entry.second;
        envelope.object.bbox_size.x+=parameters_.tracking_margin_m;
        envelope.object.bbox_size.y+=parameters_.tracking_margin_m;
        if(std::isfinite(firstCollision(current,envelope))){tight=true;break;}
      }
      if(tight) {
        auto updated=changeLane(current,0,ego,competition);
        if(!updated.points.empty()) {
          limitManeuverSpeed(updated);
          lane_change_=updated;current=std::move(updated);
          report("avoidance_geometry_updated");
        }
      }
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
  static double headingOf(const geometry_msgs::Pose& p) {
    return std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                      1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));
  }
  void report(const std::string& mode) {
    if (mode != mode_) { ROS_INFO_STREAM("[path_planner_static] " << mode); mode_ = mode; }
  }
  void updateTracks(const interfaces::Objects& objects, double now) {
    const double stamp = objects.header.stamp.toSec();
    for (const auto& object : objects.objects) {
      auto& track = tracks_[object.id];
      if (stamp <= track.last_stamp) continue;
      if (stamp - track.last_stamp > parameters_.track_expiry_sec) track = Track{};
      // Section 4 contains static obstacles; do not wait for motion estimates.
      track.moving = false;
      track.stationary = true;
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
  // Sweep the vehicle OBB over each path segment using all four SAT axes.
  double firstCollision(const Message& path,const Track& track) const {
    const auto& object=track.object;const auto station=distances(path);
    const double object_yaw=headingOf(object.bbox_pose),oc=std::cos(object_yaw),os=std::sin(object_yaw);
    const double center=(parameters_.vehicle_front_m-parameters_.vehicle_rear_m)/2;
    const double half_length=(parameters_.vehicle_front_m+parameters_.vehicle_rear_m)/2;
    const double half_width=parameters_.vehicle_half_width_m+parameters_.collision_margin_m;
    const unsigned steps=track.stationary || !object.velocity_available?0:static_cast<unsigned>(std::ceil(parameters_.dynamic_prediction_sec/.25));
    double collision=std::numeric_limits<double>::infinity();
    for(std::size_t i=1;i<path.points.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position;const auto& b=path.points[i].pose.pose.position;
      const double length=std::hypot(b.x-a.x,b.y-a.y);if(length==0)continue;
      const double c=(b.x-a.x)/length,s=(b.y-a.y)/length;
      for(unsigned k=0;k<=steps;++k) {
        const double x=object.bbox_pose.position.x+(steps?object.velocity.x*.25*k:0)-a.x-center*c;
        const double y=object.bbox_pose.position.y+(steps?object.velocity.y*.25*k:0)-a.y-center*s;
        double lo=0,hi=length;
        for(const auto& axis:std::array<std::array<double,2>,4>{{{{c,s}},{{-s,c}},{{oc,os}},{{-os,oc}}}}) {
          const double forward=axis[0]*c+axis[1]*s,lateral=-axis[0]*s+axis[1]*c;
          const double radius=half_length*std::abs(forward)+half_width*std::abs(lateral)+
              .5*object.bbox_size.x*std::abs(axis[0]*oc+axis[1]*os)+.5*object.bbox_size.y*std::abs(-axis[0]*os+axis[1]*oc);
          const double d=axis[0]*x+axis[1]*y;
          if(std::abs(forward)<1e-10){if(std::abs(d)>radius){lo=length+1;break;}}
          else {double begin=(d-radius)/forward,end=(d+radius)/forward;if(begin>end)std::swap(begin,end);lo=std::max(lo,begin);hi=std::min(hi,end);}
          if(lo>hi)break;
        }
        if(lo<=hi)collision=std::min(collision,station[i-1]+lo);
      }
    }
    return collision;
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
  bool clearsSolidBoundaries(const Message& path,double begin_m,double end_m) const {
    const auto station=distances(path);
    for(std::size_t i=0;i<path.points.size() && station[i]<=end_m;++i) {
      if(station[i]<begin_m)continue;
      const auto& pose=path.points[i].pose.pose;const auto& p=pose.position;
      const double yaw=headingOf(pose),c=std::cos(yaw),s=std::sin(yaw);
      const double half=parameters_.vehicle_half_width_m+parameters_.tracking_margin_m;
      const auto crosses=[&](const lanelet::BasicLineString3d& bound) {
        for(std::size_t k=1;k<bound.size();++k) {
          const auto local=[&](const lanelet::BasicPoint3d& q) {
            return lanelet::BasicPoint2d((q.x()-p.x)*c+(q.y()-p.y)*s,-(q.x()-p.x)*s+(q.y()-p.y)*c);
          };
          const auto a=local(bound[k-1]),b=local(bound[k]);
          const auto delta=(b-a).eval();
          double lo=0,hi=1;
          for(unsigned axis=0;axis<2;++axis) {
            const double minimum=axis==0?-parameters_.vehicle_rear_m:-half;
            const double maximum=axis==0?parameters_.vehicle_front_m:half;
            if(std::abs(delta[axis])<1e-10){if(a[axis]<minimum || a[axis]>maximum){lo=1;hi=0;break;}}
            else {double first=(minimum-a[axis])/delta[axis],last=(maximum-a[axis])/delta[axis];if(first>last)std::swap(first,last);lo=std::max(lo,first);hi=std::min(hi,last);}
          }
          if(lo<=hi)return true;
        }
        return false;
      };
      for(const auto& bound:parameters_.solid_boundaries)if(crosses(bound))return false;
    }
    return true;
  }
  Message changeLane(const Message& base,double,const nav_msgs::Odometry& ego,
                     const interfaces::CompetitionVehicleStatus& competition) {
    bool neighbor=false;
    for(const auto& span:reference_spans_)neighbor=neighbor||static_cast<bool>(graph_->right(span.lane));
    if(!neighbor)return {};
    Message candidate=resample(base);const auto station=distances(candidate);
    const std::size_t n=candidate.points.size();
    std::vector<double> lower(n,0),upper(n,parameters_.maximum_lane_offset_m),offset(n,0);
    // A box corridor uses original painted boundaries, excluding transverse stop lines.
    for(std::size_t i=0;i<n;++i) {
      const auto& pose=candidate.points[i].pose.pose;const double yaw=headingOf(pose);
      bool found=false;
      for(double d=-.6;d<=parameters_.maximum_lane_offset_m+.001;d+=.1) {
        auto point=candidate.points[i];point.pose.pose.position.x+=d*std::sin(yaw);point.pose.pose.position.y-=d*std::cos(yaw);
        Message probe;probe.points.push_back(point);
        const bool clear=clearsSolidBoundaries(probe,0,0);
        if(clear) {if(!found){lower[i]=d;found=true;}upper[i]=d;}
        else if(found)break;
      }
      if(!found || lower[i]>parameters_.vehicle_half_width_m)return {};
    }
    // Reserve unseen static faces before the first thin observation grows.
    // Measured objects and the final geometric collision test remain unchanged.
    // Each obstacle imposes a right-side clearance over the complete vehicle footprint.
    for(const auto& entry:tracks_) {
      auto envelope=entry.second;
      envelope.object.bbox_size.x+=parameters_.tracking_margin_m;
      envelope.object.bbox_size.y+=parameters_.tracking_margin_m;
      if(!std::isfinite(firstCollision(base,envelope)))continue;
      const auto& object=entry.second.object;
      const double location=signedStation(base,object.bbox_pose.position.x,object.bbox_pose.position.y);
      const auto road=sample(candidate,station,location);const double angle=headingOf(object.bbox_pose)-road.yaw;
      const double half_length=std::max(parameters_.minimum_obstacle_extent_m/2,
          (std::abs(std::cos(angle))*object.bbox_size.x+std::abs(std::sin(angle))*object.bbox_size.y)/2);
      const double half_width=std::max(parameters_.minimum_obstacle_extent_m/2,
          (std::abs(std::sin(angle))*object.bbox_size.x+std::abs(std::cos(angle))*object.bbox_size.y)/2);
      const double right=(object.bbox_pose.position.x-road.x)*std::sin(road.yaw)-(object.bbox_pose.position.y-road.y)*std::cos(road.yaw);
      const double needed=std::max(0.0,right+half_width+parameters_.vehicle_half_width_m+
          parameters_.collision_margin_m+parameters_.tracking_margin_m);
      for(std::size_t i=0;i<n;++i)if(station[i]>=location-half_length-parameters_.vehicle_front_m-parameters_.collision_margin_m &&
          station[i]<=location+half_length+parameters_.vehicle_rear_m+parameters_.collision_margin_m)lower[i]=std::max(lower[i],needed);
    }
    for(std::size_t i=0;i<std::min<std::size_t>(3,n);++i) {
      lower[i]=upper[i]=0;lower[n-1-i]=upper[n-1-i]=0;
    }
    for(std::size_t i=0;i<n;++i){if(lower[i]>upper[i])return {};offset[i]=lower[i];}
    // Box-constrained, one-dimensional minimum-bending fit; no additional ROS state or Stage.
    std::vector<std::array<double,5>> matrix(n);
    for(auto& row:matrix)row.fill(0);
    for(std::size_t i=0;i<n;++i)matrix[i][2]=.01;
    for(std::size_t i=0;i+1<n;++i) {
      matrix[i][2]+=10;matrix[i+1][2]+=10;matrix[i][3]-=10;matrix[i+1][1]-=10;
    }
    for(std::size_t i=0;i+2<n;++i)for(unsigned a=0;a<3;++a)for(unsigned b=0;b<3;++b) {
      const double ca=a==1?-2:1,cb=b==1?-2:1;const double length=std::min(parameters_.transition_length_m,parameters_.return_transition_length_m);
      matrix[i+a][2+b-a]+=10000*std::pow(length/20,4)*ca*cb;
    }
    const double penalty=20;
    std::vector<Eigen::Triplet<double>> entries;
    for(std::size_t i=0;i<n;++i)for(int d=-2;d<=2;++d)
      if(static_cast<long>(i)+d>=0 && static_cast<long>(i)+d<static_cast<long>(n))
        entries.emplace_back(i,static_cast<long>(i)+d,matrix[i][2+d]+(d==0?penalty:0));
    Eigen::SparseMatrix<double> system(n,n);system.setFromTriplets(entries.begin(),entries.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factor(system);
    Eigen::VectorXd z=Eigen::VectorXd::Zero(n),dual=z;
    for(unsigned iteration=0;iteration<1500;++iteration) {
      const Eigen::VectorXd fit=factor.solve(penalty*(z-dual));
      double error=0;
      for(std::size_t i=0;i<n;++i) {
        const double previous=z[i];z[i]=std::max(lower[i],std::min(upper[i],fit[i]+dual[i]));
        dual[i]+=fit[i]-z[i];error=std::max(error,std::max(std::abs(fit[i]-z[i]),std::abs(z[i]-previous)));
      }
      if(error<1e-5)break;
    }
    for(std::size_t i=0;i<n;++i)offset[i]=z[i];
    for(std::size_t i=0;i<n;++i) {
      auto& pose=candidate.points[i].pose.pose;const double yaw=headingOf(pose);
      pose.position.x+=offset[i]*std::sin(yaw);pose.position.y-=offset[i]*std::cos(yaw);
      candidate.points[i].velocity=parameters_.avoidance_speed_mps;
    }
    headings(candidate);candidate=rearAxleConnection(candidate,ego,competition);
    if(candidate.points.empty() || !drivable(candidate) || !clearsSolidBoundaries(candidate,2,station.back()))return {};
    for(const auto& entry:tracks_)if(std::isfinite(firstCollision(candidate,entry.second)))return {};
    handover_m_=distances(candidate).back()-parameters_.stop_buffer_m;
    return candidate;
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
    const double curvature=-std::tan(parameters_.steering_gain*competition.steering_angle_rad)/parameters_.wheelbase_m;
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
      // A failed join must not discard other feasible connections. Check the
      // complete body envelope before selecting this C2 connection.
      if(!drivable(result) || !clearsSolidBoundaries(result,2,distances(result).back())) continue;
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
  // The switch preserves the original Lanelet2 centerline exactly when disabled.
  lanelet::BasicLineString3d smoothReference(const lanelet::ConstLanelet& lane,
                                            lanelet::BasicLineString3d source) const {
    if(!parameters_.reference_smoothing_enabled)return source;
    constexpr double sigma=1.0, step=.25, maximum_shift=.20;
    auto before=graph_->previous(lane,false),after=graph_->following(lane,false);
    lanelet::BasicLineString3d prefix,suffix;
    double first_gap=.4,last_gap=.4;
    for(const auto& other:before) {
      const auto line=rawReference(other);
      const double gap=(line.back()-source.front()).head<2>().norm();
      if(gap<first_gap){first_gap=gap;prefix=line;}
    }
    for(const auto& other:after) {
      const auto line=rawReference(other);
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
      for(int j=-8;j<=8;++j) {
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
    return centerlines_.emplace(key,smoothReference(lane,rawReference(lane))).first->second;
  }
  lanelet::BasicLineString3d rawReference(const lanelet::ConstLanelet& lane) const {
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
      if (segment_m < (parameters_.reference_smoothing_enabled?.1:1e-12)) continue;
      const double take_m = std::min(segment_m, planning_length_m_ - length_m);
      appendPoint(message, from + (take_m / segment_m) * delta);
      length_m += take_m;
    }
  }


  PathPlannerStaticMissionParameters parameters_;
  double planning_length_m_;
  lanelet::traffic_rules::TrafficRulesPtr rules_;
  lanelet::routing::RoutingGraphUPtr graph_;
  std::vector<LaneSpan> reference_spans_;
  std::map<std::pair<lanelet::Id,bool>,lanelet::BasicLineString3d> centerlines_;
  std::unordered_map<std::uint32_t,Track> tracks_;
  node_pipeline::Visualizer<visualization_msgs::Marker> motion_visualizer_{"object_motion"};
  double dynamic_zone_last_blocked_sec_=-std::numeric_limits<double>::infinity();
  Message lane_change_;
  double handover_m_=0,reference_handover_m_=0;
  bool normal_lane_change_=false,reference_lane_change_=false;
  Route continuation_route_;
  std::size_t reference_end_index_=0,continuation_index_=0;
  double last_maneuver_check_stamp_=-1;
  std::string mode_;
};
}  // namespace path_planner
