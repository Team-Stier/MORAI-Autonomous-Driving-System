#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <lanelet2_core/LaneletMap.h>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <node_pipeline/extension.hpp>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <array>
#include <deque>
#include <limits>
#include <map>

// ---- inlined from libraries/integrated_highway_mission.hpp ----
//  highway mission with workspace merge timing and gap tuning.
namespace path_planner {
struct HighwayMissionParameters {
  bool reference_smoothing_enabled=true;
  double local_path_length_m=120,sample_step_m=.5,cruise_speed_mps=100.0/3.6;
  double transition_length_m=35,maximum_curvature_per_m=.12,maximum_lateral_acceleration_mps2=2.0;
  double maximum_lane_offset_m=6,maximum_join_gap_m=1.5;
  double vehicle_front_m=3.845008,vehicle_rear_m=.79,vehicle_half_width_m=1.080418;
  double front_gap_m=8,rear_gap_m=15,gap_confirmation_sec=.5;
  double entry_speed_mps=60.0/3.6,highway_end_y_m=-217.855;
  double rear_headway_sec=1,rear_clear_sec=2;
  double mission_end_y_m=-217.855;unsigned mission_change_count=3;
  double completion_lateral_m=.2,completion_heading_rad=.05;
  double acc_acceleration_mps2=1,acc_deceleration_mps2=1.5;
  double following_gap_recovery_rate_mps=.5;
  double following_gap_m=20,following_time_sec=1,stopping_deceleration_mps2=3,stop_buffer_m=3;
};
class HighwayMission {
  struct Pose {double x,y,z,yaw;};
  struct Projection {lanelet::BasicPoint3d point;std::size_t segment=0;double squared_distance=std::numeric_limits<double>::infinity();};
  struct LaneSpan {lanelet::ConstLanelet lane;double begin_m,end_m;};
 public:
  HighwayMission(HighwayMissionParameters parameters,lanelet::LaneletMap& map):parameters_(std::move(parameters)),
      planning_length_m_(parameters_.local_path_length_m),rules_(lanelet::traffic_rules::TrafficRulesFactory::create(
          lanelet::Locations::Germany,lanelet::Participants::Vehicle)),graph_(lanelet::routing::RoutingGraph::build(map,*rules_)) {}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& competition,
                  const interfaces::Objects& observations,const Route& route) {
    const auto objects=motionObservations(observations);
    publishMotion(objects);
    const double speed=std::abs(competition.velocity_x_mps),now=ego.header.stamp.toSec();
    const auto& position=ego.pose.pose.position;
    active_limit_mps_=speedLimit(ego);planning_speed_mps_=std::max(speed,active_limit_mps_);
    planning_length_m_=std::max(parameters_.local_path_length_m,planning_speed_mps_*8);
    Message path;
    if(!maneuver_.points.empty()) {
      path=trim(maneuver_,position);
      const auto& closest=path.points.front().pose.pose;
      const double angle=headingOf(ego.pose.pose)-headingOf(closest);
      if(signedStation(maneuver_,position.x,position.y)>=handover_m_ &&
          std::hypot(closest.position.x-position.x,closest.position.y-position.y)<maneuver_completion_lateral_m_ &&
          std::abs(std::atan2(std::sin(angle),std::cos(angle)))<parameters_.completion_heading_rad) {
        maneuver_.points.clear();gap_since_=-1;++completed_changes_;active_limit_mps_=speedLimit(ego);
        planning_speed_mps_=std::max(speed,active_limit_mps_);
        ROS_INFO("[highway] lane change completed count=%u",completed_changes_);
      } else {follow(path,objects,speed,now);for(auto& point:path.points)point.velocity=std::min(point.velocity,maneuver_speed_mps_);capManeuverEnd(path);publishStatus(ego,"CHANGING");return path;}
    }
    reference_lane_change_=false;change_window_exhausted_=false;const auto desired=missionRoute(ego,route);path=reference(ego,desired);
    if(path.points.size()==1) {
      auto end=path.points.front();const double yaw=headingOf(ego.pose.pose);
      end.pose.pose.position.x+=parameters_.sample_step_m*std::cos(yaw);
      end.pose.pose.position.y+=parameters_.sample_step_m*std::sin(yaw);path.points.push_back(end);
      for(auto& point:path.points){point.velocity=0;point.acceleration=0;}
      publishStatus(ego,"LANE_END");return path;
    }
    if(change_window_exhausted_) {
      for(auto& point:path.points){point.velocity=0;point.acceleration=0;}
      gap_since_=-1;publishStatus(ego,"MERGE_WAIT");return path;
    }
    if(reference_lane_change_) {
      const bool speed_ready=speed<=reference_change_speed_mps_+.5;
      if(!speed_ready) {gap_reason_="slowing for merge";blocking_id_=0;}
      const bool gap_clear=speed_ready && gapClear(ego,objects,speed,now,path);
      if(gap_clear) {
        if(gap_since_<0)gap_since_=now;
        if(now-gap_since_>=parameters_.gap_confirmation_sec) {
          maneuver_=path;handover_m_=reference_handover_m_;
          maneuver_completion_lateral_m_=reference_completion_lateral_m_;
          maneuver_speed_mps_=std::min(active_limit_mps_,reference_change_speed_mps_);
          ROS_INFO("[highway] lane change committed target=%lld",static_cast<long long>(target_lane_id_));
          follow(path,objects,speed,now);for(auto& point:path.points)point.velocity=std::min(point.velocity,maneuver_speed_mps_);capManeuverEnd(path);publishStatus(ego,"CHANGING");return path;
        }
      } else gap_since_=-1;
      const double alignment_speed=(!gap_clear && speed_ready)?mergeBehindSpeed(ego,objects,speed,now):reference_change_speed_mps_;
      path=waiting_path_;follow(path,objects,speed,now,alignment_speed);
      const auto stations=distances(path);
      for(std::size_t i=0;i<path.points.size();++i)path.points[i].velocity=std::min(path.points[i].velocity,
          std::sqrt(2*parameters_.stopping_deceleration_mps2*std::max(0.0,waiting_stop_station_m_-stations[i]-parameters_.stop_buffer_m)));
      for(auto& point:path.points)point.velocity=std::min(point.velocity,reference_change_speed_mps_);
      publishStatus(ego,"GAP_WAIT");return path;
    }
    gap_since_=-1;follow(path,objects,speed,now);publishStatus(ego,"KEEP_LANE");return path;
  }
 private:
  struct MotionSample {double time,x,y,length,width;};
  struct GapTarget {double distance,time;};
  interfaces::Objects motionObservations(const interfaces::Objects& observed) {
    const double now=observed.header.stamp.toSec();
    if(now<last_observation_sec_) motion_history_.clear();
    last_observation_sec_=now;
    for(auto it=motion_history_.begin();it!=motion_history_.end();)
      if(now-it->second.back().time>.8)it=motion_history_.erase(it);else ++it;
    auto result=observed;
    for(auto& object:result.objects) {
      auto& samples=motion_history_[object.id];
      if(!samples.empty() && now-samples.back().time>.28)samples.clear();
      if(samples.empty() || now>samples.back().time)
        samples.push_back({now,object.bbox_pose.position.x,object.bbox_pose.position.y,
                           object.bbox_size.x,object.bbox_size.y});
      while(samples.size()>1 && now-samples.front().time>.5)samples.pop_front();
      if(samples.size()<3 || samples.back().time-samples.front().time<.15)continue;
      std::vector<double> vx,vy;
      for(std::size_t i=0;i<samples.size();++i)for(std::size_t j=i+1;j<samples.size();++j) {
        const double dt=samples[j].time-samples[i].time;
        if(dt<.09 || std::abs(samples[j].length-samples[i].length)>.5 ||
            std::abs(samples[j].width-samples[i].width)>.5)continue;
        vx.push_back((samples[j].x-samples[i].x)/dt);vy.push_back((samples[j].y-samples[i].y)/dt);
      }
      if(vx.size()<2)continue;
      const auto median=[](std::vector<double> v){std::sort(v.begin(),v.end());const auto n=v.size();return (v[(n-1)/2]+v[n/2])/2;};
      const double fitted_x=median(vx),fitted_y=median(vy);
      if(object.velocity_available) {
        // A physically abrupt reported velocity change must agree with the
        // same track's observed displacement before replacing its motion.
        const double fitted_speed=std::hypot(fitted_x,fitted_y);
        if(fitted_speed<=3 || (object.velocity.x*fitted_x+object.velocity.y*fitted_y)/fitted_speed>=fitted_speed-3)continue;
        for(auto& v:vx)v=std::abs(v-fitted_x);
        for(auto& v:vy)v=std::abs(v-fitted_y);
        if(median(vx)>1.5 || median(vy)>1.5)continue;
        ROS_WARN_THROTTLE(1.,"[highway] velocity disagrees with stable position history id=%u reported=(%.2f,%.2f) fitted=(%.2f,%.2f)",
            object.id,object.velocity.x,object.velocity.y,fitted_x,fitted_y);
      }
      object.velocity.x=fitted_x;object.velocity.y=fitted_y;object.velocity.z=0;
      object.velocity_available=true;
    }
    return result;
  }
  double rearClearance(double half,double other,double ego_speed) const {
    return parameters_.vehicle_rear_m+half+
        std::max(parameters_.rear_gap_m,std::max(0.,other)*parameters_.rear_headway_sec)+
        std::max(0.,other-ego_speed)*parameters_.rear_clear_sec;
  }
  double speedLimit(const nav_msgs::Odometry& ego) const {
    if(completed_changes_==0)return parameters_.entry_speed_mps;
    const double remaining=std::max(0.0,ego.pose.pose.position.y-parameters_.highway_end_y_m);
    return std::min(parameters_.cruise_speed_mps,std::sqrt(parameters_.entry_speed_mps*parameters_.entry_speed_mps+
        2*parameters_.stopping_deceleration_mps2*remaining));
  }
  Route missionRoute(const nav_msgs::Odometry& ego,const Route& global) const {
    if(ego.pose.pose.position.y<=parameters_.mission_end_y_m)return global;
    const auto& route=*global;std::size_t nearest=0;double distance=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<std::min<std::size_t>(2,route.size());++i) {
      const auto q=project(route[i].centerline3d().basicLineString(),{ego.pose.pose.position.x,ego.pose.pose.position.y});
      if(q.squared_distance<distance){distance=q.squared_distance;nearest=i;}
    }
    lanelet::ConstLanelets lanes{route[nearest]};
    if(completed_changes_<parameters_.mission_change_count) {
      const auto left=graph_->left(lanes.back());if(left)lanes.push_back(*left);
    }
    double length=0;
    for(unsigned step=0;step<40 && length<planning_length_m_*2;++step) {
      const auto next=graph_->following(lanes.back(),false);if(next.empty())break;
      const auto tail=lanes.back().centerline3d().basicLineString();const auto direction=(tail.back()-tail[tail.size()-2]).head<2>().normalized().eval();
      auto chosen=next.front();double best=-2;
      for(const auto& candidate:next) {
        const auto line=candidate.centerline3d().basicLineString();const double aligned=direction.dot((line[1]-line[0]).head<2>().normalized());
        if(aligned>best){best=aligned;chosen=candidate;}
      }
      const auto line=chosen.centerline3d().basicLineString();for(std::size_t i=1;i<line.size();++i)length+=(line[i]-line[i-1]).head<2>().norm();
      if(std::find(lanes.begin(),lanes.end(),chosen)!=lanes.end())break;
      lanes.push_back(chosen);
    }
    return lanelet::routing::LaneletPath(lanes);
  }
  struct Coordinate {double station,lateral,yaw;};
  static Coordinate coordinate(const Message& path,double x,double y) {
    lanelet::BasicLineString3d line;for(const auto& p:path.points)line.push_back({p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z});
    const auto projected=project(line,{x,y});const auto& a=line[projected.segment];const auto d=(line[projected.segment+1]-a).head<2>().eval();
    const double yaw=std::atan2(d.y(),d.x());
    return {signedStation(path,x,y),(-(x-a.x())*d.y()+(y-a.y())*d.x())/d.norm(),yaw};
  }
  bool gapClear(const nav_msgs::Odometry& ego,const interfaces::Objects& objects,double speed,double now,
                const Message& candidate) {
    blocking_id_=0;gap_reason_="gap clear";
    const auto origin=coordinate(target_path_,ego.pose.pose.position.x,ego.pose.pose.position.y);
    // Use one longitudinal schedule for footprint occupancy and gap prediction.
    // A stopped ego must accelerate; assigning it 5 m/s underestimates merge time.
    struct Occupancy {double time,progress,speed,lateral,half_width;};
    std::vector<Occupancy> occupancy;
    const auto station=distances(candidate);
    const double extent=std::min(station.back(),change_begin_m_+transition_m_);
    const double limit=std::min(active_limit_mps_,reference_change_speed_mps_);
    double velocity=std::max(0.,std::min(speed,limit)),progress=0.,time=0.;
    const double center_offset=(parameters_.vehicle_front_m-parameters_.vehicle_rear_m)/2;
    const double half_length=(parameters_.vehicle_front_m+parameters_.vehicle_rear_m)/2;
    while(true) {
      const auto pose=sample(candidate,station,progress);
      const auto q=coordinate(target_path_,pose.x+center_offset*std::cos(pose.yaw),
                             pose.y+center_offset*std::sin(pose.yaw));
      const double turn=pose.yaw-q.yaw;
      occupancy.push_back({time,coordinate(target_path_,pose.x,pose.y).station-origin.station,velocity,q.lateral,
          half_length*std::abs(std::sin(turn))+parameters_.vehicle_half_width_m*std::abs(std::cos(turn))});
      if(progress>=extent)break;
      // Reserve response delay and 20% acceleration margin instead of relying
      // on the controller reaching its configured acceleration immediately.
      const double next=time<.8?velocity:std::min(limit,velocity+.08*parameters_.acc_acceleration_mps2);
      progress=std::min(extent,progress+.05*(velocity+next));velocity=next;time+=.1;
    }
    for(const auto& object:objects.objects) {
      const auto q=coordinate(target_path_,object.bbox_pose.position.x,object.bbox_pose.position.y);
      const double yaw=headingOf(object.bbox_pose)-q.yaw;
      const double width=(std::abs(std::sin(yaw))*object.bbox_size.x+std::abs(std::cos(yaw))*object.bbox_size.y)/2;
      if(std::abs(q.lateral)>parameters_.vehicle_half_width_m+width+.25)continue;
      const double half=(std::abs(std::cos(yaw))*object.bbox_size.x+std::abs(std::sin(yaw))*object.bbox_size.y)/2;
      if(std::none_of(occupancy.begin(),occupancy.end(),[&](const Occupancy& sample) {
        return std::abs(sample.lateral-q.lateral)<=sample.half_width+width+.25;
      }))continue;
      const double age=std::max(0.0,now-objects.header.stamp.toSec());
      const double other=object.velocity_available?object.velocity.x*std::cos(q.yaw)+object.velocity.y*std::sin(q.yaw):
          (q.station<origin.station?parameters_.cruise_speed_mps:0.0);
      const double initial=q.station-origin.station;
      // Present clearance and future closing clearance are separate checks.
      // Closing is evaluated against ego's speed at the same future instant.
      const double present_rear=parameters_.vehicle_rear_m+half+
          std::max(parameters_.rear_gap_m,std::max(0.,other)*parameters_.rear_headway_sec);
      if(initial+other*age<0 && -(initial+other*age)<present_rear) {
        gap_entry_time_sec_=occupancy.front().time;blocking_id_=object.id;
        gap_reason_="current target rear gap";return false;
      }
      for(const auto& sample:occupancy) {
        if(std::abs(sample.lateral-q.lateral)>sample.half_width+width+.25)continue;
        const double elapsed=age+sample.time;
        const double gap=initial+other*elapsed-sample.progress;
        const double front=parameters_.vehicle_front_m+half+parameters_.front_gap_m;
        const double rear=rearClearance(half,other,sample.speed);
        // Unknown rear motion spans stationary through the configured speed bound.
        // Distance can prove this entire reachable interval remains behind ego.
        const bool unknown_rear=!object.velocity_available && initial<0;
        const double slow_gap=initial-sample.progress;
        const bool blocked=unknown_rear?(slow_gap<front && gap>-rear):
            (gap>=0?gap<front:-gap<rear);
        if(blocked) {
          gap_entry_time_sec_=sample.time;blocking_id_=object.id;
          gap_reason_=!object.velocity_available?"motion pending":gap>=0?"target front gap":"target rear gap";
          return false;
        }
      }
    }
    return true;
  }
  double mergeBehindSpeed(const nav_msgs::Odometry& ego,const interfaces::Objects& objects,double speed,double now) {
    const auto origin=coordinate(target_path_,ego.pose.pose.position.x,ego.pose.pose.position.y);
    const double horizon=std::max(1.0,transition_m_/std::max(5.0,std::min(speed,reference_change_speed_mps_)));
    for(const auto& object:objects.objects) {
      if(object.id!=blocking_id_)continue;
      const auto q=coordinate(target_path_,object.bbox_pose.position.x,object.bbox_pose.position.y);
      const double yaw=headingOf(object.bbox_pose)-q.yaw;
      const double half=(std::abs(std::cos(yaw))*object.bbox_size.x+std::abs(std::sin(yaw))*object.bbox_size.y)/2;
      // Wait for the detector's motion estimate without treating an unknown rear car as stopped.
      if(!object.velocity_available)return reference_change_speed_mps_;
      const double other=std::max(0.0,object.velocity.x*std::cos(q.yaw)+object.velocity.y*std::sin(q.yaw));
      const double relative=q.station-origin.station+other*std::max(0.0,now-objects.header.stamp.toSec());
      if(relative<0 && reference_change_speed_mps_>other+2) {
        const double rear_clearance=rearClearance(half,other,speed);
        const double forward_target=other+std::max(0.0,rear_clearance+relative)/std::max(1.0,gap_entry_time_sec_);
        // Stay in this lane while opening a rear gap; gapClear still decides commitment.
        return std::min(reference_change_speed_mps_,std::max(speed,forward_target));
      }
      // A rear car is not a front leader. Preserve motion in our own lane while
      // yielding; the reserved waiting endpoint still prevents a forced merge.
      if(relative<0)return reference_change_speed_mps_;
      const double clearance=parameters_.vehicle_front_m+half+parameters_.front_gap_m+other*parameters_.following_time_sec;
      // Shape longitudinal spacing while staying in our lane; commit still requires gapClear().
      return std::min(reference_change_speed_mps_,std::max(0.0,other+(relative-clearance)/horizon));
    }
    return reference_change_speed_mps_;
  }
  void follow(Message& path,const interfaces::Objects& objects,double speed,double now,
              double alignment_speed=std::numeric_limits<double>::infinity()) {
    for(auto& point:path.points){point.velocity=active_limit_mps_;point.acceleration=0;}
    const auto origin=coordinate(path,path.points.front().pose.pose.position.x,path.points.front().pose.pose.position.y);
    for(auto it=following_gap_targets_.begin();it!=following_gap_targets_.end();)
      if(now<it->second.time || now-it->second.time>.8)it=following_gap_targets_.erase(it);else ++it;
    double target=active_limit_mps_,deceleration=parameters_.acc_deceleration_mps2;bool immediate_stop=false,following=false;
    for(const auto& object:objects.objects) {
      const auto q=coordinate(path,object.bbox_pose.position.x,object.bbox_pose.position.y);
      const double angle=headingOf(object.bbox_pose)-q.yaw;
      const double width=(std::abs(std::sin(angle))*object.bbox_size.x+std::abs(std::cos(angle))*object.bbox_size.y)/2;
      if(std::abs(q.lateral)>parameters_.vehicle_half_width_m+width+.25 || q.station<=origin.station)continue;
      following=true;
      const double half=(std::abs(std::cos(angle))*object.bbox_size.x+std::abs(std::sin(angle))*object.bbox_size.y)/2;
      const double lead=object.velocity_available?std::max(0.0,object.velocity.x*std::cos(q.yaw)+object.velocity.y*std::sin(q.yaw)):0;
      const double gap=q.station-origin.station+lead*std::max(0.0,now-objects.header.stamp.toSec())-parameters_.vehicle_front_m-half;
      // A newly acquired close leader (including a rear merge) starts at its
      // measured gap. Open the desired spacing progressively; collision checks
      // below always use the real gap and relative speed, without this ramp.
      auto slot=following_gap_targets_.emplace(object.id,GapTarget{
          std::min(parameters_.following_gap_m,std::max(parameters_.stop_buffer_m,gap)),now}).first;
      auto& requested_gap=slot->second;
      requested_gap.distance=std::min(parameters_.following_gap_m,requested_gap.distance+
          parameters_.following_gap_recovery_rate_mps*std::max(0.,std::min(.2,now-requested_gap.time)));
      requested_gap.time=now;
      const double gap_error=gap-requested_gap.distance;
      double correction=gap_error/parameters_.following_time_sec;
      // Reserve response time and comfortable braking distance before closing
      // the remaining gap; do not accelerate until reaching a late brake threshold.
      const double response_speed=parameters_.acc_deceleration_mps2*parameters_.following_time_sec;
      if(gap_error>0) correction=std::min(correction,
          std::sqrt(response_speed*response_speed+2*parameters_.acc_deceleration_mps2*gap_error)-response_speed);
      const double desired=std::max(0.0,lead+correction);
      target=std::min(target,desired);
      const double closing=std::max(0.0,speed-lead);
      deceleration=std::max(deceleration,std::min(parameters_.stopping_deceleration_mps2,
          closing*closing/(2*std::max(1.0,gap_error-closing*parameters_.following_time_sec))));
      // The following corridor includes a comfort margin. A neighbour touching
      // only that margin is not an occupied-lane emergency stop.
      const bool occupied_lane=std::abs(q.lateral)<=parameters_.vehicle_half_width_m+width;
      if(occupied_lane && (gap<=parameters_.stop_buffer_m ||
          (object.velocity_available && gap<=parameters_.stop_buffer_m+
           closing*closing/(2*parameters_.stopping_deceleration_mps2))))immediate_stop=true;
    }
    if(alignment_speed<target) {
      target=alignment_speed;
    }
    const double dt=last_follow_sec_<0?0:std::max(0.0,std::min(.2,now-last_follow_sec_));
    if(previous_target_mps_<0)previous_target_mps_=std::min(speed,active_limit_mps_);
    if(last_follow_emergency_ && !immediate_stop)
      previous_target_mps_=std::max(previous_target_mps_,std::min(speed,target));
    last_follow_emergency_=immediate_stop;
    if(immediate_stop)target=0;
    else if(target<previous_target_mps_)target=std::max(target,std::min(previous_target_mps_,speed)-deceleration*dt);
    else target=std::min(target,(following?previous_target_mps_:
        std::max(previous_target_mps_,std::min(speed,target)))+parameters_.acc_acceleration_mps2*dt);
    target=std::min(target,active_limit_mps_);previous_target_mps_=target;last_follow_sec_=now;
    for(auto& point:path.points)point.velocity=target;
  }
  void publishStatus(const nav_msgs::Odometry& ego,const std::string& mode) {
    visualization_msgs::Marker marker;marker.header=ego.header;marker.ns="highway_status";marker.id=1;
    marker.type=visualization_msgs::Marker::TEXT_VIEW_FACING;marker.action=visualization_msgs::Marker::ADD;
    marker.pose=ego.pose.pose;marker.pose.position.z+=3;marker.scale.z=1;marker.color.r=marker.color.g=marker.color.a=1;
    marker.lifetime=ros::Duration(.5);marker.text=mode+" changes="+std::to_string(completed_changes_)+"\n"+gap_reason_+" id="+std::to_string(blocking_id_);
    status_visualizer_.publish(marker);
    ROS_INFO_STREAM_THROTTLE(1.0,"[highway] progress mode="<<mode<<" changes="<<completed_changes_
        <<" target="<<target_lane_id_<<" xy="<<ego.pose.pose.position.x<<","<<ego.pose.pose.position.y
        <<" speed="<<ego.twist.twist.linear.x<<" target_speed="<<previous_target_mps_
        <<" handover_remaining="<<(maneuver_.points.empty()?0:handover_m_-signedStation(maneuver_,ego.pose.pose.position.x,ego.pose.pose.position.y)));

    if(mode!=mode_){ROS_INFO_STREAM("[highway] "<<mode<<" "<<gap_reason_<<" object="<<blocking_id_);mode_=mode;}
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
      std_msgs::ColorRGBA color;color.a=1;
      if(!object.velocity_available){color.r=1;color.g=.85;color.b=.1;}
      else if(std::hypot(object.velocity.x,object.velocity.y)>.8){color.r=1;color.g=.15;color.b=.15;}
      else {color.r=.15;color.g=.55;color.b=1;}
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
  static double headingOf(const geometry_msgs::Pose& p) {
    return std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                      1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));
  }
  static std::vector<double> distances(const Message& path) {
    std::vector<double> s(path.points.size(), 0);
    for (std::size_t i=1;i<s.size();++i) {
      const auto& a=path.points[i-1].pose.pose.position; const auto& b=path.points[i].pose.pose.position;
      s[i]=s[i-1]+std::hypot(b.x-a.x,b.y-a.y);
    }
    return s;
  }
  void capManeuverEnd(Message& path) const {
    const auto station=distances(path);
    for(std::size_t i=0;i<path.points.size();++i)
      path.points[i].velocity=std::min(path.points[i].velocity,std::sqrt(2*parameters_.stopping_deceleration_mps2*
          std::max(0.0,station.back()-station[i]-parameters_.vehicle_front_m-parameters_.stop_buffer_m)));
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
    for(auto& p:result.points) p.velocity=parameters_.cruise_speed_mps;
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
    // Source-lane successors remain usable after adjacency to the target ends.
    // Extend them independently instead of treating a lanelet split as road end.
    double length=0;
    for(std::size_t i=1;i<line.size();++i)length+=(line[i]-line[i-1]).head<2>().norm();
    for(unsigned step=0;step<40 && length<planning_length_m_*2;++step) {
      const auto following=graph_->following(source,false);if(following.empty())break;
      const auto tail=referenceCenterline(source);
      const auto direction=(tail.back()-tail[tail.size()-2]).head<2>().normalized().eval();
      auto next=following.front();double best=-2;
      for(const auto& candidate:following) {
        const auto& points=referenceCenterline(candidate);
        const double alignment=direction.dot((points[1]-points[0]).head<2>().normalized());
        if(alignment>best){best=alignment;next=candidate;}
      }
      if(!connected(source,next))break;
      const auto& points=referenceCenterline(next);
      for(const auto& point:points) {
        const double distance=(point-line.back()).head<2>().norm();
        if(distance>1e-6){length+=distance;line.push_back(point);}
      }
      source=next;
    }
    return line;
  }
  double changeWindow(const lanelet::routing::LaneletPath& route,std::size_t index,
                      const lanelet::BasicPoint2d& origin) {
    auto source=route[index];
    auto line=referenceCenterline(source);
    const auto start=project(line,origin);
    double remaining=(line[start.segment+1]-start.point).head<2>().norm();
    for(std::size_t j=start.segment+2;j<line.size();++j)remaining+=(line[j]-line[j-1]).head<2>().norm();
    // Extend a change only over successors that still permit the same adjacent move.
    for(std::size_t i=index+2;i<route.size() && connected(route[i-1],route[i]);++i) {
      lanelet::Optional<lanelet::ConstLanelet> next;
      const auto left=graph_->left(route[i]),right=graph_->right(route[i]);
      if(left && connected(source,*left) && adjacent(*left,route[i]))next=*left;
      else if(right && connected(source,*right) && adjacent(*right,route[i]))next=*right;
      if(!next)break;
      line=referenceCenterline(*next);
      for(std::size_t j=1;j<line.size();++j)remaining+=(line[j]-line[j-1]).head<2>().norm();
      source=*next;
    }
    return remaining;
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
        const double available=std::min({source_length,target_length,changeWindow(route,i,begin.point.head<2>())});
        const double minimum_transition=std::sqrt(6*offset/parameters_.maximum_curvature_per_m);
        const double duration=std::sqrt(6*offset/parameters_.maximum_lateral_acceleration_mps2);
        const double usable=std::max(0.0,available-parameters_.stop_buffer_m);
        // Slow down to retain a feasible merge instead of discarding it at the cruise design speed.
        reference_change_speed_mps_=std::min(active_limit_mps_,usable/std::max(.1,duration));
        const double transition=std::max({minimum_transition,
            std::min(parameters_.transition_length_m,usable),reference_change_speed_mps_*duration});
        // Preserve the design merge length while waiting, rather than consuming
        // it until only a crawling merge remains possible.
        const double reserved_transition=std::max({minimum_transition,
            std::min(parameters_.transition_length_m,usable),active_limit_mps_*duration});
        waiting_stop_station_m_=length_m+std::max(0.0,available-reserved_transition);
        if(source.points.size()<2 || target.points.size()<2 ||
            offset>parameters_.maximum_lane_offset_m || transition>available) {
          // Retain the requested merge rather than proceeding past its remaining legal window.
          change_window_exhausted_=true;gap_reason_="insufficient merge length";
          appendLine(message,source_line,source_projection.segment+1,length_m);
          if(message.points.size()>1)headings(message);
          return message;
        }
        const auto source_s=distances(source),target_s=distances(target);
        target_lane_id_=route[i+1].id();target_path_=target;transition_m_=transition;change_begin_m_=length_m;
        const double half_width=std::sqrt(std::min(
            project(route[i+1].leftBound3d().basicLineString(),target_projection.point.head<2>()).squared_distance,
            project(route[i+1].rightBound3d().basicLineString(),target_projection.point.head<2>()).squared_distance));
        reference_completion_lateral_m_=std::max(parameters_.completion_lateral_m,std::min(.5,
            half_width-parameters_.vehicle_half_width_m-parameters_.vehicle_front_m*std::sin(parameters_.completion_heading_rad)));

        waiting_path_=message;double waiting_length=length_m;
        for(double d=parameters_.sample_step_m;d<=source_length && waiting_length<planning_length_m_;d+=parameters_.sample_step_m) {
          const auto q=sample(source,source_s,d);appendLine(waiting_path_,{{q.x,q.y,q.z}},0,waiting_length);
        }
        headings(waiting_path_);
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
        for(double d=transition+parameters_.sample_step_m;d<=target_s.back() && length_m<planning_length_m_;d+=parameters_.sample_step_m) {
          const auto q=sample(target,target_s,d);appendLine(message,{{q.x,q.y,q.z}},0,length_m);
        }
        headings(message);return message;
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

  // Cache a world-fixed reference before projecting the moving ego onto it.
  // Gaussian arclength convolution removes short lateral teeth without fitting
  // long curves across a junction or changing the map boundaries.
  lanelet::BasicLineString3d smoothReference(const lanelet::ConstLanelet& lane,
                                            lanelet::BasicLineString3d source) const {
    if(!parameters_.reference_smoothing_enabled)return source;
    constexpr double sigma=1.0, step=.25, maximum_shift=.20;
    auto before=graph_->previous(lane,false),after=graph_->following(lane,false);
    lanelet::BasicLineString3d prefix,suffix;
    double first_gap=.4,last_gap=.4;
    for(const auto& other:before) {
      const auto line=other.centerline3d().basicLineString();
      const double gap=(line.back()-source.front()).head<2>().norm();
      if(gap<first_gap){first_gap=gap;prefix=line;}
    }
    for(const auto& other:after) {
      const auto line=other.centerline3d().basicLineString();
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
    auto key=std::make_pair(lane.id(),lane.inverted());auto found=centerlines_.find(key);
    if(found==centerlines_.end())found=centerlines_.emplace(key,smoothReference(lane,lane.centerline3d().basicLineString())).first;
    return found->second;
  }
  double previous_target_mps_=-1,last_follow_sec_=-1;
  bool last_follow_emergency_=false;
  std::map<std::uint32_t,GapTarget> following_gap_targets_;
  double gap_entry_time_sec_=0;
  double last_observation_sec_=-1;
  std::map<std::uint32_t,std::deque<MotionSample>> motion_history_;
  HighwayMissionParameters parameters_;double planning_length_m_,planning_speed_mps_=0,maneuver_speed_mps_=0,active_limit_mps_=0;
  lanelet::traffic_rules::TrafficRulesPtr rules_;lanelet::routing::RoutingGraphUPtr graph_;
  std::map<std::pair<lanelet::Id,bool>,lanelet::BasicLineString3d> centerlines_;
  std::vector<LaneSpan> reference_spans_;Message maneuver_,waiting_path_,target_path_;
  bool change_window_exhausted_=false,reference_lane_change_=false;double reference_handover_m_=0,handover_m_=0,transition_m_=0,change_begin_m_=0,gap_since_=-1;
  double reference_change_speed_mps_=0,waiting_stop_station_m_=0;
  double reference_completion_lateral_m_=.2,maneuver_completion_lateral_m_=.2;
  std::size_t reference_end_index_=0;lanelet::Id target_lane_id_=0;unsigned completed_changes_=0;
  std::uint32_t blocking_id_=0;std::string gap_reason_="",mode_;
  node_pipeline::Visualizer<visualization_msgs::Marker> motion_visualizer_{"object_motion"};
  node_pipeline::Visualizer<visualization_msgs::Marker> status_visualizer_{"highway_status"};
};
} // namespace path_planner
namespace path_planner {
struct IntegratedHighwayParameters {
  HighwayMissionParameters mission;
  double publication_horizon_sec=5.,minimum_publication_length_m=47.;
};
class IntegratedHighway {
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
  IntegratedHighway(IntegratedHighwayParameters p,SharedMap& map):
      p_(std::move(p)),mission_(p_.mission,*map.map) {}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& competition,
      const interfaces::Objects& objects,const Route& route) {
    return publishLegacyPrefix(mission_.process(ego,competition,objects,route),
        std::max(0.,double(competition.velocity_x_mps)),p_.minimum_publication_length_m,p_.publication_horizon_sec);
  }
  const char* planningOutcome() const {return "existing-highway";}
  std::string planningDiagnostics() const {return " geometry and speed profile";}
 private:
  IntegratedHighwayParameters p_;
  HighwayMission mission_;
};
}
