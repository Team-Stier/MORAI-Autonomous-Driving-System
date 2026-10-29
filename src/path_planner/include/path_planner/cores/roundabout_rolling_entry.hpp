#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <geometry_msgs/Point.h>
#include <path_planner/libraries/types.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <Eigen/Geometry>
#include <ros/console.h>
#include <array>
#include <deque>
#include <map>
#include <tuple>
#include <interfaces/Path.h>
#include <nav_msgs/Odometry.h>
#include <limits>
#include <utility>

// Roundabout gap acceptance without a mandatory stop: while approaching, the
// curve-limited entry is checked from the current speed and accepted as soon as
// it clears the predicted traffic; otherwise the yield-line stop and the
// at-rest launch check apply.
namespace path_planner {
struct RoundaboutRollingEntryParameters {
  std::vector<lanelet::Id> reference_lanelet_ids;
  lanelet::Id stop_line_id=0,departure_lanelet_id=0;
  std::vector<geometry_msgs::Point> stop_line;
  double reference_before_entry_m=65,reference_after_entry_m=100;
  double reference_start_m=0,reference_end_m=0;
  double control_step_m = 2.0;
  double integration_step_m = 0.05;
  double sample_step_m = 0.25;
  double local_path_length_m = 40.0;
  double desired_speed_mps = 5.0;
  double approach_speed_mps = 3.0;
  double curve_speed_ratio = 0.75;
  double entry_stop_margin_m = 0.8;
  double entry_decision_distance_m = 2.0;
  double entry_check_length_m = 8.0;
  double entry_gap_confirmation_sec = 0.1;
  double planning_horizon_sec = 3.0;
  double max_acceleration_mps2 = 2.0;
  double max_deceleration_mps2 = 4.0;
  double max_jerk_mps3 = 4.0;
  double max_lateral_acceleration_mps2 = 2.0;
  double time_step_sec = 0.25;
  double command_step_sec = 0.8;
  int beam_width = 64;
  double collision_margin_m = 0.5;
  double prediction_uncertainty_mps = 0.2;
  double vehicle_front_m = 3.845008;
  double vehicle_rear_m = 0.790000;
  double vehicle_half_width_m = 1.080418;
  double following_time_gap_sec = 1.2;
  double rolling_response_sec = 0.5;
  double rolling_accept_distance_m = 6.0;
  // Entry is committed once the front passes the stop line by this distance.
  double commit_past_line_m = 1.0;
  // Slowest and fastest executed launch from rest (response delay, acceleration, jerk,
  // command step); an at-rest entry clears both, a rolling entry also clears the fast one.
  double slow_launch_response_sec = 0.55;
  double slow_launch_acceleration_mps2 = 3.2;
  double slow_launch_jerk_mps3 = 12.0;
  double fast_launch_response_sec = 0.15;
  double fast_launch_acceleration_mps2 = 4.0;
  double fast_launch_jerk_mps3 = 12.0;
  double fast_launch_command_step_sec = 0.4;
  // Same-direction traffic behind the ego keeps this time gap at its own speed.
  double rear_time_gap_sec = 0.6;
  // After waiting this long at the yield target (0 never), the at-rest check only clears the
  // planned launch after relaxed_launch_response_sec, without the fast bound and the rear gap.
  double patience_sec = 8.0;
  double relaxed_launch_response_sec = 0.8;
  // Ring lanelets upstream of the merge in driving order, and the merged ring lanelet.
  std::vector<lanelet::Id> conflict_lanelet_ids;
  lanelet::Id conflict_downstream_lanelet_id=0;
  double conflict_downstream_m = 6.0;
  double conflict_zone_half_width_m = 2.5;
  double conflict_margin_m = 3.0;
  double conflict_speed_floor_mps = 4.0;
  double conflict_clear_sec = 4.5;
  double stationary_confirmation_sec = 3.0;
  double stationary_displacement_m = 1.0;
};

class RoundaboutRollingEntryMission {
  struct Tube;
 public:
  RoundaboutRollingEntryMission(RoundaboutRollingEntryParameters parameters,lanelet::LaneletMap& map) : p_(std::move(parameters)) {
    for(auto id:p_.reference_lanelet_ids) {
      for(const auto& q:map.laneletLayer.get(id).centerline3d()) {
        geometry_msgs::Point point;point.x=q.x();point.y=q.y();point.z=q.z();
        if(raw_.empty()) {raw_.push_back(point);raw_station_.push_back(0);}
        else if(distance(raw_.back(),point)>1e-6) {
          raw_station_.push_back(raw_station_.back()+distance(raw_.back(),point));raw_.push_back(point);
        }
      }
    }
    geometry_msgs::Point entry;entry.x=(p_.stop_line.front().x+p_.stop_line.back().x)/2;
    entry.y=(p_.stop_line.front().y+p_.stop_line.back().y)/2;
    const double entry_raw=rawStation(entry);
    p_.reference_start_m=std::max(0.,entry_raw-p_.reference_before_entry_m);
    p_.reference_end_m=std::min(raw_station_.back(),entry_raw+p_.reference_after_entry_m);
    const double span_m = p_.reference_end_m - p_.reference_start_m;
    const std::size_t cells = static_cast<std::size_t>(std::ceil(span_m / p_.control_step_m));
    control_step_m_ = span_m / cells;
    for (std::size_t i = 0; i <= cells; ++i)
      controls_.push_back(rawPoint(p_.reference_start_m + control_step_m_ * i));
    const std::size_t samples = static_cast<std::size_t>(std::ceil(span_m / p_.integration_step_m));
    for (std::size_t i = 0; i <= samples; ++i) {
      Sample sample;
      sample.u = static_cast<double>(cells) * i / samples;
      sample.point = evaluate(sample.u).pose.pose.position;
      if (!samples_.empty()) sample.station_m = samples_.back().station_m + distance(samples_.back().point, sample.point);
      samples_.push_back(sample);
    }
    entry_line_m_ = projectStation(entry);
    yield_station_m_ = entry_line_m_ - p_.vehicle_front_m - p_.entry_stop_margin_m;
    const auto departure=map.laneletLayer.get(p_.departure_lanelet_id).centerline3d().front();
    geometry_msgs::Point point;point.x=departure.x();point.y=departure.y();point.z=departure.z();
    departure_station_m_=projectStation(point);
    for(auto id:p_.conflict_lanelet_ids)
      for(const auto& q:map.laneletLayer.get(id).centerline3d()) {geometry_msgs::Point zone_point;zone_point.x=q.x();zone_point.y=q.y();appendZone(zone_point);}
    conflict_station_m_=zone_station_.back();
    for(const auto& q:map.laneletLayer.get(p_.conflict_downstream_lanelet_id).centerline3d()) {
      if(zone_station_.back()>=conflict_station_m_+p_.conflict_downstream_m)break;
      {geometry_msgs::Point zone_point;zone_point.x=q.x();zone_point.y=q.y();appendZone(zone_point);}
    }
  }

  Message process(const nav_msgs::Odometry& ego,
                  const interfaces::CompetitionVehicleStatus& competition,
                  const interfaces::Objects& objects,const PredictedObjects& predictions,
                  const ros::Time& stamp) {
    auto path=referencePath(ego);
    if(path.points.size()<2){for(auto& p:path.points){p.velocity=0;p.acceleration=0;}return path;}
    const double current=projectStation(path.points.front().pose.pose.position);
    const double speed=std::max(0.,double(competition.velocity_x_mps));
    if(current<previous_station_m_-5){gate_open_=entered_=stopped_at_line_=relaxed_=false;gap_since_sec_=-1;}
    previous_station_m_=current;
    entered_=entered_ || current>=entry_line_m_-p_.vehicle_front_m+p_.commit_past_line_m;
    const double remaining=yield_station_m_-current;
    std::vector<double> station(path.points.size(),0.);
    path_yaw_.resize(path.points.size());path_turn_.resize(path.points.size()-1);path_curvature_.resize(path.points.size()-1);
    path_speed_.assign(path.points.size(),p_.desired_speed_mps);
    for(std::size_t i=0;i<path.points.size();++i){path_yaw_[i]=yaw(path.points[i].pose.pose.orientation);if(i)station[i]=station[i-1]+distance(path.points[i-1].pose.pose.position,path.points[i].pose.pose.position);}
    for(std::size_t i=0;i+1<path.points.size();++i){path_turn_[i]=angle(path_yaw_[i+1]-path_yaw_[i]);path_curvature_[i]=std::abs(path_turn_[i])/std::max(1e-6,station[i+1]-station[i]);const double cap=p_.curve_speed_ratio*std::sqrt(p_.max_lateral_acceleration_mps2/std::max(1e-6,path_curvature_[i]));path_speed_[i]=std::min(path_speed_[i],cap);path_speed_[i+1]=std::min(path_speed_[i+1],cap);}
    for(std::size_t i=station.size()-1;i>0;--i)path_speed_[i-1]=std::min(path_speed_[i-1],std::sqrt(path_speed_[i]*path_speed_[i]+2*p_.max_deceleration_mps2*(station[i]-station[i-1])));
    if(gate_open_ || entered_) {
      setPhase(current>=departure_station_m_?Phase::Depart:entered_?Phase::Follow:Phase::Enter,remaining);
      followLead(path,station,objects,predictions,speed);
      return path;
    }
    tracking_padding_=std::hypot(ego.pose.pose.position.x-path.points.front().pose.pose.position.x,ego.pose.pose.position.y-path.points.front().pose.pose.position.y);
    auto obstacles=tubes(predictions);
    for(const auto& object:objects.objects){auto found=std::find_if(obstacles.begin(),obstacles.end(),[&](const Tube& t){return t.id==object.id;});if(found!=obstacles.end()){found->boxes[0]=box(object);prepareTube(*found);}}
    const double age=std::max(0.,stamp.toSec()-objects.header.stamp.toSec());
    bool motion_ready=true;
    for(std::size_t i=0;i<objects.objects.size();++i) {
      const auto& object=objects.objects[i];
      if(object.bbox_size.x<2.5 || distance(object.position,ego.pose.pose.position)>35)continue;
      const auto predicted=std::find_if(predictions[0].objects.begin(),predictions[0].objects.end(),
          [&](const interfaces::Object& q){return q.id==object.id;});
      if(predicted!=predictions[0].objects.end() && !predicted->velocity_available)motion_ready=false;
    }
    // Ring traffic that reaches the merge within the clearing time blocks entry,
    // independent of the predicted motion.
    if(conflictZoneBlocked(objects))motion_ready=false;
    if(remaining<=.35 && speed<.15 && !stopped_at_line_){stopped_at_line_=true;yield_since_sec_=objects.header.stamp.toSec();}
    if(!stopped_at_line_) {
      setPhase(Phase::Approach,remaining);
      planning_speed_mps_=p_.desired_speed_mps;planning_end_m_=station.back();
      // The moving entry is decided only near the yield target, where the conflicting
      // ring sector is observed, and must clear an on-time, a delayed and a fast start.
      const Launch planned{p_.max_acceleration_mps2,p_.max_jerk_mps3,p_.command_step_sec,p_.rear_time_gap_sec};
      const auto on_time=movingSchedule(path,station,obstacles,age,speed,0,current,planned);
      const auto delayed=on_time.empty()?on_time:movingSchedule(path,station,obstacles,age+p_.rolling_response_sec,speed,0,current,planned);
      const auto fast=delayed.empty()?delayed:movingSchedule(path,station,obstacles,age,speed,0,current,fastLaunch());
      const auto reaches=[&](const std::vector<State>& launch){return !launch.empty() &&
          launch.back().s>=std::max(0.,remaining)+p_.entry_check_length_m && launch.back().v>=p_.approach_speed_mps;};
      if(motion_ready && remaining<=p_.rolling_accept_distance_m && reaches(on_time) && reaches(delayed) && reaches(fast)) {
        if(gap_since_sec_<0)gap_since_sec_=objects.header.stamp.toSec();
        if(objects.header.stamp.toSec()-gap_since_sec_>=p_.entry_gap_confirmation_sec || objects.objects.empty())gate_open_=true;
      } else gap_since_sec_=-1;
      if(!gate_open_)return stopProfile(std::move(path),station,remaining);
      ROS_INFO("roundabout_rolling_entry: rolling entry accepted at %.2f m/s, %.2f m before yield target",speed,remaining);
      setPhase(Phase::Enter,remaining);
      for(std::size_t i=0;i<path.points.size();++i){path.points[i].velocity=path_speed_[i];path.points[i].acceleration=0;}
      return path;
    }
    setPhase(Phase::Yield,remaining);
    planning_speed_mps_=p_.desired_speed_mps;planning_end_m_=station.back();
    // Include pedal/vehicle response before the fresh launch. Check the complete
    // merge at rest for the slow and the fast executed launch, then keep the
    // accepted entry instead of alternating phases. Dense ring traffic may leave no
    // such gap; after patience_sec only the relaxed planned launch is checked.
    const auto reaches=[&](const std::vector<State>& schedule){return !schedule.empty() &&
        schedule.back().s>=std::max(0.,remaining)+p_.entry_check_length_m && schedule.back().v>=p_.approach_speed_mps;};
    const double waited=objects.header.stamp.toSec()-yield_since_sec_;
    const bool relaxed=p_.patience_sec>0 && waited>=p_.patience_sec;
    if(relaxed && !relaxed_)ROS_INFO("roundabout_rolling_entry: no gap for %.1f s, relaxed at-rest check",waited);
    relaxed_=relaxed;
    bool clear_entry=false;
    if(relaxed) {
      const Launch planned{p_.max_acceleration_mps2,p_.max_jerk_mps3,p_.command_step_sec,0};
      clear_entry=motion_ready && reaches(movingSchedule(path,station,obstacles,age+p_.relaxed_launch_response_sec,0,0,current,planned));
    } else {
      const Launch slow{p_.slow_launch_acceleration_mps2,p_.slow_launch_jerk_mps3,p_.command_step_sec,p_.rear_time_gap_sec};
      const auto launch=movingSchedule(path,station,obstacles,age+p_.slow_launch_response_sec,0,0,current,slow);
      const auto fast=launch.empty()?launch:movingSchedule(path,station,obstacles,age+p_.fast_launch_response_sec,0,0,current,fastLaunch());
      clear_entry=motion_ready && reaches(launch) && reaches(fast);
    }
    if(clear_entry){if(gap_since_sec_<0)gap_since_sec_=objects.header.stamp.toSec();if(objects.header.stamp.toSec()-gap_since_sec_>=p_.entry_gap_confirmation_sec || objects.objects.empty())gate_open_=true;}
    else gap_since_sec_=-1;
    if(!gate_open_)return stopProfile(std::move(path),station,std::max(0.,remaining));
    setPhase(Phase::Enter,remaining);for(std::size_t i=0;i<path.points.size();++i){path.points[i].velocity=path_speed_[i];path.points[i].acceleration=0;}return path;
  }

 private:
  enum class Phase { Approach, Yield, Enter, Follow, Depart };
  // A checked schedule: acceleration and jerk limits, command step, and the rear time gap it keeps.
  struct Launch { double acceleration_mps2, jerk_mps3, command_step_sec, rear_time_gap_sec; };
  Launch fastLaunch() const {
    return {p_.fast_launch_acceleration_mps2,p_.fast_launch_jerk_mps3,p_.fast_launch_command_step_sec,p_.rear_time_gap_sec};
  }

  void appendZone(const geometry_msgs::Point& point) {
    if(zone_.empty()) {zone_.push_back(point);zone_station_.push_back(0);}
    else if(distance(point,zone_.back())>1e-6) {
      zone_station_.push_back(zone_station_.back()+distance(point,zone_.back()));zone_.push_back(point);
    }
  }
  // Zone station and lateral distance of a map position on the ring conflict polyline.
  std::pair<double,double> zoneProjection(const geometry_msgs::Point& position) const {
    double closest=std::numeric_limits<double>::infinity(),station=0;
    for(std::size_t i=1;i<zone_.size();++i) {
      const auto& a=zone_[i-1];const auto& b=zone_[i];
      const double dx=b.x-a.x,dy=b.y-a.y;
      const double u=clamp(((position.x-a.x)*dx+(position.y-a.y)*dy)/(dx*dx+dy*dy),0,1);
      const double squared=std::pow(a.x+u*dx-position.x,2)+std::pow(a.y+u*dy-position.y,2);
      if(squared<closest){closest=squared;station=zone_station_[i-1]+u*(zone_station_[i]-zone_station_[i-1]);}
    }
    return {station,std::sqrt(closest)};
  }
  // Speed component along the zone direction at a zone station.
  double zoneSpeed(double station,const geometry_msgs::Vector3& velocity) const {
    const auto after=std::upper_bound(zone_station_.begin(),zone_station_.end(),station);
    const std::size_t i=std::min(zone_.size()-1,std::max<std::size_t>(1,after-zone_station_.begin()));
    const double dx=zone_[i].x-zone_[i-1].x,dy=zone_[i].y-zone_[i-1].y;
    return (velocity.x*dx+velocity.y*dy)/std::hypot(dx,dy);
  }
  bool conflictZoneBlocked(const interfaces::Objects& objects) {
    const double now=objects.header.stamp.toSec();
    for(auto it=sightings_.begin();it!=sightings_.end();)
      if(it->second.empty() || now-it->second.back().first>1.0 || now<it->second.back().first)it=sightings_.erase(it); else ++it;
    std::uint32_t blocker=0;double blocker_distance=0,blocker_speed=0;bool blocked=false;
    for(const auto& object:objects.objects) {
      const auto& position=object.bbox_pose.position;
      auto& history=sightings_[object.id];
      if(history.empty() || now>history.back().first)history.emplace_back(now,position);
      while(history.size()>1 && now-history.front().first>p_.stationary_confirmation_sec+1.0)history.pop_front();
      const auto projection=zoneProjection(position);
      const double to_conflict=conflict_station_m_-projection.first;
      if(projection.second>p_.conflict_zone_half_width_m || to_conflict<-p_.conflict_downstream_m)continue;
      double displacement=0;
      for(const auto& sighting:history)if(now-sighting.first<=p_.stationary_confirmation_sec)
        displacement=std::max(displacement,distance(sighting.second,position));
      if(now-history.front().first>=p_.stationary_confirmation_sec && displacement<p_.stationary_displacement_m)continue;
      double speed=p_.conflict_speed_floor_mps;
      for(const auto& sighting:history)if(now-sighting.first>=.5 && now-sighting.first<=1.5) {
        speed=std::max(speed,(projection.first-zoneProjection(sighting.second).first)/(now-sighting.first));break;
      }
      if(object.velocity_available)speed=std::max(speed,zoneSpeed(projection.first,object.velocity));
      if(!blocked && std::max(0.,to_conflict-p_.conflict_margin_m)/speed<p_.conflict_clear_sec) {
        blocked=true;blocker=object.id;blocker_distance=to_conflict;blocker_speed=speed;
      }
    }
    if(blocked && blocker!=zone_blocker_)
      ROS_INFO("roundabout_rolling_entry: ring object %u blocks entry, %.1f m before merge at %.1f m/s",blocker,blocker_distance,blocker_speed);
    zone_blocker_=blocked?blocker:0;
    return blocked;
  }
  void setPhase(Phase value,double remaining) {
    if(phase_==value)return;
    phase_=value;static const char* names[]={"approach","yield","enter","follow","depart"};
    ROS_INFO("roundabout_rolling_entry: %s, rear axle to yield target=%.2f m",names[static_cast<int>(value)],remaining);
  }
  // Keep the accepted entry. Only a vehicle ahead on this same reference
  // regulates cruise; rear and crossing traffic do not reopen the yield gate.
  void followLead(Message& path, const std::vector<double>& station,
                  const interfaces::Objects& objects, const PredictedObjects& predictions,
                  double ego_speed) const {
    double cap=p_.desired_speed_mps;
    for(const auto& object:objects.objects) {
      const auto observed=box(object);
      double squared=std::numeric_limits<double>::infinity(),along=0,lateral=0,heading=0;
      for(std::size_t i=1;i<path.points.size();++i) {
        const auto& a=path.points[i-1].pose.pose.position;
        const auto& b=path.points[i].pose.pose.position;
        const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
        const double u=clamp(((observed.pose.x-a.x)*dx+(observed.pose.y-a.y)*dy)/(length*length),0,1);
        const double ex=observed.pose.x-a.x-u*dx,ey=observed.pose.y-a.y-u*dy;
        if(ex*ex+ey*ey<squared) {
          squared=ex*ex+ey*ey;along=station[i-1]+u*length;
          lateral=(-ex*dy+ey*dx)/length;heading=std::atan2(dy,dx);
        }
      }
      const double turn=angle(observed.pose.yaw-heading);
      const double half_length=std::abs(std::cos(turn))*observed.half_length+
                               std::abs(std::sin(turn))*observed.half_width;
      const double half_width=std::abs(std::sin(turn))*observed.half_length+
                              std::abs(std::cos(turn))*observed.half_width;
      const auto& start=path.points.front().pose.pose.position;
      const double forward=(observed.pose.x-start.x)*std::cos(path_yaw_.front())+
                           (observed.pose.y-start.y)*std::sin(path_yaw_.front());
      if(forward<=0 || along<=0 || std::cos(turn)<.5 ||
         std::abs(lateral)>p_.vehicle_half_width_m+half_width)continue;
      double lead_speed=0;bool measured=object.velocity_available;
      if(object.velocity_available)lead_speed=object.velocity.x*std::cos(heading)+object.velocity.y*std::sin(heading);
      else {
        const auto future=std::find_if(predictions.front().objects.begin(),predictions.front().objects.end(),
            [&](const interfaces::Object& q){return q.id==object.id;});
        if(future!=predictions.front().objects.end() && future->velocity_available) {
          measured=true;lead_speed=future->velocity.x*std::cos(heading)+future->velocity.y*std::sin(heading);
        }
      }
      lead_speed=std::max(0.,lead_speed);
      const double gap=along-half_length-p_.vehicle_front_m-p_.collision_margin_m;
      const double following=std::max(0.,lead_speed+(gap-p_.following_time_gap_sec*ego_speed)/1.5);
      const double braking=std::sqrt(lead_speed*lead_speed+2*p_.max_deceleration_mps2*std::max(0.,gap));
      // Unmeasured velocity is not a stationary-car measurement. Approach
      // its occupied body with a braking profile until motion becomes known.
      cap=std::min(cap,measured?std::min(following,braking):braking);
    }
    for(std::size_t i=0;i<path.points.size();++i) {
      path.points[i].velocity=std::min(path_speed_[i],cap);path.points[i].acceleration=0;
    }
  }
  Message stopProfile(Message path,const std::vector<double>& station,double target) const {
    const double response=.35*p_.max_deceleration_mps2;
    for(std::size_t i=0;i<path.points.size();++i){const double d=target-station[i];const double value=d<=.2?0:std::sqrt(response*response+2*p_.max_deceleration_mps2*d)-response;path.points[i].velocity=std::min(path_speed_[i],value);path.points[i].acceleration=path.points[i].velocity>0?-p_.max_deceleration_mps2:0;}
    return path;
  }
  double projectStation(const geometry_msgs::Point& position) const {
    double closest = std::numeric_limits<double>::infinity(), station = 0.0;
    for (std::size_t i = 0; i + 1 < samples_.size(); ++i) {
      const auto& a = samples_[i];
      const auto& b = samples_[i + 1];
      const double dx = b.point.x - a.point.x, dy = b.point.y - a.point.y;
      const double u = clamp(((position.x-a.point.x)*dx + (position.y-a.point.y)*dy)/(dx*dx+dy*dy), 0, 1);
      const double squared = std::pow(a.point.x+u*dx-position.x, 2) +
                             std::pow(a.point.y+u*dy-position.y, 2);
      if (squared < closest) {
        closest = squared;
        station = a.station_m + u*(b.station_m-a.station_m);
      }
    }
    return station;
  }

  interfaces::Path referencePath(const nav_msgs::Odometry& ego) const {
    const auto& position = ego.pose.pose.position;
    double closest_squared_m = std::numeric_limits<double>::infinity();
    double start_m = 0.0;
    for (std::size_t i = 0; i + 1 < samples_.size(); ++i) {
      const auto& a = samples_[i];
      const auto& b = samples_[i + 1];
      const double dx = b.point.x - a.point.x, dy = b.point.y - a.point.y;
      const double ratio = std::max(0.0, std::min(1.0,
          ((position.x - a.point.x) * dx + (position.y - a.point.y) * dy) / (dx * dx + dy * dy)));
      const double error_x = a.point.x + ratio * dx - position.x;
      const double error_y = a.point.y + ratio * dy - position.y;
      const double squared_m = error_x * error_x + error_y * error_y;
      if (squared_m < closest_squared_m) {
        closest_squared_m = squared_m;
        start_m = a.station_m + ratio * (b.station_m - a.station_m);
      }
    }
    const double end_m = std::min(start_m + p_.local_path_length_m, samples_.back().station_m);
    interfaces::Path path;
    const std::size_t count = static_cast<std::size_t>(std::floor((end_m - start_m) / p_.sample_step_m));
    path.points.reserve(count + 2);
    for (std::size_t i = 0; i <= count; ++i)
      path.points.push_back(evaluate(parameterAt(std::min(start_m + i * p_.sample_step_m, end_m))));
    if (end_m - start_m - count * p_.sample_step_m > 1e-8)
      path.points.push_back(evaluate(parameterAt(end_m)));
    else
      path.points.back() = evaluate(parameterAt(end_m));
    return path;
  }

 private:
  struct Pose {
    double x, y, yaw;
  };
  struct EgoPose {
    double time;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d velocity, angular_velocity;
  };
  struct Box {
    Pose pose;
    double half_length, half_width;
  };
  struct Tube {
    std::uint32_t id=0;
    bool following_rear=false;
    std::array<Box, 11> boxes;
    std::array<double, 10> turn;
    double min_x, max_x, min_y, max_y;
    double start = 0.0, end = 5.0;
    double max_speed = 0.0;
  };
  struct State {
    double t, s, v, a, cost;
    int parent;
  };
  struct Edge {
    State start;
    double h, v1, a1;
    double c2() const { return 3 * (v1 - start.v) / (h*h) - (2*start.a + a1) / h; }
    double c3() const { return 2 * (start.v - v1) / (h*h*h) + (start.a + a1) / (h*h); }
    double velocity(double t) const { return start.v + t * (start.a + t * (c2() + t*c3())); }
    double acceleration(double t) const { return start.a + t * (2*c2() + 3*t*c3()); }
    double jerk(double t) const { return 2*c2() + 6*t*c3(); }
    double station(double t) const {
      return start.s + t * (start.v + t * (start.a/2 + t * (c2()/3 + t*c3()/4)));
    }
  };

  static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
  static double angle(double x) { return std::atan2(std::sin(x), std::cos(x)); }
  static double yaw(const geometry_msgs::Quaternion& q) {
    return std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  }
  void rememberPose(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& status) {
    const double time = ego.header.stamp.toSec();
    if (!ego_history_.empty() && time < ego_history_.back().time) ego_history_.clear();
    if (ego_history_.empty() || time >= ego_history_.back().time) {
      if (!ego_history_.empty() && time == ego_history_.back().time) ego_history_.pop_back();
      const auto& p = ego.pose.pose.position;
      const auto& q = ego.pose.pose.orientation;
      ego_history_.push_back({time, {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z},
          {status.velocity_x_mps, ego.twist.twist.linear.y, ego.twist.twist.linear.z},
          {status.angular_velocity_x_radps, status.angular_velocity_y_radps, status.angular_velocity_z_radps}});
      while (ego_history_.size() > 2 && time - ego_history_[1].time > 1.0) ego_history_.pop_front();
    }
  }
  Eigen::Isometry3d observationFrame(double time) const {
    const auto after = std::lower_bound(ego_history_.begin(), ego_history_.end(), time,
        [](const EgoPose& pose, double t) { return pose.time < t; });
    Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
    if (after != ego_history_.begin() && after != ego_history_.end() &&
        after->time - (after - 1)->time <= 0.2) {
      const auto& before = *(after - 1);
      const double u = (time - before.time) / (after->time - before.time);
      frame.linear() = before.rotation.slerp(u, after->rotation).toRotationMatrix();
      frame.translation() = (1 - u)*before.position + u*after->position;
    } else {
      const auto& pose = after == ego_history_.end() ? ego_history_.back() : *after;
      const double dt = time - pose.time;
      const double omega = pose.angular_velocity.norm();
      const double turn = omega*dt;
      const double a = std::abs(turn) < 1e-5 ? dt*dt*(0.5 - turn*turn/24) :
          (1 - std::cos(turn))/(omega*omega);
      const double b = std::abs(turn) < 1e-5 ? dt*dt*dt*(1.0/6 - turn*turn/120) :
          (turn - std::sin(turn))/(omega*omega*omega);
      const auto cross = pose.angular_velocity.cross(pose.velocity);
      frame.translation() = pose.position + pose.rotation *
          (dt*pose.velocity + a*cross + b*pose.angular_velocity.cross(cross));
      const Eigen::Quaterniond delta = omega == 0 ? Eigen::Quaterniond::Identity() :
          Eigen::Quaterniond(Eigen::AngleAxisd(turn, pose.angular_velocity / omega));
      // shortcut: constant body twist outside pose history; use timestamp-matched localization for tighter gaps.
      frame.linear() = (pose.rotation * delta).toRotationMatrix();
    }
    return frame;
  }
  static Box box(const interfaces::Object& object) {
    const auto& p = object.bbox_pose.position;
    const auto& q = object.bbox_pose.orientation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const double heading = std::atan2(2*(rotation.w()*rotation.z() + rotation.x()*rotation.y()),
                                      1 - 2*(rotation.y()*rotation.y() + rotation.z()*rotation.z()));
    const Eigen::Vector3d forward(std::cos(heading), std::sin(heading), 0);
    const Eigen::Vector3d lateral(-std::sin(heading), std::cos(heading), 0);
    const auto axes = rotation.toRotationMatrix();
    const auto& size = object.bbox_size;
    return {{p.x, p.y, heading},
        (std::abs(forward.dot(axes.col(0)))*size.x + std::abs(forward.dot(axes.col(1)))*size.y +
         std::abs(forward.dot(axes.col(2)))*size.z) / 2,
        (std::abs(lateral.dot(axes.col(0)))*size.x + std::abs(lateral.dot(axes.col(1)))*size.y +
         std::abs(lateral.dot(axes.col(2)))*size.z) / 2};
  }
  static std::vector<Tube> tubes(const PredictedObjects& predictions) {
    std::vector<Tube> result;
    for (const auto& object : predictions.front().objects) {
      Tube tube;tube.id=object.id;
      for (std::size_t i = 0; i < predictions.size(); ++i) {
        const auto& list = predictions[i].objects;
        const auto predicted = std::find_if(list.begin(), list.end(),
            [&](const interfaces::Object& item) { return item.id == object.id; });
        tube.boxes[i + 1] = box(*predicted);
      }
      tube.boxes[0] = tube.boxes[1];
      tube.boxes[0].pose.x -= tube.boxes[2].pose.x - tube.boxes[1].pose.x;
      tube.boxes[0].pose.y -= tube.boxes[2].pose.y - tube.boxes[1].pose.y;
      tube.boxes[0].pose.yaw -= angle(tube.boxes[2].pose.yaw - tube.boxes[1].pose.yaw);
      // shortcut: back-project the first half-second linearly; include t=0 in upstream predictions for tighter occupancy.
      prepareTube(tube);
      result.push_back(tube);
    }
    return result;
  }
  static void prepareTube(Tube& tube) {
    double radius = 0.0, turn = 0.0;
    tube.min_x = tube.min_y = std::numeric_limits<double>::infinity();
    tube.max_x = tube.max_y = -std::numeric_limits<double>::infinity();
    double length = 0.0, width = 0.0;
    tube.max_speed = 0.0;
    for (std::size_t i = 0; i < tube.boxes.size(); ++i) {
      const auto& box = tube.boxes[i];
      if (i + 1 < tube.boxes.size())
        tube.max_speed = std::max(tube.max_speed, std::hypot(tube.boxes[i + 1].pose.x - box.pose.x,
                                                             tube.boxes[i + 1].pose.y - box.pose.y) / 0.5);
      tube.min_x = std::min(tube.min_x, box.pose.x);
      tube.max_x = std::max(tube.max_x, box.pose.x);
      tube.min_y = std::min(tube.min_y, box.pose.y);
      tube.max_y = std::max(tube.max_y, box.pose.y);
      length = std::max(length, box.half_length);
      width = std::max(width, box.half_width);
      if (i + 1 < tube.boxes.size()) {
        tube.turn[i] = angle(tube.boxes[i + 1].pose.yaw - box.pose.yaw);
        turn = std::max(turn, std::abs(tube.turn[i]));
      }
    }
    // boxAt uses maximum endpoint extents. Include those extents and the SAT angular padding.
    radius = std::hypot(length, width) * (1.0 + std::sqrt(2.0)*turn);
    tube.min_x -= radius;
    tube.max_x += radius;
    tube.min_y -= radius;
    tube.max_y += radius;
  }
  static Box boxAt(const Tube& tube, double time) {
    const double index = clamp(time / 0.5, 0, 10);
    const std::size_t before = std::min<std::size_t>(9, static_cast<std::size_t>(index));
    const double u = index - before;
    const auto& a = tube.boxes[before];
    const auto& b = tube.boxes[before + 1];
    return {{a.pose.x + u*(b.pose.x - a.pose.x), a.pose.y + u*(b.pose.y - a.pose.y),
             a.pose.yaw + u*tube.turn[before]},
            std::max(a.half_length, b.half_length), std::max(a.half_width, b.half_width)};
  }
  Pose poseAt(const Message& path, const std::vector<double>& station, double s) const {
    const auto after = std::upper_bound(station.begin(), station.end(), s);
    const std::size_t i = std::min<std::size_t>(station.size() - 2, after - station.begin() - 1);
    const double u = clamp((s - station[i]) / (station[i + 1] - station[i]), 0, 1);
    const auto& a = path.points[i].pose.pose;
    const auto& b = path.points[i + 1].pose.pose;
    return {a.position.x + u*(b.position.x - a.position.x),
            a.position.y + u*(b.position.y - a.position.y),
            path_yaw_[i] + u*path_turn_[i]};
  }
  Box egoBox(const Pose& pose) const {
    const double offset = (p_.vehicle_front_m - p_.vehicle_rear_m) / 2;
    return {{pose.x + offset*std::cos(pose.yaw), pose.y + offset*std::sin(pose.yaw), pose.yaw},
            (p_.vehicle_front_m + p_.vehicle_rear_m) / 2, p_.vehicle_half_width_m};
  }
  static double support(const Box& box, double axis) {
    return box.half_length*std::abs(std::cos(box.pose.yaw - axis)) +
           box.half_width*std::abs(std::sin(box.pose.yaw - axis));
  }
  static bool sweptOverlap(const Box& a0, const Box& a1, const Box& b0, const Box& b1, double pad) {
    pad += std::hypot(a0.half_length, a0.half_width)*std::abs(angle(a1.pose.yaw - a0.pose.yaw)) +
           std::hypot(b0.half_length, b0.half_width)*std::abs(angle(b1.pose.yaw - b0.pose.yaw));
    double lo = 0, hi = 1;
    for (double axis : {a0.pose.yaw, a0.pose.yaw + M_PI/2, b0.pose.yaw, b0.pose.yaw + M_PI/2}) {
      const double x = std::cos(axis), y = std::sin(axis);
      const double d0 = (b0.pose.x - a0.pose.x)*x + (b0.pose.y - a0.pose.y)*y;
      const double d1 = (b1.pose.x - a1.pose.x)*x + (b1.pose.y - a1.pose.y)*y;
      const double extent = support(a0, axis) + support(b0, axis) + pad;
      const double delta = d1 - d0;
      if (std::abs(delta) < 1e-12) {
        if (std::abs(d0) > extent) return false;
      } else {
        const double enter = (-extent - d0) / delta, leave = (extent - d0) / delta;
        lo = std::max(lo, std::min(enter, leave));
        hi = std::min(hi, std::max(enter, leave));
        if (lo > hi) return false;
      }
    }
    return true;
  }
  double curvature(const Message&, const std::vector<double>& station, double a, double b) const {
    double maximum = 0;
    const auto first = std::lower_bound(station.begin() + 1, station.end(), a);
    for (std::size_t i = first - station.begin() - 1;
         i + 1 < station.size() && station[i] <= b; ++i) {
      if (station[i + 1] < a) continue;
      maximum = std::max(maximum, path_curvature_[i]);
    }
    return maximum;
  }
  double speedTarget(const std::vector<double>& station, double s) const {
    const auto after = std::upper_bound(station.begin(), station.end(), s);
    const auto index = static_cast<std::size_t>(after - station.begin());
    return path_speed_[index == 0 ? 0 : index - 1];
  }
  bool admissible(const Edge& edge, double cap, double k) const {
    std::vector<double> times{0, edge.h};
    const double activation_time = p_.command_step_sec - edge.start.t;
    if (entered_ && activation_time > 0.0 && activation_time < edge.h)
      times.push_back(activation_time);
    if (std::abs(edge.c3()) > 1e-12) {
      const double t = -edge.c2() / (3*edge.c3());
      if (t > 0 && t < edge.h) times.push_back(t);
    }
    const double a = 3*edge.c3(), b = 2*edge.c2(), c = edge.start.a;
    const double discriminant = b*b - 4*a*c;
    if (std::abs(a) > 1e-12 && discriminant >= 0) {
      for (double t : {(-b - std::sqrt(discriminant))/(2*a), (-b + std::sqrt(discriminant))/(2*a)})
        if (t > 0 && t < edge.h) times.push_back(t);
    } else if (std::abs(b) > 1e-12) {
      const double t = -c / b;
      if (t > 0 && t < edge.h) times.push_back(t);
    }
    for (double t : times) {
      const double v = edge.velocity(t), acceleration = edge.acceleration(t);
      if (v < -1e-9 || v > cap + 1e-9 || acceleration < -p_.max_deceleration_mps2 - 1e-9 ||
          acceleration > p_.max_acceleration_mps2 + 1e-9 || std::abs(edge.jerk(t)) > p_.max_jerk_mps3 + 1e-9 ||
          v*v*k > p_.max_lateral_acceleration_mps2 + 1e-9) return false;
    }
    return true;
  }
  bool clear(const Edge& edge, const Message& path, const std::vector<double>& station,
             const std::vector<Tube>& obstacles, double age, const Launch& model) const {
    const double acceleration_limit = model.acceleration_mps2;
    if (obstacles.empty()) return true;
    const double first_station = edge.start.s, last_station = edge.station(edge.h);
    const auto first_pose = poseAt(path, station, first_station);
    const auto last_pose = poseAt(path, station, last_station);
    double min_x = std::min(first_pose.x, last_pose.x), max_x = std::max(first_pose.x, last_pose.x);
    double min_y = std::min(first_pose.y, last_pose.y), max_y = std::max(first_pose.y, last_pose.y);
    for (auto it = std::upper_bound(station.begin(), station.end(), first_station);
         it != station.end() && *it < last_station; ++it) {
      const auto& point = path.points[it - station.begin()].pose.pose.position;
      min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
    }
    const double k = curvature(path, station, first_station, last_station);
    const double turn = std::min(M_PI, k * std::max(0.0, last_station - first_station));
    const double vmax = std::max(edge.start.v, edge.v1) + acceleration_limit * (edge.h + 0.05);
    const double gap = p_.following_time_gap_sec * vmax;
    const double radius = std::hypot(std::max(p_.vehicle_front_m, p_.vehicle_rear_m),
                                     p_.vehicle_half_width_m);
    const double box_radius = std::hypot((p_.vehicle_front_m + p_.vehicle_rear_m) / 2 + gap/2,
                                         p_.vehicle_half_width_m);
    // Bound every narrow-phase slice, including follow distance and all existing safety padding.
    const double padding = p_.collision_margin_m + tracking_padding_ +
        p_.prediction_uncertainty_mps * (age + edge.start.t + edge.h) +
        (std::max(acceleration_limit, p_.max_deceleration_mps2) + vmax*vmax*k)*0.05*0.05/8 +
        (std::abs(p_.vehicle_front_m - p_.vehicle_rear_m) + box_radius)*turn;
    // Padding is applied along rotated SAT axes; sqrt(2) also encloses their corners in XY.
    const double expansion = radius + gap + std::sqrt(2.0)*padding + 1e-8;
    std::vector<const Tube*> candidates;
    candidates.reserve(obstacles.size());
    for (const auto& tube : obstacles) {
      if (tube.following_rear) continue;
      if (tube.end < age + edge.start.t || tube.start > age + edge.start.t + edge.h) continue;
      const double reach = expansion + model.rear_time_gap_sec * tube.max_speed;
      if (tube.max_x < min_x - reach || tube.min_x > max_x + reach ||
          tube.max_y < min_y - reach || tube.min_y > max_y + reach) continue;
      candidates.push_back(&tube);
    }
    if (candidates.empty()) return true;
    for (double t = 0; t < edge.h - 1e-10;) {
      double end = std::min(edge.h, t + 0.05);
      const double next_slice = (std::floor((age + edge.start.t + t) / 0.5 + 1e-9) + 1)*0.5;
      end = std::min(end, next_slice - age - edge.start.t);
      const auto knot = std::upper_bound(station.begin(), station.end(), edge.station(t) + 1e-9);
      if (knot != station.end() && edge.station(end) > *knot) {
        double lo = t, hi = end;
        for (int i = 0; i < 40; ++i) {
          const double middle = (lo + hi) / 2;
          if (edge.station(middle) < *knot) lo = middle;
          else hi = middle;
        }
        end = hi;
      }
      const double s0 = edge.station(t), s1 = edge.station(end);
      const auto a0 = egoBox(poseAt(path, station, s0));
      const auto a1 = egoBox(poseAt(path, station, s1));
      const double k = curvature(path, station, s0, s1);
      const double dt = end - t;
      const double vmax = std::max(edge.velocity(t), edge.velocity(end)) + acceleration_limit*dt;
      const double deviation = (std::max(acceleration_limit, p_.max_deceleration_mps2) + vmax*vmax*k)*dt*dt/8;
      const double offset = std::abs(p_.vehicle_front_m - p_.vehicle_rear_m) / 2;
      const double pad = p_.collision_margin_m + tracking_padding_ +
                         p_.prediction_uncertainty_mps*(age + edge.start.t + end) +
                         deviation + 2*offset*std::abs(angle(a1.pose.yaw - a0.pose.yaw));
      for (const auto* candidate : candidates) {
        const auto& tube = *candidate;
        const double begin_time = std::max(t, tube.start - age - edge.start.t);
        const double end_time = std::min(end, tube.end - age - edge.start.t);
        if (begin_time > end_time) continue;
        auto from = begin_time == t ? a0 : egoBox(poseAt(path, station, edge.station(begin_time)));
        auto to = end_time == end ? a1 : egoBox(poseAt(path, station, edge.station(end_time)));
        const auto other_from = boxAt(tube, age + edge.start.t + begin_time);
        const auto other_to = boxAt(tube, age + edge.start.t + end_time);
        const double ahead = (other_from.pose.x - from.pose.x)*std::cos(from.pose.yaw) +
                             (other_from.pose.y - from.pose.y)*std::sin(from.pose.yaw);
        const bool aligned = std::abs(angle(other_from.pose.yaw - from.pose.yaw)) < M_PI/3;
        if (ahead > 0 && aligned) {
          // Extend only the front envelope: standing/crossing traffic still uses geometric clearance.
          const double gap = p_.following_time_gap_sec *
              std::max(edge.velocity(begin_time), edge.velocity(end_time));
          for (auto* footprint : {&from, &to}) {
            footprint->pose.x += gap/2*std::cos(footprint->pose.yaw);
            footprint->pose.y += gap/2*std::sin(footprint->pose.yaw);
            footprint->half_length += gap/2;
          }
        } else if (ahead < 0 && aligned && end_time > begin_time) {
          // Traffic closing from behind in the same lane keeps a time gap at its own speed.
          const double gap = model.rear_time_gap_sec *
              std::hypot(other_to.pose.x - other_from.pose.x, other_to.pose.y - other_from.pose.y) /
              (end_time - begin_time);
          for (auto* footprint : {&from, &to}) {
            footprint->pose.x -= gap/2*std::cos(footprint->pose.yaw);
            footprint->pose.y -= gap/2*std::sin(footprint->pose.yaw);
            footprint->half_length += gap/2;
          }
        }
        if (sweptOverlap(from, to, other_from, other_to, pad)) return false;
      }
      t = end;
    }
    return true;
  }
  std::vector<State> movingSchedule(const Message& path,const std::vector<double>& station,
                                    const std::vector<Tube>& obstacles,double age,double velocity,
                                    double acceleration,double,const Launch& model) const {
    // Check the same curve-limited forward motion that is executed after commitment.
    // A slower/braking candidate must never authorize a different, faster entry.
    const double end=5.0-age;
    State q{0,0,velocity,acceleration,0,-1};std::vector<State> launch{q};
    for(;q.t<end-1e-9;) {
      const double h=std::min(p_.time_step_sec,end-q.t);
      const double target=speedTarget(station,q.s);
      const double desired=clamp((target-q.v-q.a*std::abs(q.a)/(2*model.jerk_mps3))/model.command_step_sec,
          -p_.max_deceleration_mps2,model.acceleration_mps2);
      const double a=clamp(desired,q.a-model.jerk_mps3*h,q.a+model.jerk_mps3*h);
      const double v=std::max(0.,q.v+(q.a+a)*h/2);
      const Edge edge{q,h,v,a};const double progress=edge.station(h);
      if(progress>station.back() || !clear(edge,path,station,obstacles,age,model))return {};
      q={q.t+h,progress,v,a,0,-1};launch.push_back(q);
    }
    return launch;
  }
  struct Sample {
    double u = 0.0;
    double station_m = 0.0;
    geometry_msgs::Point point;
  };

  static double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
  }

  double rawStation(const geometry_msgs::Point& point) const {
    double best=std::numeric_limits<double>::infinity(),station=0;
    for(std::size_t i=1;i<raw_.size();++i) {
      const double dx=raw_[i].x-raw_[i-1].x,dy=raw_[i].y-raw_[i-1].y;
      const double u=clamp(((point.x-raw_[i-1].x)*dx+(point.y-raw_[i-1].y)*dy)/(dx*dx+dy*dy),0,1);
      const double d=std::pow(point.x-raw_[i-1].x-u*dx,2)+std::pow(point.y-raw_[i-1].y-u*dy,2);
      if(d<best){best=d;station=raw_station_[i-1]+u*(raw_station_[i]-raw_station_[i-1]);}
    }
    return station;
  }

  geometry_msgs::Point rawPoint(double station_m) const {
    const auto after = std::upper_bound(raw_station_.begin(), raw_station_.end(), station_m);
    const std::size_t i = std::min(static_cast<std::size_t>(after - raw_station_.begin()), raw_.size() - 1);
    const double t = (station_m - raw_station_[i - 1]) / (raw_station_[i] - raw_station_[i - 1]);
    geometry_msgs::Point point;
    point.x = raw_[i - 1].x + t * (raw_[i].x - raw_[i - 1].x);
    point.y = raw_[i - 1].y + t * (raw_[i].y - raw_[i - 1].y);
    point.z = raw_[i - 1].z + t * (raw_[i].z - raw_[i - 1].z);
    return point;
  }

  geometry_msgs::Point control(int index) const {
    if (index >= 0 && index < static_cast<int>(controls_.size())) return controls_[index];
    const auto& a = index < 0 ? controls_.front() : controls_.back();
    const auto& b = index < 0 ? controls_[1] : controls_[controls_.size() - 2];
    geometry_msgs::Point point;
    point.x = 2 * a.x - b.x;
    point.y = 2 * a.y - b.y;
    return point;
  }

  interfaces::PathPoint evaluate(double u) const {
    const int cell = std::min(static_cast<int>(u), static_cast<int>(controls_.size()) - 2);
    const double t = u - cell, v = 1.0 - t;
    const double weights[] = {v * v * v / 6, (3 * t * t * t - 6 * t * t + 4) / 6,
                             (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6, t * t * t / 6};
    const double derivatives[] = {-v * v / 2, 1.5 * t * t - 2 * t,
                                 -1.5 * t * t + t + 0.5, t * t / 2};
    interfaces::PathPoint point;
    double dx = 0.0, dy = 0.0;
    for (int j = 0; j < 4; ++j) {
      const auto c = control(cell + j - 1);
      point.pose.pose.position.x += weights[j] * c.x;
      point.pose.pose.position.y += weights[j] * c.y;
      dx += derivatives[j] * c.x;
      dy += derivatives[j] * c.y;
    }
    point.pose.pose.position.z = rawPoint(p_.reference_start_m + control_step_m_ * u).z;
    const double heading = std::atan2(dy, dx);
    point.pose.pose.orientation.z = std::sin(heading / 2);
    point.pose.pose.orientation.w = std::cos(heading / 2);
    point.velocity = std::numeric_limits<double>::quiet_NaN();
    return point;
  }

  double parameterAt(double station_m) const {
    const auto after = std::upper_bound(samples_.begin(), samples_.end(), station_m,
        [](double value, const Sample& sample) { return value < sample.station_m; });
    const std::size_t i = std::min(static_cast<std::size_t>(after - samples_.begin()), samples_.size() - 1);
    const auto& a = samples_[i - 1];
    const auto& b = samples_[i];
    return a.u + (station_m - a.station_m) * (b.u - a.u) / (b.station_m - a.station_m);
  }

  std::deque<EgoPose> ego_history_;
  double last_speed_ = 0;
  double last_speed_time_ = -std::numeric_limits<double>::infinity();
  double last_acceleration_ = 0;
  double tracking_padding_ = 0;
  double entry_line_m_ = 0.0, yield_station_m_ = 0.0, departure_station_m_ = 0.0;
  double previous_station_m_ = 0.0;
  double gap_since_sec_ = -1.0, yield_since_sec_ = 0.0;
  double planning_speed_mps_ = 0.0, planning_end_m_ = 0.0;
  bool entered_ = false, gate_open_ = false,stopped_at_line_=false,relaxed_=false;
  std::vector<geometry_msgs::Point> zone_;
  std::vector<double> zone_station_;
  double conflict_station_m_ = 0.0;
  std::map<std::uint32_t,std::deque<std::pair<double,geometry_msgs::Point>>> sightings_;
  std::uint32_t zone_blocker_ = 0;
  Phase phase_ = Phase::Approach;
  std::vector<double> path_yaw_, path_turn_, path_curvature_, path_speed_;
  RoundaboutRollingEntryParameters p_;
  std::vector<geometry_msgs::Point> raw_, controls_;
  std::vector<double> raw_station_;
  std::vector<Sample> samples_;
  double control_step_m_ = 0.0;
};
}  // namespace path_planner
namespace path_planner {
class RoundaboutRollingEntry {
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
  RoundaboutRollingEntry(RoundaboutRollingEntryParameters p,SharedMap& map):mission_(std::move(p),*map.map) {}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& competition,
      const interfaces::Objects& objects,const PredictedObjects& predictions,const ros::Time& stamp) {
    return publishLegacyPrefix(mission_.process(ego,competition,objects,predictions,stamp),
        std::max(0.,double(competition.velocity_x_mps)),20.);
  }
  const char* planningOutcome() const {return "rolling-roundabout";}
 private:
  RoundaboutRollingEntryMission mission_;
};
}
