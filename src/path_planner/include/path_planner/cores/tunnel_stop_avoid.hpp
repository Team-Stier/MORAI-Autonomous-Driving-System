#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <interfaces/Path.h>
#include <ros/console.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace path_planner {
struct TunnelStopAvoidParameters {
  bool tunnel_region_enabled=true;
  double tunnel_origin_x_m=-23.5,tunnel_origin_y_m=-519.2000122070312;
  double tunnel_axis_yaw_rad=-2.6434409392630127;
  double tunnel_min_station_m=-58.61842773450474,tunnel_max_station_m=58.61843533549703;
  double tunnel_right_wall_m=-6.01444403868076,tunnel_left_wall_m=5.995789344373857;
  double cruise_speed_mps=25./3.6, approach_speed_mps=20./3.6, avoidance_speed_mps=8./3.6;
  double reacquisition_speed_mps=10./3.6, unclassified_timeout_sec=2.;
  double detection_distance_m=45., stop_gap_m=12., stopping_deceleration_mps2=2.;
  double stopping_response_sec=.3, stopped_speed_mps=.1, stopped_confirmation_sec=.3;
  double vehicle_front_m=3.845008, vehicle_rear_m=.79, vehicle_width_m=2.160836;
  double wheelbase_m=3., obstacle_margin_m=.3, road_margin_m=.05;
  double minimum_object_extent_m=1.2, dynamic_minimum_extent_m=.6;
  double dynamic_lateral_speed_mps=.5, dynamic_direction_ratio=1.5;
  double dynamic_displacement_m=.3, dynamic_span_sec=.5;
  double stationary_speed_mps=.25, stationary_span_sec=1.;
  double stationary_oscillation_ratio=.35, stationary_longitudinal_span_m=2.;
  double stationary_lateral_span_m=.3;
  double history_sec=4., observation_freshness_sec=.4;
  double clear_margin_m=.15, clear_confirmation_sec=.3;
  double return_length_m=18., return_clearance_m=1.5;
  double steering_limit_rad=.537561409, steering_rate_radps=.806342114;
  double lateral_acceleration_mps2=1.5;
  double reference_smoothing_m=1.;
};

// Local Stage Core selected through the existing section mapping.
// Track history, maneuver commitment and occupancy belong to this Core.
class TunnelStopAvoid {
 public:
  explicit TunnelStopAvoid(TunnelStopAvoidParameters parameters, SharedMap& map)
      : p_(parameters), map_(map.map) {}

  interfaces::Path process(const nav_msgs::Odometry& ego,
      const interfaces::CompetitionVehicleStatus& status, const interfaces::Objects& objects,
      const Route& route, const ros::Time& stamp) {
    const double now=stamp.toSec();
    if(now<last_time_)reset();
    last_time_=now;
    reference_z_=ego.pose.pose.position.z;
    const Pose vehicle=pose(ego.pose.pose);
    const double speed=std::max(0.,double(status.velocity_x_mps));
    if(state_==State::Cruise)buildRoad(vehicle,route);
    if(!target_ && p_.tunnel_region_enabled && !insideTunnel(vehicle.x,vehicle.y) &&
        !insideTunnel(vehicle.x+p_.vehicle_front_m*std::cos(vehicle.yaw),
                      vehicle.y+p_.vehicle_front_m*std::sin(vehicle.yaw)))
      return straight(vehicle,p_.cruise_speed_mps);
    observe(objects,now,vehicle);
    const Projection car=project(vehicle.x,vehicle.y);
    for(std::size_t i=0;i<tracks_.size();++i) {
      const auto& other=tracks_[i];
      if(other.dynamic && !other.released && other.s>car.s-p_.vehicle_rear_m &&
          (!target_ || i!=target_index_)) {
        target_=true;target_index_=i;committed_.clear();
        clear_since_=-1;last_clear_sample_=-1;clear_samples_=0;
        break;
      }
    }
    if(!target_)return straight(vehicle,p_.cruise_speed_mps);
    auto& target=tracks_[target_index_];
    if(speed<=p_.stopped_speed_mps) {
      if(stopped_since_<0)stopped_since_=now;
    } else stopped_since_=-1;
    const bool stopped=stopped_since_>=0 && now-stopped_since_>=p_.stopped_confirmation_sec;

    // Direct travel resumes only after newly observed outward clearance.
    if(target.dynamic && !target.released) {
      const bool fresh=now-target.last_seen<=p_.observation_freshness_sec;
      const double half=std::max(p_.dynamic_minimum_extent_m/2,target.latest_half_l);
      // Two measured footprints must already be outside the vehicle corridor.
      // Recent outward motion may establish the additional clearance margin
      // during confirmation, but cannot clear an unseen footprint inside it.
      const bool clear=std::abs(target.l)>p_.vehicle_width_m/2+half;
      const bool outward=target.l*target.v_l>=0;
      const double age=std::max(0.,now-target.last_seen);
      const bool margin_clear=std::abs(target.l+age*target.v_l)>
          p_.vehicle_width_m/2+half+p_.clear_margin_m;
      if(fresh && clear && outward) {
        if(clear_since_<0)clear_since_=now;
        if(target.last_seen>last_clear_sample_) {++clear_samples_;last_clear_sample_=target.last_seen;}
        if(clear_samples_>=2 && margin_clear && now-clear_since_>=p_.clear_confirmation_sec) {
          target.released=true;
          if(makeAvoidance(vehicle,target,speed,true)) {
            state_=State::Return;
            ROS_INFO("[tunnel] observed corridor clear; return to nominal lane");
            return pathMessage(committed_,0);
          }
          target.released=false;
        }
      } else {clear_since_=-1;last_clear_sample_=-1;clear_samples_=0;}
    }
    if(state_==State::Avoid || state_==State::Return || state_==State::Replan) {
      const double rear_clear=target.s+target.half_s+p_.vehicle_rear_m+p_.return_clearance_m;
      if(state_!=State::Replan && car.s>rear_clear)state_=State::Return;
      if(state_==State::Return && (target.released || car.s>rear_clear) && std::abs(car.l)<.15 &&
          std::abs(wrap(vehicle.yaw-at(car.s).yaw))<.08) {
        target.released=true;target_=false;state_=State::Cruise;committed_.clear();
        ROS_INFO("[tunnel] stationary bypass complete");
        return straight(vehicle,p_.cruise_speed_mps);
      }
      const std::size_t begin=nearest(committed_,vehicle);
      if(pathClear(committed_,begin,speed)) {
        if(state_==State::Replan)state_=State::Avoid;
        return pathMessage(committed_,begin);
      }
      if(makeAvoidance(vehicle,target,speed)) {
        state_=State::Avoid;
        return pathMessage(committed_,0);
      }
      state_=State::Replan;
      ROS_WARN_THROTTLE(1.,"[tunnel] no continuous collision-free update; hold and retain bypass");
      auto result=hold(ego);
      for(std::size_t i=begin;i<committed_.size();++i) {
        geometry_msgs::Pose q;
        q.position.x=committed_[i].x;q.position.y=committed_[i].y;q.position.z=reference_z_;
        q.orientation.z=std::sin(committed_[i].yaw/2);q.orientation.w=std::cos(committed_[i].yaw/2);
        result.lateral_reference.push_back(q);
      }
      return result;
    }
    // Finish an emergency stop before retrying a maneuver. Otherwise an
    // uncertain classification is never a prerequisite for a feasible bypass.
    if(state_!=State::Classify || stopped) {
      if(makeAvoidance(vehicle,target,speed)) {
        state_=State::Avoid;
        ROS_INFO("[tunnel] collision-free bypass; start without classification wait");
        return pathMessage(committed_,0);
      }
    }
    state_=State::Classify;
    return hold(ego);
  }

 private:
  struct Pose {double x=0,y=0,yaw=0,s=0;};
  struct Projection {double s=0,l=0;};
  struct Sample {double t=0,s=0,l=0,half_s=0,half_l=0;};
  struct Track {
    unsigned id=0;
    std::deque<Sample> samples;
    double s=0,l=0,half_s=.6,half_l=.6,latest_half_l=.3;
    double observed_half_s=0,observed_half_l=0;
    double last_seen=0,v_s=0,v_l=0;
    bool dynamic=false,stationary=false,released=false;
  };
  enum class State {Cruise,Approach,Classify,WaitDynamic,Avoid,Return,Replan};
  TunnelStopAvoidParameters p_;
  std::shared_ptr<lanelet::LaneletMap> map_;
  std::vector<Pose> reference_,committed_;
  std::vector<std::vector<Pose>> road_;
  std::vector<Track> tracks_;
  State state_=State::Cruise;
  bool target_=false;
  std::size_t target_index_=0;
  double last_time_=0,last_cloud_time_=-1,stopped_since_=-1,clear_since_=-1;
  double last_clear_sample_=-1;
  double reference_z_=0;
  unsigned clear_samples_=0;

  static double wrap(double x) {return std::atan2(std::sin(x),std::cos(x));}
  static Pose pose(const geometry_msgs::Pose& q) {
    const auto& r=q.orientation;
    return {q.position.x,q.position.y,std::atan2(2*(r.w*r.z+r.x*r.y),1-2*(r.y*r.y+r.z*r.z)),0};
  }
  static double median(std::vector<double> x) {
    std::sort(x.begin(),x.end());const auto n=x.size();
    return n%2?x[n/2]:(x[n/2-1]+x[n/2])/2;
  }
  void reset() {
    target_=false;tracks_.clear();reference_.clear();road_.clear();committed_.clear();
    state_=State::Cruise;last_cloud_time_=-1;stopped_since_=clear_since_=-1;
    last_clear_sample_=-1;clear_samples_=0;
  }
  void buildRoad(const Pose& vehicle,const Route& route) {
    double best=std::numeric_limits<double>::infinity();
    lanelet::ConstLanelet chosen=route->front();
    for(const auto& lane:*route) {
      const auto center=lane.centerline3d();
      for(std::size_t i=1;i<center.size();++i) {
        const double dx=center[i].x()-center[i-1].x(),dy=center[i].y()-center[i-1].y();
        const double length=std::hypot(dx,dy);
        if(length<1e-6)continue;
        const double angle=std::atan2(dy,dx);
        if(std::abs(wrap(angle-vehicle.yaw))>1.2)continue;
        const double u=std::max(0.,std::min(1.,((vehicle.x-center[i-1].x())*dx+
            (vehicle.y-center[i-1].y())*dy)/(length*length)));
        const double distance=std::hypot(vehicle.x-center[i-1].x()-u*dx,
            vehicle.y-center[i-1].y()-u*dy);
        const double score=distance+.3*std::abs(wrap(angle-vehicle.yaw));
        if(score<best) {best=score;chosen=lane;}
      }
    }
    reference_.clear();road_.clear();
    // Pair the two boundaries by normalized arc length. The native Lanelet
    // centerline has irregular supports; differentiating these directly can
    // reject a traversable maneuver on steering-rate noise.
    const auto boundary=[](const lanelet::ConstLineString3d& line) {
      std::vector<Pose> result;double length=0;
      for(std::size_t i=0;i<line.size();++i) {
        if(i)length+=std::hypot(line[i].x()-line[i-1].x(),line[i].y()-line[i-1].y());
        result.push_back({line[i].x(),line[i].y(),0,length});
      }
      return result;
    };
    const auto sample=[](const std::vector<Pose>& line,double fraction) {
      const double s=line.back().s*fraction;
      auto b=std::upper_bound(line.begin(),line.end(),s,
          [](double value,const Pose& q){return value<q.s;});
      if(b==line.begin())++b;
      if(b==line.end())--b;
      const auto& a=*(b-1);const double u=(s-a.s)/(b->s-a.s);
      return Pose{a.x+u*(b->x-a.x),a.y+u*(b->y-a.y),0,0};
    };
    std::vector<lanelet::ConstLanelet> lanes;
    bool started=false;double length=0,vehicle_station=0;
    double closest=std::numeric_limits<double>::infinity();
    for(const auto& lane:*route) {
      if(lane.id()==chosen.id())started=true;
      if(!started)continue;
      lanes.push_back(lane);
      const auto left=boundary(lane.leftBound3d()),right=boundary(lane.rightBound3d());
      const int n=std::max(2,int(std::ceil(std::max(left.back().s,right.back().s)/.2))+1);
      for(int i=0;i<n;++i) {
        const auto a=sample(left,i/double(n-1)),b=sample(right,i/double(n-1));
        Pose q{(a.x+b.x)/2,(a.y+b.y)/2,0,0};
        if(!reference_.empty()) {
          const double distance=std::hypot(q.x-reference_.back().x,q.y-reference_.back().y);
          if(distance<.01)continue;
          length+=distance;
        }
        reference_.push_back(q);
        const double distance_to_vehicle=std::hypot(q.x-vehicle.x,q.y-vehicle.y);
        if(distance_to_vehicle<closest) {closest=distance_to_vehicle;vehicle_station=length;}
      }
      if(length-vehicle_station>=100)break;
    }
    const int count=static_cast<int>(reference_.size());
    const auto raw=reference_;
    const double sigma=p_.reference_smoothing_m/.2;
    const int support=int(std::ceil(3*sigma));
    for(int i=0;i<count;++i) {
      double x=0,y=0,total=0;
      for(int j=-support;j<=support;++j) {
        const int index=std::max(0,std::min(count-1,i+j));
        const double weight=std::exp(-.5*j*j/(sigma*sigma));
        x+=weight*raw[index].x;y+=weight*raw[index].y;total+=weight;
      }
      reference_[i].x=x/total;reference_[i].y=y/total;
    }
    for(int i=0;i<count;++i) {
      const auto& a=reference_[i?i-1:i];const auto& b=reference_[i+1<count?i+1:i];
      reference_[i].yaw=std::atan2(b.y-a.y,b.x-a.x);
      if(i)reference_[i].s=reference_[i-1].s+std::hypot(reference_[i].x-reference_[i-1].x,
          reference_[i].y-reference_[i-1].y);
    }
    for(const auto& lane:map_->laneletLayer) {
      bool included=false;
      for(const auto& support_lane:lanes)included=included || lane.id()==support_lane.id() ||
          lane.leftBound3d().id()==support_lane.rightBound3d().id() ||
          lane.rightBound3d().id()==support_lane.leftBound3d().id();
      if(!included)continue;
      std::vector<Pose> polygon;
      for(const auto& q:lane.polygon3d())polygon.push_back({q.x(),q.y(),0,0});
      road_.push_back(std::move(polygon));
    }
  }
  Pose at(double s,double l=0)const {
    auto b=std::upper_bound(reference_.begin(),reference_.end(),s,
        [](double value,const Pose& q){return value<q.s;});
    if(b==reference_.begin())++b;
    if(b==reference_.end())--b;
    const auto& a=*(b-1);const double u=std::max(0.,std::min(1.,(s-a.s)/(b->s-a.s)));
    const double yaw=a.yaw+u*wrap(b->yaw-a.yaw);
    return {a.x+u*(b->x-a.x)-std::sin(yaw)*l,
        a.y+u*(b->y-a.y)+std::cos(yaw)*l,yaw,s};
  }
  Projection project(double x,double y)const {
    Projection result;double best=std::numeric_limits<double>::infinity();
    for(std::size_t i=1;i<reference_.size();++i) {
      const auto& a=reference_[i-1];const auto& b=reference_[i];
      const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
      if(length<1e-6)continue;
      const double u=std::max(0.,std::min(1.,((x-a.x)*dx+(y-a.y)*dy)/(length*length)));
      const double ex=x-a.x-u*dx,ey=y-a.y-u*dy,d=std::hypot(ex,ey);
      if(d<best) {best=d;result={a.s+u*length,(-dy*ex+dx*ey)/length};}
    }
    return result;
  }
  bool insideTunnel(double x,double y)const {
    const double c=std::cos(p_.tunnel_axis_yaw_rad),s=std::sin(p_.tunnel_axis_yaw_rad);
    const double dx=x-p_.tunnel_origin_x_m,dy=y-p_.tunnel_origin_y_m;
    const double station=c*dx+s*dy,lateral=-s*dx+c*dy;
    return station>=p_.tunnel_min_station_m && station<=p_.tunnel_max_station_m &&
        lateral>=p_.tunnel_right_wall_m && lateral<=p_.tunnel_left_wall_m;
  }
  bool touchesTunnel(const interfaces::Object& object)const {
    const double c=std::cos(p_.tunnel_axis_yaw_rad),s=std::sin(p_.tunnel_axis_yaw_rad);
    const double dx=object.position.x-p_.tunnel_origin_x_m,dy=object.position.y-p_.tunnel_origin_y_m;
    const double x=c*dx+s*dy-(p_.tunnel_min_station_m+p_.tunnel_max_station_m)/2;
    const double y=-s*dx+c*dy-(p_.tunnel_right_wall_m+p_.tunnel_left_wall_m)/2;
    const double hs=(p_.tunnel_max_station_m-p_.tunnel_min_station_m)/2;
    const double hl=(p_.tunnel_left_wall_m-p_.tunnel_right_wall_m)/2;
    const double angle=pose(object.bbox_pose).yaw-p_.tunnel_axis_yaw_rad;
    for(double axis:{0.,1.5707963267948966,angle,angle+1.5707963267948966}) {
      const double distance=std::abs(x*std::cos(axis)+y*std::sin(axis));
      const double region=hs*std::abs(std::cos(axis))+hl*std::abs(std::sin(axis));
      const double extent=object.bbox_size.x/2*std::abs(std::cos(angle-axis))+
          object.bbox_size.y/2*std::abs(std::sin(angle-axis));
      if(distance>region+extent)return false;
    }
    return true;
  }
  void observe(const interfaces::Objects& objects,double now,const Pose& vehicle) {
    const double time=objects.header.stamp.toSec();
    if(time<=last_cloud_time_)return;
    last_cloud_time_=time;
    const double car_s=project(vehicle.x,vehicle.y).s;
    for(const auto& object:objects.objects) {
      if(p_.tunnel_region_enabled && !touchesTunnel(object))continue;
      const auto q=project(object.position.x,object.position.y);
      if(q.s<car_s-p_.vehicle_rear_m || q.s>car_s+p_.detection_distance_m || std::abs(q.l)>6)continue;
      auto it=std::find_if(tracks_.begin(),tracks_.end(),
          [&](const Track& t){return t.id==object.id;});
      if(it==tracks_.end()) {
        double best=1.5;
        for(auto candidate=tracks_.begin();candidate!=tracks_.end();++candidate) {
          if(candidate->released || candidate->dynamic ||
              now-candidate->last_seen<=p_.observation_freshness_sec)continue;
          const bool still_observed=std::any_of(objects.objects.begin(),objects.objects.end(),
              [&](const interfaces::Object& q){return q.id==candidate->id;});
          if(still_observed)continue;
          const double distance=std::hypot(q.s-candidate->s,q.l-candidate->l);
          if(distance<best) {best=distance;it=candidate;}
        }
        if(it!=tracks_.end())it->id=object.id;
      }
      if(it==tracks_.end()) {
        Track track;track.id=object.id;track.half_s=track.half_l=p_.minimum_object_extent_m/2;
        tracks_.push_back(track);it=tracks_.end()-1;
      }
      auto& track=*it;
      const double yaw=pose(object.bbox_pose).yaw-at(q.s).yaw;
      const double hs=(std::abs(std::cos(yaw))*object.bbox_size.x+
          std::abs(std::sin(yaw))*object.bbox_size.y)/2;
      const double hl=(std::abs(std::sin(yaw))*object.bbox_size.x+
          std::abs(std::cos(yaw))*object.bbox_size.y)/2;
      track.observed_half_s=std::max(track.observed_half_s,hs);
      track.observed_half_l=std::max(track.observed_half_l,hl);
      if(track.released && track.dynamic &&
          std::abs(q.l)<=p_.vehicle_width_m/2+std::max(p_.dynamic_minimum_extent_m/2,hl)+p_.clear_margin_m)
        track.released=false;
      track.latest_half_l=hl;track.last_seen=now;track.samples.push_back({time,q.s,q.l,hs,hl});
      while(time-track.samples.front().t>p_.history_sec)track.samples.pop_front();
      std::vector<double> s,l;
      for(const auto& sample:track.samples) {s.push_back(sample.s);l.push_back(sample.l);}
      track.s=median(s);track.l=median(l);
      // Keep measured physical size; update observation uncertainty from the
      // current window without adding the size prior to every centre offset.
      track.half_s=std::max(p_.minimum_object_extent_m/2,track.observed_half_s);
      track.half_l=std::max(p_.minimum_object_extent_m/2,track.observed_half_l);
      for(const auto& sample:track.samples) {
        track.half_s=std::max(track.half_s,std::abs(sample.s-track.s)+sample.half_s);
        track.half_l=std::max(track.half_l,std::abs(sample.l-track.l)+sample.half_l);
      }
      const auto motion=[&](double window,double span,double interval,double& vs,double& vl) {
        std::vector<Sample> a;
        for(const auto& sample:track.samples)if(time-sample.t<=window)a.push_back(sample);
        if(a.size()<4 || a.back().t-a.front().t<span)return false;
        std::vector<double> ds,dl;
        for(std::size_t i=0;i<a.size();++i)for(std::size_t j=i+1;j<a.size();++j) {
          const double dt=a[j].t-a[i].t;
          if(dt>=interval) {ds.push_back((a[j].s-a[i].s)/dt);dl.push_back((a[j].l-a[i].l)/dt);}
        }
        if(ds.empty())return false;
        vs=median(ds);vl=median(dl);return true;
      };
      double vs=0,vl=0;
      if(motion(1.5,p_.dynamic_span_sec,.3,vs,vl) &&
          std::abs(vl)>=p_.dynamic_lateral_speed_mps &&
          std::abs(vl)>=p_.dynamic_direction_ratio*std::abs(vs)) {
        const auto first=std::find_if(track.samples.begin(),track.samples.end(),
            [&](const Sample& sample){return time-sample.t<=1.5;});
        if(std::abs(track.samples.back().l-first->l)>=p_.dynamic_displacement_m)track.dynamic=true;
      }
      track.v_s=vs;track.v_l=vl;
      if(!track.dynamic && motion(p_.history_sec,p_.stationary_span_sec,.7,vs,vl) &&
          std::hypot(vs,vl)<=p_.stationary_speed_mps)track.stationary=true;
      // Occlusion can move the fitted center back and forth along the road.
      // Require bounded support, repeated direction reversals and little net
      // progress; absence of lateral motion alone is not stationary evidence.
      if(!track.dynamic && track.samples.size()>=4 &&
          time-track.samples.front().t>=p_.stationary_span_sec) {
        double min_s=track.samples.front().s,max_s=min_s,min_l=track.samples.front().l,max_l=min_l;
        double travel=0;int previous_sign=0;unsigned reversals=0;
        for(std::size_t i=0;i<track.samples.size();++i) {
          const auto& q=track.samples[i];
          min_s=std::min(min_s,q.s);max_s=std::max(max_s,q.s);
          min_l=std::min(min_l,q.l);max_l=std::max(max_l,q.l);
          if(i) {
            const double ds=q.s-track.samples[i-1].s;travel+=std::abs(ds);
            const int sign=ds>.05?1:(ds<-.05?-1:0);
            if(sign && previous_sign && sign!=previous_sign)++reversals;
            if(sign)previous_sign=sign;
          }
        }
        if(reversals>=2 && max_s-min_s<=p_.stationary_longitudinal_span_m &&
            max_l-min_l<=p_.stationary_lateral_span_m &&
            std::abs(track.samples.back().s-track.samples.front().s)<=p_.stationary_oscillation_ratio*travel) {
          track.stationary=true;
        }
      }
      if(track.dynamic) {track.s=q.s;track.l=q.l;track.stationary=false;}
    }
    if(!target_) {
      const auto guide=straight(vehicle,p_.cruise_speed_mps);
      double nearest_s=std::numeric_limits<double>::infinity();
      for(std::size_t i=0;i<tracks_.size();++i) {
        const auto& track=tracks_[i];
        if(track.released || track.s<=car_s || track.s>=nearest_s)continue;
        // An object in the adjacent lane is not automatically a stop target.
        // Include the actual lane-return geometry and the whole vehicle body.
        const bool blocking=std::any_of(guide.points.begin(),guide.points.end(),
            [&](const interfaces::PathPoint& q) {
              return overlap(pose(q.pose.pose),track,p_.obstacle_margin_m);
            });
        if(blocking) {nearest_s=track.s;target_index_=i;target_=true;}
      }
      if(target_) {state_=State::Approach;stopped_since_=-1;}
    }
  }
  static bool inside(double x,double y,const std::vector<Pose>& polygon) {
    bool in=false;
    for(std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
      const auto& a=polygon[i];const auto& b=polygon[j];
      const double dx=b.x-a.x,dy=b.y-a.y;
      const double cross=(x-a.x)*dy-(y-a.y)*dx;
      if(std::abs(cross)<1e-8 && (x-a.x)*(x-b.x)+(y-a.y)*(y-b.y)<=0)return true;
      if((a.y>y)!=(b.y>y) && x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x)in=!in;
    }
    return in;
  }
  bool withinRoad(const Pose& q)const {
    // Sample every edge of the expanded full vehicle footprint, including
    // the shared boundary between the two permitted lane polygons.
    const double front=p_.vehicle_front_m+p_.road_margin_m,rear=p_.vehicle_rear_m+p_.road_margin_m;
    const double half=p_.vehicle_width_m/2+p_.road_margin_m;
    const double c=std::cos(q.yaw),s=std::sin(q.yaw);
    for(int edge=0;edge<4;++edge)for(int i=0;i<=24;++i) {
      const double u=i/24.;
      const double x=edge<2?-rear+u*(front+rear):(edge==2?front:-rear);
      const double y=edge<2?(edge==0?half:-half):-half+u*2*half;
      bool found=false;
      for(const auto& polygon:road_)if(inside(q.x+c*x-s*y,q.y+s*x+c*y,polygon)){found=true;break;}
      if(!found)return false;
    }
    return true;
  }
  bool overlap(const Pose& q,const Track& track,double margin)const {
    const Pose center=at(track.s,track.l);
    const double forward=(p_.vehicle_front_m-p_.vehicle_rear_m)/2;
    const double x=q.x+forward*std::cos(q.yaw)-center.x;
    const double y=q.y+forward*std::sin(q.yaw)-center.y;
    const double length=(p_.vehicle_front_m+p_.vehicle_rear_m)/2;
    const double width=p_.vehicle_width_m/2;
    const double hs=track.half_s+margin,hl=track.half_l+margin;
    for(double axis:{q.yaw,q.yaw+1.5707963267948966,center.yaw,center.yaw+1.5707963267948966}) {
      const double distance=std::abs(x*std::cos(axis)+y*std::sin(axis));
      const double r1=length*std::abs(std::cos(q.yaw-axis))+width*std::abs(std::sin(q.yaw-axis));
      const double r2=hs*std::abs(std::cos(center.yaw-axis))+hl*std::abs(std::sin(center.yaw-axis));
      if(distance>r1+r2)return false;
    }
    return true;
  }
  double traversalSpeed(double speed,double distance)const {
    const double braking_distance=std::max(0.,distance-speed*p_.stopping_response_sec);
    return std::sqrt(std::max(p_.avoidance_speed_mps*p_.avoidance_speed_mps,
        speed*speed-2*p_.stopping_deceleration_mps2*braking_distance));
  }
  bool pathClear(const std::vector<Pose>& path,std::size_t begin,double speed)const {
    double time=0,distance=0;
    for(std::size_t i=begin;i<path.size();++i) {
      if(i>begin) {
        const double arc=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
        distance+=arc;time+=arc/traversalSpeed(speed,distance);
      }
      for(const auto& track:tracks_) {
        if(track.released && !track.dynamic)continue;
        // Crossing clearance removes the old swept history, not the current
        // footprint. An off-centre vehicle must still avoid the departing body.
        auto occupied=track;
        if(track.released) {
          occupied.half_s=std::max(p_.dynamic_minimum_extent_m/2,track.samples.back().half_s);
          occupied.half_l=std::max(p_.dynamic_minimum_extent_m/2,track.samples.back().half_l);
        }
        // Missing observations never remove an uncleared occupied region.
        if(overlap(path[i],occupied,track.released?0:p_.obstacle_margin_m))return false;
        if(std::hypot(track.v_s,track.v_l)<p_.stationary_speed_mps)continue;
        for(double uncertainty:{-.5,0.,.5}) {
          const double future=std::max(0.,std::min(5.,time+uncertainty));
          auto predicted=track;
          predicted.s+=track.v_s*future;predicted.l+=track.v_l*future;
          predicted.half_s=std::max(p_.minimum_object_extent_m/2,track.observed_half_s);
          predicted.half_l=std::max(p_.minimum_object_extent_m/2,track.observed_half_l);
          if(overlap(path[i],predicted,p_.obstacle_margin_m))return false;
        }
      }
    }
    return true;
  }
  bool makeAvoidance(const Pose& vehicle,const Track& track,double speed,bool return_to_lane=false) {
    const auto car=project(vehicle.x,vehicle.y);
    const double available=track.s-track.half_s-p_.vehicle_front_m-.7-car.s;
    const double hold_end=track.s+track.half_s+p_.vehicle_rear_m+p_.return_clearance_m;
    const bool returning=return_to_lane || car.s>=hold_end;
    const double end=return_to_lane?std::max(car.s+45,hold_end+p_.return_length_m):
        std::max(car.s,hold_end)+p_.return_length_m;
    if(end+p_.vehicle_front_m>reference_.back().s)return false;
    double second_derivative=0;
    if(committed_.size()>2) {
      const auto i=std::max<std::size_t>(1,std::min(committed_.size()-2,nearest(committed_,vehicle)));
      const auto a=project(committed_[i-1].x,committed_[i-1].y);
      const auto b=project(committed_[i].x,committed_[i].y);
      const auto c=project(committed_[i+1].x,committed_[i+1].y);
      if(b.s-a.s>1e-4 && c.s-b.s>1e-4)
        second_derivative=2*((c.l-b.l)/(c.s-b.s)-(b.l-a.l)/(b.s-a.s))/(c.s-a.s);
    }
    const double check_speed=std::max(speed,p_.avoidance_speed_mps);
    std::vector<double> lengths{30.,26.,22.,18.,16.,14.,12.,10.,8.,6.,4.,2.};
    if(available>0 && available<18)lengths.push_back(available);
    double best=std::numeric_limits<double>::infinity();
    std::vector<Pose> candidate;
    unsigned collision_count=0,road_count=0,steering_count=0,rate_count=0,acceleration_count=0;
    for(int sign:{1,-1})for(double extra:{0.,.3,.6,.9,1.2}) {
      if(returning && (sign<0 || extra!=0))continue;
      const double distance=track.half_l+p_.vehicle_width_m/2+p_.obstacle_margin_m+.1+extra;
      const double offset=returning?0:track.l+sign*distance;
      for(const double length:lengths) {
        // Feasibility is determined along the entire vehicle footprint.
        // A distance to the obstacle alone cannot bound an ongoing lateral move.
        std::vector<Pose> path;
        const double slope=std::tan(wrap(vehicle.yaw-at(car.s).yaw));
        for(double station=car.s;station<=end;station+=.2) {
          const double u=std::max(0.,std::min(1.,(station-car.s)/length));
          const double blend=u*u*u*(10+u*(-15+6*u));
          double l=car.l+(offset-car.l)*blend+slope*length*(u-6*u*u*u+8*u*u*u*u-3*u*u*u*u*u)
              +.5*second_derivative*length*length*u*u*(1-u)*(1-u)*(1-u);
          const double r=std::max(0.,std::min(1.,(station-hold_end)/p_.return_length_m));
          if(!returning)l*=1-r*r*r*(10+r*(-15+6*r));
          path.push_back(at(station,l));
        }
        for(std::size_t i=0;i<path.size();++i) {
          const auto& a=path[i?i-1:i];const auto& b=path[i+1<path.size()?i+1:i];
          path[i].yaw=std::atan2(b.y-a.y,b.x-a.x);
        }
        path.front().x=vehicle.x;path.front().y=vehicle.y;path.front().yaw=vehicle.yaw;
        bool feasible=pathClear(path,0,check_speed);
        double previous=0,travel=0;bool have_previous=false;
        if(!feasible)++collision_count;
        for(std::size_t i=1;i+1<path.size() && feasible;++i) {
          const double arc=std::hypot(path[i+1].x-path[i-1].x,path[i+1].y-path[i-1].y);
          travel+=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
          const double local_speed=traversalSpeed(check_speed,travel);
          const double k=wrap(path[i+1].yaw-path[i-1].yaw)/arc;
          const double angle=std::atan(p_.wheelbase_m*k);
          if(std::abs(angle)>p_.steering_limit_rad) {feasible=false;++steering_count;}
          else if(std::abs(k)*local_speed*local_speed>p_.lateral_acceleration_mps2)
            {feasible=false;++acceleration_count;}
          else if(have_previous && std::abs(angle-previous)/(.2/local_speed)>p_.steering_rate_radps)
            {feasible=false;++rate_count;}
          else if(!withinRoad(path[i])) {feasible=false;++road_count;}
          previous=angle;have_previous=true;
        }
        const double cost=std::abs(offset-car.l)+.02*(18-length);
        if(feasible && cost<best) {best=cost;candidate=std::move(path);}
      }
    }
    if(candidate.empty())ROS_WARN_THROTTLE(1.,
        "[tunnel] no bypass: available=%.2f collision=%u road=%u steer=%u rate=%u lateral_acceleration=%u",
        available,collision_count,road_count,steering_count,rate_count,acceleration_count);
    if(candidate.empty())return false;
    committed_=std::move(candidate);
    return true;
  }
  static std::size_t nearest(const std::vector<Pose>& path,const Pose& car) {
    double best=std::numeric_limits<double>::infinity();std::size_t result=0;
    for(std::size_t i=0;i<path.size();++i) {
      const double distance=std::hypot(path[i].x-car.x,path[i].y-car.y);
      if(distance<best){best=distance;result=i;}
    }
    return result+1<path.size()?result:path.size()-2;
  }
  interfaces::Path pathMessage(const std::vector<Pose>& path,std::size_t begin)const {
    interfaces::Path result;
    for(std::size_t i=begin;i<path.size();++i) {
      interfaces::PathPoint q;q.pose.pose.position.x=path[i].x;q.pose.pose.position.y=path[i].y;
      q.pose.pose.position.z=reference_z_;
      q.pose.pose.orientation.z=std::sin(path[i].yaw/2);q.pose.pose.orientation.w=std::cos(path[i].yaw/2);
      q.velocity=p_.avoidance_speed_mps;result.points.push_back(q);
    }
    return result;
  }
  interfaces::Path straight(const Pose& vehicle,double speed,double end=-1,bool stop=false)const {
    const auto car=project(vehicle.x,vehicle.y);
    if(end<0)end=std::min(car.s+45,reference_.back().s-p_.vehicle_front_m-.5);
    interfaces::Path result;
    if(end<=car.s+.15) {
      interfaces::PathPoint point;point.pose.pose.position.x=vehicle.x;point.pose.pose.position.y=vehicle.y;
      point.pose.pose.position.z=reference_z_;
      point.pose.pose.orientation.z=std::sin(vehicle.yaw/2);point.pose.pose.orientation.w=std::cos(vehicle.yaw/2);
      result.points.push_back(point);point.time_from_start=ros::Duration(5);result.points.push_back(point);return result;
    }
    const int count=std::max(2,int(std::ceil(std::max(.01,end-car.s)/.25))+1);
    for(int i=0;i<count;++i) {
      const double station=car.s+(end-car.s)*i/(count-1);
      const double u=std::max(0.,std::min(1.,(station-car.s)/10.));
      const double lateral=car.l*(1-u*u*u*(10+u*(-15+6*u)));
      const Pose q=at(station,lateral);interfaces::PathPoint point;
      point.pose.pose.position.x=q.x;point.pose.pose.position.y=q.y;
      point.pose.pose.position.z=reference_z_;
      point.pose.pose.orientation.z=std::sin(q.yaw/2);point.pose.pose.orientation.w=std::cos(q.yaw/2);
      point.velocity=speed;
      if(stop) {
        const double a=p_.stopping_deceleration_mps2,r=a*p_.stopping_response_sec;
        point.velocity=std::min(speed,-r+std::sqrt(r*r+2*a*std::max(0.,end-station)));
      }
      result.points.push_back(point);
    }
    return result;
  }
  static interfaces::Path hold(const nav_msgs::Odometry& ego) {
    interfaces::Path output;interfaces::PathPoint point;point.pose.pose=ego.pose.pose;
    point.velocity=0;point.acceleration=0;output.points.push_back(point);
    point.time_from_start=ros::Duration(5);output.points.push_back(point);return output;
  }
};
}  // namespace path_planner
