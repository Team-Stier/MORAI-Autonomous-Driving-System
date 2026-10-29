#pragma once
#include <path_planner/libraries/types.hpp>
#include <sensor_msgs/Imu.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>
#include <map>
#include <limits>
#include <set>
#include <tuple>
namespace path_planner {
// A track continues across gaps up to max(continuity_gap_sec, continuity_gap_frames x frame interval)
// and fits velocity on samples within max(fit_window_sec, fit_window_frames x frame interval).
// The frame interval is the latest /objects stamp step; frame factors 0 keep the fixed limits.
struct IntegratedPredictionParameters {
  double continuity_gap_sec=.3,continuity_gap_frames=0;
  double fit_window_sec=.5,fit_window_frames=0;
};
class IntegratedPrediction {
  struct Sample {double time,x,y,length,width;};
  struct Track {
    std::deque<Sample> samples;
    double vx=0,vy=0;
    bool motion=false;
  };
 public:
  explicit IntegratedPrediction(IntegratedPredictionParameters parameters={}):parameters_(parameters) {}
  using Result=std::tuple<nav_msgs::Odometry,interfaces::CompetitionVehicleStatus,
      interfaces::Objects,tf2_msgs::TFMessage,sensor_msgs::Imu,Route,PredictedObjects>;
  Result process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& competition,
      const interfaces::Objects& objects,const tf2_msgs::TFMessage& tf,
      const sensor_msgs::Imu& imu,const Route& route) {
    const double now=objects.header.stamp.toSec();
    if(now<last_){history_.clear();frame_interval_=0;}
    else if(now>last_ && last_>0)frame_interval_=now-last_;
    last_=now;
    const double gap=std::max(parameters_.continuity_gap_sec,parameters_.continuity_gap_frames*frame_interval_);
    const double window=std::max(parameters_.fit_window_sec,parameters_.fit_window_frames*frame_interval_);
    for(auto it=history_.begin();it!=history_.end();)
      if(now-it->second.samples.back().time>1.)it=history_.erase(it);else ++it;
    auto current=objects;PredictedObjects future;
    std::set<std::uint32_t> present,associated;
    for(const auto& o:current.objects)present.insert(o.id);
    for(auto& o:current.objects) {
      auto& track=history_[o.id];auto& samples=track.samples;
      bool linked_motion=false;double linked_vx=0,linked_vy=0;
      if(!samples.empty() && now-samples.back().time>gap)track=Track{};
      // LiDAR can replace an ID while the same car remains visible. Carry
      // measured motion only across a unique, nearby continuation of a track
      // absent from this observation. Never merge two currently observed IDs.
      if(!o.velocity_available && (samples.size()<3 || now-samples.front().time<.15)) {
        auto match=history_.end();double best=std::numeric_limits<double>::infinity(),second=best;
        for(auto it=history_.begin();it!=history_.end();++it) {
          const auto& old=it->second;
          if(!old.motion || old.samples.empty() || present.count(it->first) || associated.count(it->first))continue;
          const auto& last=old.samples.back();const double dt=now-last.time;
          if(dt<=0 || dt>gap)continue;
          const double dx=o.position.x-last.x-old.vx*dt,dy=o.position.y-last.y-old.vy*dt;
          const double speed=std::hypot(old.vx,old.vy);
          // Partial car faces can move the OBB centre along its travel axis.
          // Keep the cross-track gate narrow so adjacent lanes cannot match.
          const double distance=speed>.3 ? std::hypot(
              (dx*old.vx+dy*old.vy)/(3.*speed),(dx*old.vy-dy*old.vx)/(.8*speed)) :
              std::hypot(dx,dy);
          if(distance<best){second=best;best=distance;match=it;}else second=std::min(second,distance);
        }
        if(match!=history_.end() && best<1.0 && second>best+.5) {
          samples=match->second.samples;associated.insert(match->first);
          linked_motion=true;linked_vx=match->second.vx;linked_vy=match->second.vy;
          match->second.motion=false;
        }
      }
      if(samples.empty() || now>samples.back().time)samples.push_back({now,o.position.x,o.position.y,o.bbox_size.x,o.bbox_size.y});
      while(samples.size()>1 && now-samples.front().time>window)samples.pop_front();
      if(samples.size()>=3 && samples.back().time-samples.front().time>.15) {
        std::vector<double> vx,vy;
        for(std::size_t i=0;i<samples.size();++i)for(std::size_t j=i+1;j<samples.size();++j) {
          const double dt=samples[j].time-samples[i].time;
          if(dt<.09 || std::abs(samples[j].length-samples[i].length)>.5 ||
              std::abs(samples[j].width-samples[i].width)>.5)continue;
          vx.push_back((samples[j].x-samples[i].x)/dt);vy.push_back((samples[j].y-samples[i].y)/dt);
        }
        if(vx.size()>=2) {
          const auto median=[](std::vector<double> v){std::sort(v.begin(),v.end());return (v[(v.size()-1)/2]+v[v.size()/2])/2;};
          const double x=median(vx),y=median(vy),speed=std::hypot(x,y);
          bool use_fit=!o.velocity_available;
          if(!use_fit && speed>3 && (o.velocity.x*x+o.velocity.y*y)/speed<speed-3) {
            for(auto& v:vx)v=std::abs(v-x);
            for(auto& v:vy)v=std::abs(v-y);
            use_fit=median(vx)<=1.5 && median(vy)<=1.5;
          }
          if(use_fit) {o.velocity.x=x;o.velocity.y=y;o.velocity_available=true;}
        }
      }
      if(!o.velocity_available && linked_motion) {
        o.velocity.x=linked_vx;o.velocity.y=linked_vy;o.velocity_available=true;
      }
      track.motion=o.velocity_available;
      if(track.motion){track.vx=o.velocity.x;track.vy=o.velocity.y;}
    }
    for(std::size_t i=0;i<10;++i) {
      future[i]=current;const double dt=.5*(i+1);
      for(auto& o:future[i].objects)if(o.velocity_available) {
        o.position.x+=o.velocity.x*dt;o.position.y+=o.velocity.y*dt;
        o.bbox_pose.position=o.position;
      }
    }
    return std::make_tuple(ego,competition,objects,tf,imu,route,future);
  }
 private:
  IntegratedPredictionParameters parameters_;
  std::map<std::uint32_t,Track> history_;
  double last_=0,frame_interval_=0;
};
}
