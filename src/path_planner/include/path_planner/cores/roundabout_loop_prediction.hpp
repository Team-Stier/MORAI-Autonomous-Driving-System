#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <limits>
#include <tuple>
#include <string>
#include <vector>

namespace path_planner {
struct RoundaboutLoopPredictionParameters {
  std::vector<lanelet::Id> ring_lanelet_ids;
  double history_sec = .8;
  double minimum_history_sec = .3;
  double minimum_partial_history_sec = .6;
  double pair_baseline_sec = .2;
  double maximum_observation_gap_sec = .28;
  double track_expiry_sec = 1;
  double ring_match_distance_m = 3;
  double tangent_window_m = 1.5;
  double maximum_speed_mps = 25;
  double maximum_lateral_speed_mps = 1.5;
  double maximum_longitudinal_acceleration_mps2 = 1.5;
  double acceleration_horizon_sec = 1.0;
};

// Position history estimates progress on the closed circulating lane. OBB yaw
// remains a geometric observation and is not interpreted as a driving heading.
class RoundaboutLoopPrediction {
  using Point = Eigen::Vector3d;
  struct Projection { double station, lateral, along, distance, height; };
  struct Observation { double time, station, lateral, distance; Point point; double length, width; };
  struct Track { std::deque<Observation> observations; bool on_ring=false; double station=0,time=0,speed=0; };
  struct Motion { bool known=false, loop=false; double speed=0,acceleration=0; Point velocity=Point::Zero(); };
 public:
  using Result = std::tuple<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
      interfaces::Objects, tf2_msgs::TFMessage, sensor_msgs::Imu, Route, PredictedObjects>;
  RoundaboutLoopPrediction(RoundaboutLoopPredictionParameters parameters, SharedMap& shared)
      : p_(std::move(parameters)) {
    for (auto id : p_.ring_lanelet_ids) {
      const auto lane = std::find_if(shared.map->laneletLayer.begin(), shared.map->laneletLayer.end(),
          [id](const lanelet::Lanelet& q) { return q.id()==id; });
      for (const auto& q : lane->centerline3d()) {
        const Point point(q.x(),q.y(),q.z());
        if (ring_.empty() || (point-ring_.back()).head<2>().norm()>1e-6) ring_.push_back(point);
      }
    }
    if ((ring_.front()-ring_.back()).head<2>().norm()>1e-6) ring_.push_back(ring_.front());
    station_.push_back(0);
    for (std::size_t i=1;i<ring_.size();++i)
      station_.push_back(station_.back()+(ring_[i]-ring_[i-1]).head<2>().norm());
    length_=station_.back();
  }

  Result process(const nav_msgs::Odometry& ego, const interfaces::CompetitionVehicleStatus& competition,
                 const interfaces::Objects& objects, const tf2_msgs::TFMessage& tf,
                 const sensor_msgs::Imu& imu, const Route& route) {
    const double now=objects.header.stamp.toSec();
    if (now<last_time_) tracks_.clear();
    last_time_=now;
    for (auto it=tracks_.begin();it!=tracks_.end();)
      if (now-it->second.observations.back().time>p_.track_expiry_sec) it=tracks_.erase(it); else ++it;
    PredictedObjects predictions;
    predictions.fill(objects);
    auto marker=visualization(objects);
    for (std::size_t index=0;index<objects.objects.size();++index) {
      const auto& object=objects.objects[index];
      const Point origin=point(object.bbox_pose.position);
      auto projection=project(origin);
      const double yaw=tangent(projection.station);
      const Eigen::Quaterniond original(object.bbox_pose.orientation.w,object.bbox_pose.orientation.x,
                                        object.bbox_pose.orientation.y,object.bbox_pose.orientation.z);
      const Eigen::Matrix3d axes=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const Point enclosing=(axes.transpose()*original.toRotationMatrix()).cwiseAbs()*point(object.bbox_size);
      if(tracks_.find(object.id)==tracks_.end() && object.bbox_size.x>=3.5 && object.bbox_size.y>=1.4) {
        auto previous=tracks_.end();double closest=p_.ring_match_distance_m;
        for(auto it=tracks_.begin();it!=tracks_.end();++it) {
          const auto& old=it->second;
          if(!old.on_ring || now-old.observations.back().time>p_.maximum_observation_gap_sec ||
              std::any_of(objects.objects.begin(),objects.objects.end(),
                  [&](const interfaces::Object& q){return q.id==it->first;}))continue;
          const double distance=(origin-sample(old.station+old.speed*(now-old.time))).head<2>().norm();
          if(distance<closest){closest=distance;previous=it;}
        }
        if(previous!=tracks_.end()) {
          tracks_[object.id]=std::move(previous->second);tracks_.erase(previous);
        }
      }
      auto& track=tracks_[object.id];auto& history=track.observations;
      if (!history.empty() && now-history.back().time>p_.maximum_observation_gap_sec) {history.clear();}
      if (history.empty() || now>history.back().time) {
        double station=projection.station;
        if (!history.empty()) station=history.back().station+signedWrap(station-wrap(history.back().station));
        history.push_back({now,station,projection.lateral,projection.distance,origin,enclosing.x(),enclosing.y()});
      }
      while (history.size()>1 && history.front().time<now-p_.history_sec) history.pop_front();
      auto motion=estimate(object,projection,history);
      const bool was_on_ring=track.on_ring;
      track.on_ring=track.on_ring || (motion.loop && object.bbox_size.x>=3.5 && object.bbox_size.y>=1.4);
      if(track.on_ring) {
        motion.loop=true;motion.speed=motion.known?std::max(0.,motion.speed):0;
        double station=history.back().station;
        if(was_on_ring && (now==track.time || !motion.known))station=track.station;
        else if(was_on_ring && now>track.time && motion.speed>.1) {
          const double expected=track.station+motion.speed*(now-track.time);
          const double innovation=signedWrap(station-wrap(expected));
          station=expected+std::max(-1.,std::min(1.,innovation))*.5;
        }
        projection.station=wrap(station);projection.lateral=projection.along=0;
        track.station=station;track.time=now;track.speed=motion.speed;
      }
      std::vector<Point> future_points{origin};
      for (std::size_t k=0;k<predictions.size();++k) {
        const double time=.5*(k+1);
        auto& future=predictions[k].objects[index];
        Point predicted=origin, velocity=Point::Zero(), acceleration=Point::Zero();
        if (motion.loop) {
          double acceleration_end=p_.acceleration_horizon_sec;
          if(motion.acceleration<0)acceleration_end=std::min(acceleration_end,motion.speed/-motion.acceleration);
          if(motion.acceleration>0)acceleration_end=std::min(acceleration_end,(p_.maximum_speed_mps-motion.speed)/motion.acceleration);
          const double duration=std::min(time,acceleration_end);
          const double travel=motion.speed*time+motion.acceleration*duration*(time-duration/2);
          const double speed=std::max(0.,motion.speed+motion.acceleration*duration);
          const double s=projection.station+travel;
          predicted=offsetPoint(s,projection);
          const Point direction=(offsetPoint(s+.25,projection)-offsetPoint(s-.25,projection))/.5;
          velocity=speed*direction;
          acceleration=speed*speed*(offsetPoint(s+.25,projection)-2*predicted+
                                                offsetPoint(s-.25,projection))/.0625;
          if(time<acceleration_end && speed>0)acceleration+=motion.acceleration*direction;
        } else if (motion.known) { velocity=motion.velocity; predicted+=time*velocity; }
        if (motion.loop || projection.distance<=p_.ring_match_distance_m) {
          const double heading=tangent(project(predicted).station);
          future.bbox_pose.orientation.x=future.bbox_pose.orientation.y=0;
          future.bbox_pose.orientation.z=std::sin(heading/2);
          future.bbox_pose.orientation.w=std::cos(heading/2);
          set(future.bbox_size,enclosing);
        }
        set(future.position,predicted); future.bbox_pose.position=future.position;
        set(future.velocity,velocity);set(future.acceleration,acceleration);
        future.velocity_available=motion.known;future.acceleration_available=motion.known;
        future_points.push_back(predicted);
      }
      const auto color=motion.known?(motion.velocity.head<2>().norm()<.3 && std::abs(motion.speed)<.3?
          rgba(.15,.5,1):rgba(1,.2,.15)):rgba(1,.8,.1);
      box(marker,object,color);
      const auto preview_color=motion.loop?rgba(.1,.95,1):motion.known?rgba(.8,.45,1):rgba(1,.8,.1);
      for (std::size_t k=1;k<future_points.size();++k) {
        line(marker,future_points[k-1],future_points[k],preview_color);
        const Point& q=future_points[k];
        line(marker,q+Point(.12,0,0),q-Point(.12,0,0),preview_color);
        line(marker,q+Point(0,.12,0),q-Point(0,.12,0),preview_color);
      }
      if (motion.loop) {
        const Point forward(std::cos(yaw),std::sin(yaw),0),left(-std::sin(yaw),std::cos(yaw),0);
        const Point end=origin+2*forward;
        line(marker,origin,end,preview_color);
        line(marker,end,end-.4*forward+.2*left,preview_color);
        line(marker,end,end-.4*forward-.2*left,preview_color);
        box(marker,predictions[1].objects[index],preview_color);
        visualization_msgs::Marker label;label.header=objects.header;
        label.ns="roundabout_prediction_time";label.id=object.id;
        label.type=visualization_msgs::Marker::TEXT_VIEW_FACING;label.action=visualization_msgs::Marker::ADD;
        label.pose=predictions[1].objects[index].bbox_pose;label.pose.position.z+=1.8;
        label.scale.z=.6;label.color=preview_color;label.lifetime=ros::Duration(.5);
        label.text="ID "+std::to_string(object.id)+" +1s";view_.publish(label);
      }
    }
    view_.publish(marker);
    for (std::size_t k=0;k<predictions.size();++k) prediction_views_[k].publish(predictions[k]);
    return std::make_tuple(ego,competition,objects,tf,imu,route,predictions);
  }
 private:
  static Point point(const geometry_msgs::Point& p) { return {p.x,p.y,p.z}; }
  static Point point(const geometry_msgs::Vector3& p) { return {p.x,p.y,p.z}; }
  template<class Message> static void set(Message& message,const Point& p) {message.x=p.x();message.y=p.y();message.z=p.z();}
  double wrap(double station) const { return station-length_*std::floor(station/length_); }
  double signedWrap(double distance) const { return wrap(distance+length_/2)-length_/2; }
  Point sample(double station) const {
    station=wrap(station);
    const auto upper=std::upper_bound(station_.begin(),station_.end(),station);
    const std::size_t i=std::min(ring_.size()-1,std::max<std::size_t>(1,upper-station_.begin()));
    const double ratio=(station-station_[i-1])/(station_[i]-station_[i-1]);
    return ring_[i-1]+ratio*(ring_[i]-ring_[i-1]);
  }
  double tangent(double s) const {
    const Point delta=sample(s+p_.tangent_window_m)-sample(s-p_.tangent_window_m);
    return std::atan2(delta.y(),delta.x());
  }
  Point offsetPoint(double s,const Projection& projection) const {
    const double yaw=tangent(s);
    return sample(s)+Point(projection.along*std::cos(yaw)-projection.lateral*std::sin(yaw),
                            projection.along*std::sin(yaw)+projection.lateral*std::cos(yaw),projection.height);
  }
  Projection project(const Point& p) const {
    double minimum=std::numeric_limits<double>::infinity(),chosen=0;
    Point foot=ring_.front();
    for (std::size_t i=1;i<ring_.size();++i) {
      const auto d=(ring_[i]-ring_[i-1]).head<2>().eval();
      const double u=std::max(0.,std::min(1.,(p-ring_[i-1]).head<2>().dot(d)/d.squaredNorm()));
      const Point q=ring_[i-1]+u*(ring_[i]-ring_[i-1]);
      const double distance=(p-q).head<2>().squaredNorm();
      if (distance<minimum) {minimum=distance;chosen=station_[i-1]+u*(station_[i]-station_[i-1]);foot=q;}
    }
    const double yaw=tangent(chosen);const Point delta=p-foot;
    return {chosen,-delta.x()*std::sin(yaw)+delta.y()*std::cos(yaw),
            delta.x()*std::cos(yaw)+delta.y()*std::sin(yaw),std::sqrt(minimum),delta.z()};
  }
  static double median(std::vector<double> values) {
    const auto middle=values.begin()+values.size()/2;std::nth_element(values.begin(),middle,values.end());
    return values.size()%2?*middle:(*middle+*std::max_element(values.begin(),middle))/2;
  }
  Motion estimate(const interfaces::Object& object,const Projection& current,
                  const std::deque<Observation>& history) const {
    Motion motion;double lateral_speed=0;
    // The tracker fits a longer motion history. Refit short progress history
    // only until that map velocity is available.
    const double history_span=object.bbox_size.x>=3.5 && object.bbox_size.y>=1.4?
        p_.minimum_history_sec:p_.minimum_partial_history_sec;
    const bool enough=!object.velocity_available && history.size()>=3 && history.back().time-history.front().time>=history_span;
    if (enough) {
      std::vector<double> along,lateral,x,y,z;
      for (std::size_t i=0;i<history.size();++i) for (std::size_t j=i+1;j<history.size();++j) {
        const double dt=history[j].time-history[i].time;
        if (dt<p_.pair_baseline_sec) continue;
        // A newly visible side can move the OBB center without moving the car.
        // Fit only comparable visible extents; do not infer reverse/crossing motion from that jump.
        if (current.distance<=p_.ring_match_distance_m &&
            (std::abs(history[j].length-history[i].length)>.5 ||
             std::abs(history[j].width-history[i].width)>.5)) continue;
        const Point velocity=(history[j].point-history[i].point)/dt;
        x.push_back(velocity.x());y.push_back(velocity.y());z.push_back(velocity.z());
        if (history[i].distance<=p_.ring_match_distance_m && history[j].distance<=p_.ring_match_distance_m) {
          along.push_back((history[j].station-history[i].station)/dt);
          lateral.push_back((history[j].lateral-history[i].lateral)/dt);
        }
      }
      if (!x.empty()) {
        motion.known=true;motion.velocity=Point(median(x),median(y),median(z));
        if (!along.empty()) {motion.speed=median(along);lateral_speed=median(lateral);}
        else lateral_speed=p_.maximum_lateral_speed_mps+1;
      }
    } else if (object.velocity_available) {
      motion.known=true;motion.velocity=point(object.velocity);
      const double yaw=tangent(current.station);
      motion.speed=motion.velocity.x()*std::cos(yaw)+motion.velocity.y()*std::sin(yaw);
      lateral_speed=-motion.velocity.x()*std::sin(yaw)+motion.velocity.y()*std::cos(yaw);
    }
    motion.speed=std::max(-p_.maximum_speed_mps,std::min(p_.maximum_speed_mps,motion.speed));
    const double speed=motion.velocity.head<2>().norm();
    if (speed>p_.maximum_speed_mps) motion.velocity*=p_.maximum_speed_mps/speed;
    if(!object.velocity_available && current.distance<=p_.ring_match_distance_m &&
        object.bbox_size.x>=3.5 && object.bbox_size.y>=1.4 && motion.speed<-.5) {
      motion.known=false;motion.velocity.setZero();motion.speed=0;
    }
    // Circulating traffic stays on the mapped road even when its visible OBB
    // jitters laterally. Pure radial crossing motion still uses the linear model.
    motion.loop=motion.known && current.distance<=p_.ring_match_distance_m &&
                (motion.speed>.5 || std::abs(lateral_speed)<=p_.maximum_lateral_speed_mps);
    if (motion.loop) motion.speed=std::max(0.,motion.speed);
    if(motion.loop && object.acceleration_available) {
      const double heading=tangent(current.station);
      motion.acceleration=std::max(-p_.maximum_longitudinal_acceleration_mps2,
          std::min(p_.maximum_longitudinal_acceleration_mps2,
              object.acceleration.x*std::cos(heading)+object.acceleration.y*std::sin(heading)));
      if(history.size()>=6 && history.back().time-history.front().time>=.6) {
        const double middle=(history.front().time+history.back().time)/2;
        std::vector<double> early,late,early_times,late_times;
        for(std::size_t i=0;i<history.size();++i)for(std::size_t j=i+1;j<history.size();++j) {
          const double dt=history[j].time-history[i].time;
          if(dt<p_.pair_baseline_sec || history[i].distance>p_.ring_match_distance_m ||
              history[j].distance>p_.ring_match_distance_m ||
              std::abs(history[j].length-history[i].length)>.5 ||
              std::abs(history[j].width-history[i].width)>.5)continue;
          const double time=(history[i].time+history[j].time)/2;
          auto& speeds=time<middle?early:late;auto& times=time<middle?early_times:late_times;
          speeds.push_back((history[j].station-history[i].station)/dt);times.push_back(time);
        }
        if(early.size()>=3 && late.size()>=3) {
          const double trend=(median(late)-median(early))/(median(late_times)-median(early_times));
          motion.acceleration=motion.acceleration*trend>0?
              std::copysign(std::min(std::abs(motion.acceleration),std::abs(trend)),motion.acceleration):0;
        }
      }
    }
    return motion;
  }
  static std_msgs::ColorRGBA rgba(double r,double g,double b,double a=1) {
    std_msgs::ColorRGBA c;c.r=r;c.g=g;c.b=b;c.a=a;return c;
  }
  static void line(visualization_msgs::Marker& marker,const Point& a,const Point& b,const std_msgs::ColorRGBA& color) {
    geometry_msgs::Point q;set(q,a);marker.points.push_back(q);set(q,b);marker.points.push_back(q);
    marker.colors.push_back(color);marker.colors.push_back(color);
  }
  static void box(visualization_msgs::Marker& marker,const interfaces::Object& object,const std_msgs::ColorRGBA& color) {
    const auto& q=object.bbox_pose.orientation;
    const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);const Point size=point(object.bbox_size),origin=point(object.bbox_pose.position);
    std::array<Point,8> corners;
    for (unsigned i=0;i<8;++i) corners[i]=origin+rotation*Point((i&1?.5:-.5)*size.x(),(i&2?.5:-.5)*size.y(),(i&4?.5:-.5)*size.z());
    for (unsigned i=0;i<8;++i) for (unsigned axis=0;axis<3;++axis) {
      const unsigned other=i^(1u<<axis);if (other>i) line(marker,corners[i],corners[other],color);
    }
  }
  visualization_msgs::Marker visualization(const interfaces::Objects& objects) const {
    visualization_msgs::Marker marker;marker.header=objects.header;marker.ns="roundabout_loop_prediction";marker.id=0;
    marker.type=visualization_msgs::Marker::LINE_LIST;marker.action=visualization_msgs::Marker::ADD;
    marker.pose.orientation.w=1;marker.scale.x=.07;marker.color.a=1;marker.lifetime=ros::Duration(.5);
    for (std::size_t i=1;i<ring_.size();++i) line(marker,ring_[i-1]+Point(0,0,.1),ring_[i]+Point(0,0,.1),rgba(.15,.45,.8,.5));
    return marker;
  }
  RoundaboutLoopPredictionParameters p_;
  std::vector<Point> ring_;
  std::vector<double> station_;
  double length_=0,last_time_=0;
  std::map<std::uint32_t,Track> tracks_;
  node_pipeline::Visualizer<visualization_msgs::Marker> view_{"object_motion"};
  using View=node_pipeline::Visualizer<interfaces::Objects>;
  std::array<View,10> prediction_views_{{View("predicted_objects_0_5s"),View("predicted_objects_1_0s"),
      View("predicted_objects_1_5s"),View("predicted_objects_2_0s"),View("predicted_objects_2_5s"),
      View("predicted_objects_3_0s"),View("predicted_objects_3_5s"),View("predicted_objects_4_0s"),
      View("predicted_objects_4_5s"),View("predicted_objects_5_0s")}};
};
}  // namespace path_planner
