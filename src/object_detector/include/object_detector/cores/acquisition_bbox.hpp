#pragma once
#include <object_detector/libraries/types.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Objects.h>
#include <tf2_msgs/TFMessage.h>
#include <lanelet2_core/LaneletMap.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>
#include <deque>
namespace object_detector {
struct AcquisitionBboxParameters {
  lanelet::LaneletMapPtr map;
  bool require_road_center_for_completion=false;
  double association_gate_m=3.5,maximum_speed_mps=35.,motion_history_sec=.6;
  std::vector<lanelet::Id> ring_lanelet_ids;
  double vehicle_length_m=4.85,vehicle_width_m=2.1,minimum_height_m=.5,road_match_distance_m=3.;
  double vehicle_height_m=1.65;
  Eigen::Vector3d lidar_position=Eigen::Vector3d(1.5,0,1.403);
};
// Complete the unobserved part of a road-aligned vehicle box away from the
// sensor-facing surfaces. A side view uses its observed longitudinal midpoint.
class AcquisitionBbox {
  struct Segment {Eigen::Vector2d a,b;bool ring;double z0,z1;};
 public:
  explicit AcquisitionBbox(AcquisitionBboxParameters parameters):p_(std::move(parameters)) {
    for(auto lane:p_.map->laneletLayer) {
      lane.resetCache();const auto line=lane.centerline3d();
      Road road;
      for(const auto& q:lane.leftBound3d())road.outline.push_back({q.x(),q.y()});
      const auto right=lane.rightBound3d().basicLineString();
      for(auto q=right.rbegin();q!=right.rend();++q)road.outline.push_back({q->x(),q->y()});
      road.minimum=road.maximum=road.outline.front();
      for(const auto& q:road.outline){road.minimum=road.minimum.cwiseMin(q);road.maximum=road.maximum.cwiseMax(q);}
      roads_.push_back(std::move(road));
      const bool ring=std::find(p_.ring_lanelet_ids.begin(),p_.ring_lanelet_ids.end(),lane.id())!=p_.ring_lanelet_ids.end();
      for(std::size_t i=1;i<line.size();++i)
        segments_.push_back({{line[i-1].x(),line[i-1].y()},{line[i].x(),line[i].y()},ring,line[i-1].z(),line[i].z()});
    }
    for(auto id:p_.ring_lanelet_ids) {
      const auto lane=std::find_if(p_.map->laneletLayer.begin(),p_.map->laneletLayer.end(),
          [id](const lanelet::Lanelet& q){return q.id()==id;});
      for(const auto& q:lane->centerline3d()) {
        const Eigen::Vector2d point(q.x(),q.y());
        if(ring_.empty() || (point-ring_.back()).norm()>1e-6)ring_.push_back(point);
      }
    }
    if(!ring_.empty()) {
      if((ring_.front()-ring_.back()).norm()>1e-6)ring_.push_back(ring_.front());
      station_.push_back(0);
      for(std::size_t i=1;i<ring_.size();++i)station_.push_back(station_.back()+(ring_[i]-ring_[i-1]).norm());
    }
  }
  std::tuple<interfaces::Objects,interfaces::CompetitionVehicleStatus> process(
      const Clusters& clusters,const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& transforms,const ros::Time& stamp) {
    const auto frame=std::find_if(transforms.transforms.begin(),transforms.transforms.end(),[](const auto& t){
      return t.header.frame_id=="map" && t.child_frame_id=="base_link";
    });
    const auto& q=frame->transform.rotation;const auto& t=frame->transform.translation;
    const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    const Eigen::Vector3d observer=rotation*p_.lidar_position+Eigen::Vector3d(t.x,t.y,t.z);
    interfaces::Objects result;
    for(const auto& cloud:clusters)result.objects.push_back(fit(cloud,observer.head<2>()));
    alignObservationTimes(result,clusters,stamp.toSec());
    return std::make_tuple(std::move(result),competition);
  }
 private:
  struct MotionSample {double time;Eigen::Vector2d position,size;};
  struct MotionTrack {std::deque<MotionSample> history;Eigen::Vector2d velocity=Eigen::Vector2d::Zero();bool known=false;};
  static double median(std::vector<double> values) {
    std::sort(values.begin(),values.end());return .5*(values[(values.size()-1)/2]+values[values.size()/2]);
  }
  // One-to-one augmentation uses the same team Hungarian assignment approach.
  std::vector<int> matchMotion(const interfaces::Objects& objects,const std::vector<double>& times) const {
    const std::size_t n=objects.objects.size(),m=motion_tracks_.size()+n;
    std::vector<double> u(n+1),v(m+1);std::vector<std::size_t> owner(m+1),way(m+1);
    std::vector<std::vector<double>> cost(n,std::vector<double>(m,p_.association_gate_m));
    for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<motion_tracks_.size();++j) {
      const auto& track=motion_tracks_[j];const auto& previous=track.history.back();const auto& o=objects.objects[i];
      Eigen::Vector2d velocity=track.velocity;
      if(!track.known && track.history.size()>=2) {
        const auto& a=track.history[track.history.size()-2];const double dt=previous.time-a.time;
        if(dt>.03)velocity=(previous.position-a.position)/dt;
        if(velocity.norm()>p_.maximum_speed_mps)velocity.setZero();
      }
      const Eigen::Vector2d expected=previous.position+velocity*(times[i]-previous.time);
      const double distance=(Eigen::Vector2d(o.position.x,o.position.y)-expected).norm();
      const double shape=.2*(std::abs(o.bbox_size.x-previous.size.x())+std::abs(o.bbox_size.y-previous.size.y()));
      cost[i][j]=distance>p_.association_gate_m?1e9:distance+shape;
    }
    for(std::size_t row=1;row<=n;++row) {
      owner[0]=row;std::size_t column=0;std::vector<double> minimum(m+1,1e12);std::vector<bool> used(m+1,false);
      do {
        used[column]=true;const auto current=owner[column];double delta=1e12;std::size_t next=0;
        for(std::size_t j=1;j<=m;++j)if(!used[j]) {
          const double value=cost[current-1][j-1]-u[current]-v[j];
          if(value<minimum[j]){minimum[j]=value;way[j]=column;}
          if(minimum[j]<delta){delta=minimum[j];next=j;}
        }
        for(std::size_t j=0;j<=m;++j)if(used[j]){u[owner[j]]+=delta;v[j]-=delta;}else minimum[j]-=delta;
        column=next;
      }while(owner[column]);
      do {const auto previous=way[column];owner[column]=owner[previous];column=previous;}while(column);
    }
    std::vector<int> result(n,-1);
    for(std::size_t j=1;j<=motion_tracks_.size();++j)if(owner[j] && cost[owner[j]-1][j-1]<p_.association_gate_m)
      result[owner[j]-1]=j-1;
    return result;
  }
  void fitVelocity(MotionTrack& track) const {
    track.known=false;
    if(track.history.size()<3 || track.history.back().time-track.history.front().time<.15)return;
    std::vector<double> vx,vy;
    for(std::size_t i=0;i<track.history.size();++i)for(std::size_t j=i+1;j<track.history.size();++j) {
      const auto& a=track.history[i];const auto& b=track.history[j];const double dt=b.time-a.time;
      if(dt<.08 || (a.size-b.size).cwiseAbs().maxCoeff()>.75)continue;
      const Eigen::Vector2d velocity=(b.position-a.position)/dt;vx.push_back(velocity.x());vy.push_back(velocity.y());
    }
    if(vx.size()<3)return;
    const double x=median(vx),y=median(vy);
    for(auto& a:vx)a=std::abs(a-x);
    for(auto& a:vy)a=std::abs(a-y);
    if(std::hypot(median(vx),median(vy))>1.5 || std::hypot(x,y)>p_.maximum_speed_mps)return;
    track.velocity={x,y};track.known=true;
  }
  void alignObservationTimes(interfaces::Objects& result,const Clusters& clusters,double reference_time) {
    if(reference_time<last_reference_time_)motion_tracks_.clear();
    last_reference_time_=reference_time;
    motion_tracks_.erase(std::remove_if(motion_tracks_.begin(),motion_tracks_.end(),[&](const MotionTrack& track) {
      return reference_time-track.history.back().time>p_.motion_history_sec+.2;
    }),motion_tracks_.end());
    std::vector<double> times;for(const auto& cloud:clusters)times.push_back(cloud.header.stamp*1e-6);
    const auto matches=matchMotion(result,times);
    for(std::size_t i=0;i<result.objects.size();++i) {
      auto& object=result.objects[i];const int match=matches[i];
      if(match<0)motion_tracks_.push_back(MotionTrack{});
      auto& track=motion_tracks_[match<0?motion_tracks_.size()-1:static_cast<std::size_t>(match)];
      const Eigen::Vector2d position(object.position.x,object.position.y),size(object.bbox_size.x,object.bbox_size.y);
      if(!track.history.empty() && (size-track.history.back().size).cwiseAbs().maxCoeff()>.75)track.history.clear();
      if(track.history.empty() || times[i]>track.history.back().time)track.history.push_back({times[i],position,size});
      while(track.history.size()>12 || (track.history.size()>1 && times[i]-track.history.front().time>p_.motion_history_sec))track.history.pop_front();
      fitVelocity(track);
      if(!track.known || size.x()<3.5 || size.y()<1.4)continue;
      const double age=std::max(-.2,std::min(.2,reference_time-times[i]));
      const auto corrected=position+track.velocity*age;
      object.position.x=object.bbox_pose.position.x=corrected.x();
      object.position.y=object.bbox_pose.position.y=corrected.y();
      object.velocity.x=track.velocity.x();object.velocity.y=track.velocity.y();object.velocity.z=0;
      object.velocity_available=true;
    }
  }
  struct Road {std::vector<Eigen::Vector2d> outline;Eigen::Vector2d minimum,maximum;};
  bool onRoad(const Eigen::Vector2d& point) const {
    for(const auto& road:roads_) {
      if((point.array()<road.minimum.array()).any() || (point.array()>road.maximum.array()).any())continue;
      bool inside=false;
      for(std::size_t i=0,j=road.outline.size()-1;i<road.outline.size();j=i++) {
        const auto& a=road.outline[j];const auto& b=road.outline[i];const Eigen::Vector2d d=b-a;
        const double t=std::max(0.,std::min(1.,(point-a).dot(d)/std::max(1e-12,d.squaredNorm())));
        if((point-a-t*d).squaredNorm()<1e-12)return true;
        if((a.y()>point.y())!=(b.y()>point.y()) &&
            point.x()<a.x()+(point.y()-a.y())*d.x()/d.y())inside=!inside;
      }
      if(inside)return true;
    }
    return false;
  }
  struct Extent {double u0,u1,v0,v1,z0,z1;};
  Extent extent(const Cloud& cloud,double angle) const {
    Extent e{1e30,-1e30,1e30,-1e30,1e30,-1e30};const double c=std::cos(angle),s=std::sin(angle);
    for(const auto& point:cloud) {
      const double u=c*point.x+s*point.y,v=-s*point.x+c*point.y;
      e.u0=std::min(e.u0,u);e.u1=std::max(e.u1,u);e.v0=std::min(e.v0,v);e.v1=std::max(e.v1,v);
      e.z0=std::min(e.z0,double(point.z));e.z1=std::max(e.z1,double(point.z));
    }
    return e;
  }
  Eigen::Vector2d visibleCenter(const Cloud& cloud,const Extent& e,double length,double width,
                               double angle,const Eigen::Vector2d& observer) const {
    const double c=std::cos(angle),s=std::sin(angle);
    const double observer_u=c*observer.x()+s*observer.y(),observer_v=-s*observer.x()+c*observer.y();
    std::vector<Eigen::Vector2d> points;points.reserve(cloud.size());
    for(const auto& point:cloud)points.push_back({c*point.x+s*point.y,-s*point.x+c*point.y});
    const double middle_u=(e.u0+e.u1)/2,middle_v=(e.v0+e.v1)/2;
    Eigen::Vector2d best(middle_u,middle_v);double cost=std::numeric_limits<double>::infinity();
    for(unsigned i=0;i<=8;++i)for(unsigned j=0;j<=8;++j) {
      const double u=e.u1-length/2+(length-e.u1+e.u0)*i/8;
      const double v=e.v1-width/2+(width-e.v1+e.v0)*j/8;
      const bool front=observer_u>u+length/2,rear=observer_u<u-length/2;
      const bool left=observer_v>v+width/2,right=observer_v<v-width/2;
      if(!front && !rear && !left && !right)continue;
      double error=0;
      for(const auto& point:points) {
        double distance=.3;
        if(front || rear)distance=std::min(distance,std::abs(point.x()-u-(front?length/2:-length/2)));
        if(left || right)distance=std::min(distance,std::abs(point.y()-v-(left?width/2:-width/2)));
        error+=distance*distance;
      }
      error/=points.size();error+=1e-4*((u-middle_u)*(u-middle_u)+(v-middle_v)*(v-middle_v));
      if(error<cost){cost=error;best={u,v};}
    }
    return best;
  }
  bool roadHeading(const Eigen::Vector2d& point,double& heading,double& ground_z) const {
    double closest=p_.road_match_distance_m*p_.road_match_distance_m;bool found=false,matched_ring=false;
    for(const auto& line:segments_) {
      if(matched_ring && !line.ring)continue;
      const auto d=(line.b-line.a).eval();const double u=std::max(0.,std::min(1.,(point-line.a).dot(d)/d.squaredNorm()));
      const double distance=(point-line.a-u*d).squaredNorm();
      if((line.ring && !matched_ring && distance<=p_.road_match_distance_m*p_.road_match_distance_m) || distance<=closest) {
        closest=distance;heading=std::atan2(d.y(),d.x());found=true;matched_ring=line.ring;ground_z=line.z0+u*(line.z1-line.z0);
      }
    }
    if(matched_ring) {
      double distance=std::numeric_limits<double>::infinity(),station=0;
      for(std::size_t i=1;i<ring_.size();++i) {
        const Eigen::Vector2d d=ring_[i]-ring_[i-1];
        const double u=std::max(0.,std::min(1.,(point-ring_[i-1]).dot(d)/d.squaredNorm()));
        const double squared=(point-ring_[i-1]-u*d).squaredNorm();
        if(squared<distance){distance=squared;station=station_[i-1]+u*(station_[i]-station_[i-1]);}
      }
      const Eigen::Vector2d d=ringPoint(station+1.5)-ringPoint(station-1.5);
      heading=std::atan2(d.y(),d.x());
    }
    return found;
  }
  Eigen::Vector2d ringPoint(double station) const {
    const double length=station_.back();station-=length*std::floor(station/length);
    const auto after=std::upper_bound(station_.begin(),station_.end(),station);
    const std::size_t i=std::min(ring_.size()-1,std::max<std::size_t>(1,after-station_.begin()));
    return ring_[i-1]+(station-station_[i-1])/(station_[i]-station_[i-1])*(ring_[i]-ring_[i-1]);
  }
  interfaces::Object fit(const Cloud& cloud,const Eigen::Vector2d& observer) const {
    auto e=extent(cloud,0);Eigen::Vector2d center((e.u0+e.u1)/2,(e.v0+e.v1)/2);
    double angle=0,length=e.u1-e.u0,width=e.v1-e.v0,ground_z=e.z0,confidence=0;
    for(unsigned iteration=0;iteration<3;++iteration) {
      const bool road=roadHeading(center,angle,ground_z);e=extent(cloud,angle);
      const double observed_length=e.u1-e.u0,observed_width=e.v1-e.v0;
      // A single horizontal laser ring can see a wide rear/front face with
      // almost zero height. Visible road-aligned width still supports a car.
      const double support=std::max(observed_width/p_.vehicle_width_m,observed_length/p_.vehicle_length_m);
      const bool vehicle=road && (e.z1-e.z0>=p_.minimum_height_m || support>=.45) &&
          observed_length<=8. && observed_width<=3.5;
      const double u_support=std::max(0.,std::min(1.,(support-.20)/.25));
      confidence=vehicle?u_support*u_support*(3-2*u_support):0.;
      length=observed_length+confidence*std::max(0.,p_.vehicle_length_m-observed_length);
      width=observed_width+confidence*std::max(0.,p_.vehicle_width_m-observed_width);
      const double c=std::cos(angle),s=std::sin(angle),u=(e.u0+e.u1)/2,v=(e.v0+e.v1)/2;
      const double view_u=c*observer.x()+s*observer.y()-u,view_v=-s*observer.x()+c*observer.y()-v;
      const double wu=std::min(1.,std::abs(view_u)/std::max(.1,std::abs(view_v)));
      const double wv=std::min(1.,std::abs(view_v)/std::max(.1,std::abs(view_u)));
      const auto visible=visibleCenter(cloud,e,length,width,angle,observer);
      const double completed_u=confidence>.99?visible.x():u-std::copysign((length-observed_length)*wu/2,view_u);
      const double completed_v=confidence>.99?visible.y():v-std::copysign((width-observed_width)*wv/2,view_v);
      center={c*completed_u-s*completed_v,s*completed_u+c*completed_v};
      // A roadside panel may be long enough to resemble a car side. Do not
      // invent an unseen vehicle body whose center is outside every road.
      if(p_.require_road_center_for_completion && confidence>0 && !onRoad(center)) {
        confidence=0;length=observed_length;width=observed_width;
        center={c*u-s*v,s*u+c*v};
      }
    }
    interfaces::Object object;object.label="unknown";
    object.position.x=center.x();object.position.y=center.y();object.position.z=(e.z0+e.z1)/2;
    object.bbox_pose.position=object.position;object.bbox_pose.orientation.z=std::sin(angle/2);object.bbox_pose.orientation.w=std::cos(angle/2);
    object.bbox_size.x=length;object.bbox_size.y=width;object.bbox_size.z=e.z1-e.z0;
    if(confidence>.99) {
      const double bottom=std::min(ground_z,e.z0),top=std::max(bottom+p_.vehicle_height_m,e.z1);
      object.position.z=object.bbox_pose.position.z=(bottom+top)/2;object.bbox_size.z=top-bottom;
    }
    return object;
  }
  std::vector<MotionTrack> motion_tracks_;
  double last_reference_time_=-1;
  AcquisitionBboxParameters p_;std::vector<Segment> segments_;std::vector<Road> roads_;
  std::vector<Eigen::Vector2d> ring_;std::vector<double> station_;
};
}  // namespace object_detector
