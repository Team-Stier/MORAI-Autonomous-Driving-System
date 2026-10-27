#pragma once
#include <interfaces/Objects.h>
#include <cstdint>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <deque>
#include <limits>

namespace object_detector {
struct TrackingParameters {
  bool use_observation_velocity = false;
  double association_gate_m = 2.5;
  double shape_weight = 0.2;
  double max_age_sec = 0.6;
  double moving_vehicle_coast_sec = 0;
  int history_size = 21;
  int min_velocity_observations = 8;
  double minimum_velocity_span_sec = 1.0;
  double pair_baseline_sec = 0.3;
  double history_window_sec = 2.0;
  int min_acceleration_observations = 8;
  std::vector<Eigen::Vector3d> ring_centerline;
  double ring_match_distance_m = 3.0;
};
class Tracking {
 public:
  explicit Tracking(TrackingParameters parameters = {}) : parameters_(parameters) {}
  interfaces::Objects process(const interfaces::Objects& input,
                              const ros::Time& stamp) {
    const double time = stamp.toSec();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
      return time - track.history.back().time > parameters_.max_age_sec;
    }), tracks_.end());

    auto objects=input;
    for(auto& object:objects.objects)alignToRing(object);
    const auto observation_cost=[&](const interfaces::Object& object) {
      double best=std::numeric_limits<double>::infinity();
      for(const auto& track:tracks_) {
        const auto expected=parameters_.moving_vehicle_coast_sec>0 && track.velocity_available?
            vector(projected(track,time).position):track.history.back().position;
        best=std::min(best,(vector(object.position)-expected).squaredNorm());
      }
      return best;
    };
    // Separate visible surfaces can each complete into the same full car.
    // Keep the observation continuing a track, rather than creating two IDs.
    for(std::size_t i=0;i<objects.objects.size();++i)
      for(std::size_t j=i+1;j<objects.objects.size();) {
        if(sameBody(objects.objects[i],objects.objects[j]) || sameBody(objects.objects[j],objects.objects[i])) {
          if(observation_cost(objects.objects[j])<observation_cost(objects.objects[i]))
            objects.objects[i]=objects.objects[j];
          objects.objects.erase(objects.objects.begin()+j);
        } else ++j;
      }
    std::vector<Eigen::Vector3d> positions;
    for (const auto& object : objects.objects)
      positions.push_back(vector(object.position));
    const auto matches = associateTeamHungarian(objects, positions, time);
    interfaces::Objects output;
    output.objects = objects.objects;
    for (std::size_t i = 0; i < output.objects.size(); ++i) {
      const std::size_t index = matches[i];
      if (index == noTrack()) {
        Track track;
        track.id = next_id_++;
        tracks_.push_back(std::move(track));
      }
      auto& track = tracks_[index == noTrack() ? tracks_.size() - 1 : index];
      if(parameters_.moving_vehicle_coast_sec>0 && track.velocity_available &&
          track.size.x()>3.5 && track.size.y()>1.4 &&
          output.objects[i].bbox_size.x*output.objects[i].bbox_size.y<.35*track.size.x()*track.size.y() &&
          time-track.history.back().time<=parameters_.moving_vehicle_coast_sec) {
        output.objects[i]=projected(track,time);continue;
      }
      track.label = output.objects[i].label;
      track.size = vector(output.objects[i].bbox_size);
      if (!track.history.empty() && track.history.back().time == time)
        track.history.back().position = positions[i];
      else track.history.push_back({time, positions[i]});
      while (track.history.size() > static_cast<std::size_t>(parameters_.history_size) ||
             (track.history.size() > 1 && time - track.history.front().time > parameters_.history_window_sec))
        track.history.pop_front();
      fitMotion(track);
      if(parameters_.use_observation_velocity && output.objects[i].velocity_available) {
        // AcquisitionBbox estimates motion before propagating the observation.
        // Its velocity is not contaminated by the onset of that position correction.
        track.velocity=vector(output.objects[i].velocity);track.velocity_available=true;
        track.acceleration.setZero();track.acceleration_available=false;
      }
      auto& object = output.objects[i];
      object.id = track.id;
      object.velocity = message(track.velocity);
      object.acceleration = message(track.acceleration);
      object.velocity_available = track.velocity_available;
      object.acceleration_available = track.acceleration_available;
      track.last_object=object;
    }
    for(const auto& track:tracks_) {
      const double age=time-track.history.back().time;
      if(age<=0 || age>parameters_.moving_vehicle_coast_sec || !track.velocity_available ||
          track.velocity.head<2>().norm()<.8 || track.size.x()<3.5 || track.size.y()<1.4)continue;
      const auto present=std::find_if(output.objects.begin(),output.objects.end(),[&](const interfaces::Object& q){return q.id==track.id;});
      if(present==output.objects.end()) {
        const auto coast=projected(track,time);
        // A reacquired body may get a new ID after its visible centre jumps.
        // Do not retain the extrapolated old body on top of a live observation.
        const bool observed=std::any_of(output.objects.begin(),output.objects.end(),
            [&](const interfaces::Object& q){return sameBody(coast,q);});
        if(!observed)output.objects.push_back(coast);
      }
    }
    // A thin fragment can become a retained full body only after association.
    // Collapse that final overlap as well, preferring a measured motion track.
    for(std::size_t i=0;i<output.objects.size();++i)
      for(std::size_t j=i+1;j<output.objects.size();) {
        auto& a=output.objects[i];const auto& b=output.objects[j];
        if(sameBody(a,b) || sameBody(b,a)) {
          if((b.velocity_available && !a.velocity_available) ||
             (b.velocity_available==a.velocity_available && b.id<a.id))a=b;
          output.objects.erase(output.objects.begin()+j);
        } else ++j;
      }
    return output;
  }
 private:
  static bool sameBody(const interfaces::Object& a,const interfaces::Object& b) {
    if(b.bbox_size.x<3.5 || b.bbox_size.y<1.4)return false;
    const auto& q=a.bbox_pose.orientation;
    const double heading=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const auto& r=b.bbox_pose.orientation;
    const double other=std::atan2(2*(r.w*r.z+r.x*r.y),1-2*(r.y*r.y+r.z*r.z));
    if(a.bbox_size.x*a.bbox_size.y<.35*b.bbox_size.x*b.bbox_size.y) {
      const double dx=a.position.x-b.position.x,dy=a.position.y-b.position.y;
      const double c=std::abs(std::cos(heading-other)),s=std::abs(std::sin(heading-other));
      const double length=(c*a.bbox_size.x+s*a.bbox_size.y)/2;
      const double width=(s*a.bbox_size.x+c*a.bbox_size.y)/2;
      return std::abs(dx*std::cos(other)+dy*std::sin(other))+length<=b.bbox_size.x/2+.3 &&
          std::abs(-dx*std::sin(other)+dy*std::cos(other))+width<=b.bbox_size.y/2+.3;
    }
    if(std::cos(other-heading)<.85)return false;
    const double dx=b.position.x-a.position.x,dy=b.position.y-a.position.y;
    return std::abs(dx*std::cos(heading)+dy*std::sin(heading))<.45*(a.bbox_size.x+b.bbox_size.x) &&
           std::abs(-dx*std::sin(heading)+dy*std::cos(heading))<.25*(a.bbox_size.y+b.bbox_size.y);
  }
  void alignToRing(interfaces::Object& object) const {
    double station=0,heading=0,lateral=0;
    if(!ringProjection(vector(object.position),station,heading,lateral))return;
    const auto& q = object.bbox_pose.orientation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const auto axes = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d enclosing = (axes.transpose()*rotation.toRotationMatrix()).cwiseAbs()*vector(object.bbox_size);
    object.bbox_size = message(enclosing);
    object.bbox_pose.orientation.x = object.bbox_pose.orientation.y = 0;
    object.bbox_pose.orientation.z = std::sin(heading/2);
    object.bbox_pose.orientation.w = std::cos(heading/2);
  }
  struct Observation { double time; Eigen::Vector3d position; };
  struct Track {
    std::uint32_t id = 0;
    std::string label;
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    std::deque<Observation> history;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    bool velocity_available = false, acceleration_available = false;
    double velocity_time = 0;
    interfaces::Object last_object;
  };
  interfaces::Object projected(const Track& track,double time) const {
    auto object=track.last_object;const auto size=object.bbox_size;
    const double age=std::max(0.,time-track.history.back().time);
    double station=0,heading=0,lateral=0;
    if(track.size.x()>=3.5 && track.size.y()>=1.4 &&
        ringProjection(vector(object.position),station,heading,lateral) &&
        std::abs(-object.velocity.x*std::sin(heading)+object.velocity.y*std::cos(heading))<=1.5) {
      const double speed=std::max(0.,object.velocity.x*std::cos(heading)+object.velocity.y*std::sin(heading));
      const auto position=ringPoint(station+speed*age,heading);
      heading=ringHeading(station+speed*age);
      object.position.x=position.x()-lateral*std::sin(heading);
      object.position.y=position.y()+lateral*std::cos(heading);
      object.position.z+=age*object.velocity.z;object.bbox_pose.position=object.position;
      object.bbox_pose.orientation.x=object.bbox_pose.orientation.y=0;
      object.bbox_pose.orientation.z=std::sin(heading/2);object.bbox_pose.orientation.w=std::cos(heading/2);
      object.velocity.x=speed*std::cos(heading);object.velocity.y=speed*std::sin(heading);
      object.acceleration_available=false;object.acceleration={};return object;
    }
    for(double elapsed=0;elapsed<age-1e-9;) {
      const auto before=object.bbox_pose.orientation;
      alignToRing(object);object.bbox_size=size;
      const auto after=object.bbox_pose.orientation;
      const double turn=2*std::atan2(after.z,after.w)-2*std::atan2(before.z,before.w);
      const double vx=object.velocity.x,vy=object.velocity.y;
      object.velocity.x=std::cos(turn)*vx-std::sin(turn)*vy;
      object.velocity.y=std::sin(turn)*vx+std::cos(turn)*vy;
      const double h=std::min(.05,age-elapsed);
      object.position.x+=h*object.velocity.x;object.position.y+=h*object.velocity.y;
      object.position.z+=h*object.velocity.z;object.bbox_pose.position=object.position;elapsed+=h;
    }
    object.acceleration_available=false;object.acceleration={};return object;
  }
  bool ringProjection(const Eigen::Vector3d& point,double& station,double& heading,double& lateral) const {
    double closest=parameters_.ring_match_distance_m*parameters_.ring_match_distance_m,offset=0;
    bool matched=false;
    const auto& ring=parameters_.ring_centerline;
    for(std::size_t i=1;i<ring.size();++i) {
      const auto d=(ring[i]-ring[i-1]).head<2>().eval();const double length=d.norm();
      const double u=std::max(0.,std::min(1.,(point-ring[i-1]).head<2>().dot(d)/d.squaredNorm()));
      const Eigen::Vector2d delta=(point-ring[i-1]).head<2>()-u*d;
      if(delta.squaredNorm()<=closest) {
        closest=delta.squaredNorm();station=offset+u*length;heading=std::atan2(d.y(),d.x());
        lateral=(-delta.x()*d.y()+delta.y()*d.x())/length;matched=true;
      }
      offset+=length;
    }
    if(matched) {
      const auto foot=ringPoint(station,heading);heading=ringHeading(station);
      lateral=-(point.x()-foot.x())*std::sin(heading)+(point.y()-foot.y())*std::cos(heading);
    }
    return matched;
  }
  double ringHeading(double station) const {
    double before_heading=0,after_heading=0;
    const Eigen::Vector2d delta=ringPoint(station+1.5,after_heading)-ringPoint(station-1.5,before_heading);
    return std::atan2(delta.y(),delta.x());
  }
  Eigen::Vector2d ringPoint(double station,double& heading) const {
    const auto& ring=parameters_.ring_centerline;
    double length=0;
    for(std::size_t i=1;i<ring.size();++i)length+=(ring[i]-ring[i-1]).head<2>().norm();
    station-=length*std::floor(station/length);
    for(std::size_t i=1;i<ring.size();++i) {
      const auto d=(ring[i]-ring[i-1]).head<2>().eval();const double span=d.norm();
      if(station<=span || i+1==ring.size()) {
        heading=std::atan2(d.y(),d.x());return ring[i-1].head<2>()+station/span*d;
      }
      station-=span;
    }
    return ring.front().head<2>();
  }
  template<class Vector> static Eigen::Vector3d vector(const Vector& value) {
    return {value.x, value.y, value.z};
  }
  static geometry_msgs::Vector3 message(const Eigen::Vector3d& value) {
    geometry_msgs::Vector3 result;
    result.x = value.x(); result.y = value.y(); result.z = value.z();
    return result;
  }
  static double median(std::vector<double> values) {
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    const double upper = *middle;
    if (values.size() % 2) return upper;
    return (upper + *std::max_element(values.begin(), middle)) / 2;
  }
  void fitMotion(Track& track) const {
    const auto count = track.history.size();
    const bool previous_velocity_available = track.velocity_available;
    const Eigen::Vector3d previous_velocity = track.velocity;
    const double previous_velocity_time = track.velocity_time;
    const double now = track.history.back().time;
    const double span = now - track.history.front().time;
    track.velocity_available = count >= static_cast<std::size_t>(parameters_.min_velocity_observations) &&
        span >= parameters_.minimum_velocity_span_sec;
    track.acceleration_available = false;
    track.acceleration.setZero();
    if (!track.velocity_available) { track.velocity.setZero(); return; }
    std::vector<double> slopes[3];
    for (std::size_t i = 0; i < count; ++i) {
      for (std::size_t j = i + 1; j < count; ++j) {
        const double dt = track.history[j].time - track.history[i].time;
        if (dt < parameters_.pair_baseline_sec) continue;
        const Eigen::Vector3d velocity = (track.history[j].position - track.history[i].position) / dt;
        for (int axis = 0; axis < 3; ++axis) slopes[axis].push_back(velocity[axis]);
      }
    }
    for (int axis = 0; axis < 3; ++axis) track.velocity[axis] = median(std::move(slopes[axis]));
    double current_station=0,heading=0,lateral=0;
    if(track.size.x()>=3.5 && track.size.y()>=1.4 &&
        ringProjection(track.history.back().position,current_station,heading,lateral)) {
      double circumference=0;
      const auto& ring=parameters_.ring_centerline;
      for(std::size_t i=1;i<ring.size();++i)circumference+=(ring[i]-ring[i-1]).head<2>().norm();
      std::vector<double> stations(count,0);
      std::vector<bool> matched(count,false);
      for(std::size_t i=0;i<count;++i) {
        double h=0,l=0;
        matched[i]=ringProjection(track.history[i].position,stations[i],h,l);
      }
      std::vector<double> progress;
      for(std::size_t i=0;i<count;++i) for(std::size_t j=i+1;j<count;++j) {
        const double dt=track.history[j].time-track.history[i].time;
        if(dt<parameters_.pair_baseline_sec)continue;
        if(!matched[i] || !matched[j])continue;
        double distance=stations[j]-stations[i];distance-=circumference*std::floor(distance/circumference+.5);
        progress.push_back(distance/dt);
      }
      if(!progress.empty()) {
        const double speed=std::max(0.,median(std::move(progress)));
        const double lateral_speed=-track.velocity.x()*std::sin(heading)+track.velocity.y()*std::cos(heading);
        if(speed>.5 || std::abs(lateral_speed)<=1.5) {
          track.velocity.x()=speed*std::cos(heading);track.velocity.y()=speed*std::sin(heading);
        }
      }
    }
    track.acceleration_available = previous_velocity_available && now > previous_velocity_time &&
        count >= static_cast<std::size_t>(parameters_.min_acceleration_observations);
    if (track.acceleration_available)
      track.acceleration = (track.velocity - previous_velocity) / (now - previous_velocity_time);
    track.velocity_time = now;
  }
  std::vector<std::size_t> associateTeamHungarian(const interfaces::Objects& objects,
                                    const std::vector<Eigen::Vector3d>& positions, double time) const {
    const std::size_t rows = positions.size(), columns = tracks_.size() + rows;
    std::vector<std::size_t> result(rows, noTrack());
    if (rows == 0) return result;
    const double unmatched = parameters_.association_gate_m * parameters_.association_gate_m;
    const double forbidden = 1e12;
    std::vector<std::vector<double>> costs(rows, std::vector<double>(columns, unmatched));
    for (std::size_t i = 0; i < rows; ++i) {
      for (std::size_t j = 0; j < tracks_.size(); ++j) {
        const auto& track = tracks_[j];
        const double dt = time - track.history.back().time;
        Eigen::Vector3d expected = track.history.back().position;
        if (track.velocity_available) expected = parameters_.moving_vehicle_coast_sec>0?
            vector(projected(track,time).position):expected+track.velocity*dt;
        const double distance = (positions[i] - expected).squaredNorm();
        const auto expected_body=track.velocity_available?projected(track,time):track.last_object;
        const bool same_body=sameBody(objects.objects[i],expected_body) ||
            sameBody(expected_body,objects.objects[i]);
        const auto& label = objects.objects[i].label;
        if ((distance > unmatched && !same_body) || (label != "unknown" && track.label != "unknown" && label != track.label)) {
          costs[i][j] = forbidden;
          continue;
        }
        const Eigen::Vector3d size = vector(objects.objects[i].bbox_size);
        // Visibility can change either observed extent; compare them symmetrically.
        const Eigen::Vector3d delta = (size - track.size).cwiseQuotient(
          size.cwiseAbs() + track.size.cwiseAbs() + Eigen::Vector3d::Constant(0.1));
        costs[i][j] = (same_body?.25:1.)*distance + parameters_.shape_weight * delta.squaredNorm();
      }
    }
    // Team Tracking::match augmentation; cost/gating and tuning: workspace maintainer.
    std::vector<double> u(rows + 1, 0.0), v(columns + 1, 0.0);
    std::vector<std::size_t> p(columns + 1, 0), way(columns + 1, 0);
    for (std::size_t i = 1; i <= rows; ++i) {
      p[0] = i;
      std::size_t j0 = 0;
      std::vector<double> minv(columns + 1, forbidden);
      std::vector<bool> used(columns + 1, false);
      do {
        used[j0] = true;
        const auto i0 = p[j0];
        double delta = forbidden;
        std::size_t j1 = 0;
        for (std::size_t j = 1; j <= columns; ++j) {
          if (used[j]) continue;
          const double cost = costs[i0 - 1][j - 1];
          const double cur = cost - u[i0] - v[j];
          if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
          if (minv[j] < delta) { delta = minv[j]; j1 = j; }
        }
        for (std::size_t j = 0; j <= columns; ++j) {
          if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
          else minv[j] -= delta;
        }
        j0 = j1;
      } while (p[j0] != 0);
      do {
        const auto j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }
    for (std::size_t j = 1; j <= tracks_.size(); ++j)
      if (p[j] != 0 && costs[p[j] - 1][j - 1] < unmatched)
        result[p[j] - 1] = j - 1;
    return result;
  }
  static std::size_t noTrack() { return std::numeric_limits<std::size_t>::max(); }
  TrackingParameters parameters_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_ = 1;
};
}  // namespace object_detector
