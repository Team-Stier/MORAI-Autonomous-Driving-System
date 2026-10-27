#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>
#include <limits>
#include <random>

namespace lidar_leveler {
struct MotionLevelerParameters {
  double self_mask_padding_m = .01;
  double history_duration_sec = 1.0;
  double maximum_extrapolation_sec = 0.03;
  bool compensate_translation = true;
  double minimum_plane_improvement_ratio = 0.95;
};
class MotionLeveler {
  struct Pose {
    double time;
    tf2::Quaternion orientation;
    tf2::Vector3 position,velocity;
  };
 public:
  explicit MotionLeveler(MotionLevelerParameters parameters = {}) : parameters_(parameters) {}
  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud,
                                  const interfaces::CompetitionVehicleStatus& competition,
                                  const tf2_msgs::TFMessage& tf_static) {
    remember(competition);
    const auto mount=std::find_if(tf_static.transforms.begin(),tf_static.transforms.end(),
        [&](const geometry_msgs::TransformStamped& t) {
          return t.header.frame_id=="base_link" && t.child_frame_id==cloud.header.frame_id;
        });
    const auto& q=mount->transform.rotation;const auto& t=mount->transform.translation;
    const tf2::Transform mounting(tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(t.x,t.y,t.z));
    const double reference_time=cloud.header.stamp.toSec();
    const auto reference=sample(reference_time);
    tf2::Quaternion level_from_world;level_from_world.setRPY(0,0,-heading(reference.orientation));
    auto output=cloud;
    const std::array<const char*,4> names{{"x","y","z","time"}};
    std::array<std::uint32_t,4> offsets;
    for(std::size_t i=0;i<names.size();++i) offsets[i]=std::find_if(cloud.fields.begin(),cloud.fields.end(),
        [&](const sensor_msgs::PointField& f){return f.name==names[i];})->offset;
    std::vector<tf2::Vector3> rigid,motion;
    std::vector<bool> keep;keep.reserve(cloud.height*cloud.width);
    rigid.reserve(cloud.height*cloud.width);motion.reserve(cloud.height*cloud.width);
    for(std::size_t row=0;row<cloud.height;++row) for(std::size_t col=0;col<cloud.width;++col) {
      auto* data=output.data.data()+row*cloud.row_step+col*cloud.point_step;
      std::array<float,4> value;
      for(std::size_t i=0;i<value.size();++i) std::memcpy(&value[i],data+offsets[i],sizeof(float));
      const auto acquisition=sample(reference_time+value[3]);
      const auto body=mounting*tf2::Vector3(value[0],value[1],value[2]);
      // Classify self returns in their acquisition body frame, before deskew.
      const double pad=parameters_.self_mask_padding_m;
      keep.push_back(!(body.x()>=-.79-pad && body.x()<=3.845008373260498+pad &&
          body.y()>=-1.080418-pad && body.y()<=1.080418+pad &&
          body.z()>=-.3685-pad && body.z()<=1.2755463123321533+pad));
      auto world_relative=tf2::quatRotate(acquisition.orientation,body);
      if(parameters_.compensate_translation) world_relative+=acquisition.position-reference.position;
      motion.push_back(tf2::quatRotate(level_from_world,world_relative));
      rigid.push_back(tf2::quatRotate(level_from_world,tf2::quatRotate(reference.orientation,body)));
    }
    const bool use_motion=selectMotion(rigid,motion);
    const auto& chosen=use_motion?motion:rigid;
    std::size_t index=0,retained=0;
    for(std::size_t row=0;row<cloud.height;++row) for(std::size_t col=0;col<cloud.width;++col,++index) {
      if(!keep[index])continue;
      auto* data=output.data.data()+retained*cloud.point_step;
      const auto* source=cloud.data.data()+row*cloud.row_step+col*cloud.point_step;
      std::memcpy(data,source,cloud.point_step);++retained;
      const std::array<float,3> value{{static_cast<float>(chosen[index].x()),
          static_cast<float>(chosen[index].y()),static_cast<float>(chosen[index].z())}};
      for(std::size_t axis=0;axis<3;++axis) std::memcpy(data+offsets[axis],&value[axis],sizeof(float));
    }
    output.height=1;output.width=retained;output.row_step=retained*output.point_step;
    output.data.resize(output.row_step);
    return output;
  }
 private:
  // Compare plane residuals, not horizontal tilt: actual road grade is retained.
  static std::vector<double> planeResiduals(const std::vector<tf2::Vector3>& points) {
    tf2::Vector3 mean(0,0,0);
    for(const auto& p:points) mean+=p;
    mean/=points.size();
    double xx=0,xy=0,yy=0,xz=0,yz=0;
    for(const auto& p:points) {
      const auto d=p-mean;xx+=d.x()*d.x();xy+=d.x()*d.y();yy+=d.y()*d.y();
      xz+=d.x()*d.z();yz+=d.y()*d.z();
    }
    const double determinant=xx*yy-xy*xy;
    std::vector<double> residuals(points.size(),std::numeric_limits<double>::infinity());
    if(std::abs(determinant)<1e-10) return residuals;
    const double a=(xz*yy-yz*xy)/determinant,b=(yz*xx-xz*xy)/determinant;
    const double denominator=std::sqrt(1+a*a+b*b);
    for(std::size_t i=0;i<points.size();++i) {
      const auto d=points[i]-mean;residuals[i]=std::abs(d.z()-a*d.x()-b*d.y())/denominator;
    }
    return residuals;
  }
  bool selectMotion(const std::vector<tf2::Vector3>& rigid,const std::vector<tf2::Vector3>& motion) const {
    std::vector<std::size_t> indices;
    const std::size_t stride=std::max<std::size_t>(1,rigid.size()/1000);
    for(std::size_t i=0;i<rigid.size();i+=stride) {
      const auto& p=rigid[i];
      if(p.x()>4 && p.x()<22 && std::abs(p.y())<4 && p.z()>-2 && p.z()<1) indices.push_back(i);
    }
    // Without a spatially supported road patch, use the kinematic motion model.
    if(indices.size()<40) return true;
    std::minstd_rand generator(17);
    std::uniform_int_distribution<std::size_t> sample_index(0,indices.size()-1);
    std::vector<std::size_t> support;
    for(int iteration=0;iteration<192;++iteration) {
      const auto& a=rigid[indices[sample_index(generator)]];
      const auto& b=rigid[indices[sample_index(generator)]];
      const auto& c=rigid[indices[sample_index(generator)]];
      auto normal=(b-a).cross(c-a);const double length=normal.length();
      if(length<1e-8) continue;
      normal/=length;if(normal.z()<0) normal=-normal;
      if(normal.z()<0.9) continue;
      const double offset=-normal.dot(a),height=-offset/normal.z();
      if(height< -1.5 || height>0.5) continue;
      std::vector<std::size_t> inliers;
      for(auto index:indices) if(std::abs(normal.dot(rigid[index])+offset)<0.055) inliers.push_back(index);
      if(inliers.size()>support.size()) support=std::move(inliers);
    }
    if(support.size()<40) return true;
    indices=std::move(support);
    std::vector<tf2::Vector3> a,b;
    for(auto index:indices) {a.push_back(rigid[index]);b.push_back(motion[index]);}
    const auto ar=planeResiduals(a),br=planeResiduals(b);
    double score_a=0,score_b=0;
    for(std::size_t i=0;i<ar.size();++i) {score_a+=ar[i]*ar[i];score_b+=br[i]*br[i];}
    const double improvement=parameters_.minimum_plane_improvement_ratio;
    return score_b<score_a*improvement*improvement;
  }
  static double heading(const tf2::Quaternion& q) {
    return std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
  }
  void remember(const interfaces::CompetitionVehicleStatus& status) {
    Pose pose;pose.time=status.header.stamp.toSec();
    pose.orientation.setRPY(status.roll_rad,status.pitch_rad,status.yaw_rad);pose.orientation.normalize();
    pose.position.setZero();
    pose.velocity=tf2::Vector3(status.velocity_x_mps*std::cos(status.yaw_rad),
                              status.velocity_x_mps*std::sin(status.yaw_rad),0);
    const auto next=std::lower_bound(history_.begin(),history_.end(),pose.time,
        [](const Pose& p,double time){return p.time<time;});
    if(next!=history_.end() && next->time==pose.time) *next=pose;
    else history_.insert(next,pose);
    while(history_.size()>2 && history_.back().time-history_[1].time>parameters_.history_duration_sec)
      history_.pop_front();
    // Relative integration is re-anchored at the oldest retained observation.
    history_.front().position.setZero();
    for(std::size_t i=1;i<history_.size();++i)
      history_[i].position=history_[i-1].position+(history_[i-1].velocity+history_[i].velocity)*
          (0.5*(history_[i].time-history_[i-1].time));
  }
  Pose sample(double time) const {
    if(history_.size()==1) {
      Pose result=history_.front();result.position+=result.velocity*(time-result.time);return result;
    }
    const auto next=std::upper_bound(history_.begin(),history_.end(),time,
        [](double t,const Pose& p){return t<p.time;});
    const std::size_t right=std::min(history_.size()-1,std::max<std::size_t>(1,next-history_.begin()));
    const auto& a=history_[right-1];const auto& b=history_[right];
    const double evaluation=std::max(a.time-parameters_.maximum_extrapolation_sec,
        std::min(b.time+parameters_.maximum_extrapolation_sec,time));
    const double ratio=(evaluation-a.time)/(b.time-a.time);
    Pose result=a;result.orientation=a.orientation.slerp(b.orientation,ratio);result.orientation.normalize();
    if(ratio<0) result.position=a.position+a.velocity*(evaluation-a.time);
    else if(ratio>1) result.position=b.position+b.velocity*(evaluation-b.time);
    else result.position=a.position+(b.position-a.position)*ratio;
    return result;
  }
  MotionLevelerParameters parameters_;
  std::deque<Pose> history_;
};
}  // namespace lidar_leveler
