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
struct AcquisitionLevelerParameters {
  double self_mask_padding_m = .01;
  double history_duration_sec = 1.0;
  double maximum_extrapolation_sec = 0.03;
};
class AcquisitionLeveler {
  struct Pose {
    double time;
    tf2::Quaternion orientation;
    tf2::Vector3 position,velocity;
  };
 public:
  explicit AcquisitionLeveler(AcquisitionLevelerParameters parameters = {}) : parameters_(parameters) {}
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
    std::vector<tf2::Vector3> motion;
    std::vector<bool> keep;keep.reserve(cloud.height*cloud.width);
    motion.reserve(cloud.height*cloud.width);
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
      world_relative+=acquisition.position-reference.position;
      motion.push_back(tf2::quatRotate(level_from_world,world_relative));
    }
    const auto& chosen=motion;
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
  AcquisitionLevelerParameters parameters_;
  std::deque<Pose> history_;
};
}  // namespace lidar_leveler
