#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <object_detector/libraries/types.hpp>
#include <tf2_msgs/TFMessage.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <tuple>
namespace object_detector {
class AcquisitionClusterTf {
 public:
  std::tuple<Clusters,interfaces::CompetitionVehicleStatus,tf2_msgs::TFMessage> process(
      const Clusters& clusters,const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& tf,const ros::Time& stamp) {
    const auto& selected=*std::find_if(tf.transforms.begin(),tf.transforms.end(),
        [](const geometry_msgs::TransformStamped& q){return q.header.frame_id=="map" && q.child_frame_id=="base_link_level";});
    const auto& q=selected.transform.rotation;const auto& t=selected.transform.translation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const double time=selected.header.stamp.toSec();double rate=0;
    if(time>previous_time_ && previous_time_>=0 && time-previous_time_<=.5)
      rate=std::atan2(std::sin(yaw-previous_yaw_),std::cos(yaw-previous_yaw_))/(time-previous_time_);
    else if(time==previous_time_)rate=previous_rate_;
    rate=std::max(-1.,std::min(1.,rate));previous_time_=time;previous_yaw_=yaw;previous_rate_=rate;
    const double dt=std::max(-.1,std::min(.1,stamp.toSec()-time));
    const double distance=competition.velocity_x_mps*dt;
    const Eigen::Vector3d translation(t.x+distance*std::cos(yaw+.5*rate*dt),
                                      t.y+distance*std::sin(yaw+.5*rate*dt),t.z);
    const Eigen::AngleAxisd rotation(yaw+rate*dt,Eigen::Vector3d::UnitZ());
    auto output=clusters;
    for(auto& cloud:output) {
      for(auto& point:cloud) {
        const Eigen::Vector3d position=rotation*Eigen::Vector3d(point.x,point.y,point.z)+translation;
        point.x=position.x();point.y=position.y();point.z=position.z();
      }
      cloud.header.frame_id="map"; // Acquisition time remains the time of the returns.
    }
    return std::make_tuple(std::move(output),competition,tf);
  }
 private:
  double previous_time_=-1,previous_yaw_=0,previous_rate_=0;
};
}
