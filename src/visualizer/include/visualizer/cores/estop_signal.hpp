#pragma once
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

namespace visualizer {
// /estop as an overlay text under the traffic-light state: red ON while the stop holds, grey OFF otherwise.
class EstopSignal {
 public:
  visualization_msgs::Marker process(const std_msgs::Bool& message) {
    visualization_msgs::Marker marker;
    marker.ns = "estop";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 8;
    marker.pose.position.z = 2;
    marker.pose.orientation.w = 1;
    marker.scale.z = 1;
    marker.color.r = message.data ? 1.0f : 0.6f;
    marker.color.g = message.data ? 0.0f : 0.6f;
    marker.color.b = message.data ? 0.0f : 0.6f;
    marker.color.a = 1;
    marker.text = message.data ? "E-stop: ON" : "E-stop: OFF";
    marker.frame_locked = true;
    marker.lifetime = ros::Duration(1);
    return marker;
  }
};
}  // namespace visualizer
