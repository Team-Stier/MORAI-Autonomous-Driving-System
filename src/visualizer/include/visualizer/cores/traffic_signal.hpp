#pragma once
#include <interfaces/TrafficLight.h>
#include <visualization_msgs/Marker.h>
#include <array>

namespace visualizer {
class TrafficSignal {
 public:
  visualization_msgs::Marker process(const interfaces::TrafficLight& message) {
    struct Signal {
      const char* text;
      float red;
      float green;
      float blue;
    };
    static constexpr std::array<Signal, 17> signals{{
        {"UNKNOWN", 0.6f, 0.6f, 0.6f},
        {"OFF", 0.6f, 0.6f, 0.6f},
        {"RED", 1, 0, 0},
        {"YELLOW", 1, 1, 0},
        {"GREEN", 0, 1, 0},
        {"LEFT", 0, 1, 0},
        {"RED + YELLOW", 1, 0, 0},
        {"RED + GREEN", 1, 0, 0},
        {"RED + LEFT", 1, 0, 0},
        {"YELLOW + GREEN", 1, 1, 0},
        {"YELLOW + LEFT", 1, 1, 0},
        {"GREEN + LEFT", 0, 1, 0},
        {"RED + YELLOW + GREEN", 1, 0, 0},
        {"RED + YELLOW + LEFT", 1, 0, 0},
        {"RED + GREEN + LEFT", 1, 0, 0},
        {"YELLOW + GREEN + LEFT", 1, 1, 0},
        {"RED + YELLOW + GREEN + LEFT", 1, 0, 0},
    }};
    const auto& signal = signals[message.state];
    visualization_msgs::Marker marker;
    marker.ns = "traffic_light";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 8;
    marker.pose.position.z = 3;
    marker.pose.orientation.w = 1;
    marker.scale.z = 1;
    marker.color.r = signal.red;
    marker.color.g = signal.green;
    marker.color.b = signal.blue;
    marker.color.a = 1;
    marker.text = "Traffic light: ";
    marker.text += signal.text;
    marker.frame_locked = true;
    marker.lifetime = ros::Duration(1);
    return marker;
  }
};
}  // namespace visualizer
