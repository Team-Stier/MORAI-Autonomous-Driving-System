#pragma once
#include <interfaces/Path.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>

namespace visualizer {
struct LocalPathParameters {
  bool flatten_z = true;
};

class LocalPath {
 public:
  explicit LocalPath(LocalPathParameters parameters = {}) : parameters_(parameters) {}

  visualization_msgs::Marker process(const interfaces::Path& input) {
    visualization_msgs::Marker path;
    path.ns = "local_path";
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.action = input.points.empty() ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.35;
    path.frame_locked = true;
    path.lifetime = ros::Duration(0.5);
    path.points.reserve(input.points.size());
    path.colors.reserve(input.points.size());
    constexpr double max_speed_mps = 100.0 / 3.6;
    for (const auto& input_point : input.points) {
      auto point = input_point.pose.pose.position;
      point.z = (parameters_.flatten_z ? 0.0 : point.z) + 0.1;
      path.points.push_back(point);
      const double color_position = std::min(1.0, std::max(0.0, input_point.velocity / max_speed_mps));
      std_msgs::ColorRGBA color;
      // Keep the green transition within 40–60 km/h on the fixed 0–100 km/h scale.
      if (color_position <= 0.4) {
        color.r = 1.0;
        color.g = color_position / 0.4;
      } else if (color_position <= 0.5) {
        color.r = (0.5 - color_position) / 0.1;
        color.g = 1.0;
      } else if (color_position <= 0.6) {
        color.g = 1.0;
        color.b = (color_position - 0.5) / 0.1;
      } else {
        color.g = (1.0 - color_position) / 0.4;
        color.b = 1.0;
      }
      color.a = 1.0;
      path.colors.push_back(color);
    }
    return path;
  }

 private:
  LocalPathParameters parameters_;
};
}  // namespace visualizer
