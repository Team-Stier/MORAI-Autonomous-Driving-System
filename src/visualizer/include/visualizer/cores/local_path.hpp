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
    // shortcut: colors saturate at 100 km/h; raise the scale if faster profiles are used.
    constexpr double max_speed_mps = 100.0 / 3.6;
    for (const auto& input_point : input.points) {
      auto point = input_point.pose.pose.position;
      point.z = (parameters_.flatten_z ? 0.0 : point.z) + 0.1;
      path.points.push_back(point);
      const double color_position = 4.0 * std::min(1.0, std::max(0.0, input_point.velocity / max_speed_mps));
      std_msgs::ColorRGBA color;
      color.r = std::min(1.0, std::max(0.0, 2.0 - color_position));
      color.g = std::min(1.0, std::min(color_position, 4.0 - color_position));
      color.b = std::min(1.0, std::max(0.0, color_position - 2.0));
      color.a = 1.0;
      path.colors.push_back(color);
    }
    return path;
  }

 private:
  LocalPathParameters parameters_;
};
}  // namespace visualizer
