#pragma once
#include <section_annotator/types.hpp>

namespace section_annotator {
class CheckpointSections {
 public:
  explicit CheckpointSections(CheckpointParameters parameters) : parameters_(parameters) {}
  Message process(const nav_msgs::Odometry& ego) {
    const auto& position = ego.pose.pose.position;
    double closest_distance = std::numeric_limits<double>::infinity();
    std::size_t closest_index = 0;
    for (std::size_t i = 0; i < parameters_.checkpoints.size(); ++i) {
      const auto& point = parameters_.checkpoints[i];
      const double distance = std::hypot(position.x - point.x_m, position.y - point.y_m);
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = i;
      }
    }
    if (closest_distance <= parameters_.radius_m)
      section_ = static_cast<std::uint8_t>(closest_index + 1);
    Message message;
    message.state = section_;
    return message;
  }

 private:
  CheckpointParameters parameters_;
  std::uint8_t section_ = Message::UNKNOWN;
};
}  // namespace section_annotator
