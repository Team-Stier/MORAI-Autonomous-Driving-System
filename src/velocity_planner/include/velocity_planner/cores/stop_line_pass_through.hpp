#pragma once
#include <velocity_planner/banks/stop_line_state.hpp>
#include <interfaces/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <utility>

namespace velocity_planner {
class StopLinePassThrough {
 public:
  explicit StopLinePassThrough(StopLineState& state) : state_(state) {}
  std::pair<interfaces::Path, tf2_msgs::TFMessage> process(
      const interfaces::Path& path, const tf2_msgs::TFMessage& tf) {
    interfaces::Path output = path;
    state_.green_frames = 0;
    if (state_.stopping) {
      for (auto& point : output.points) {
        point.velocity = 0;
        point.acceleration = 0;
      }
    }
    return {std::move(output), tf};
  }
 private:
  StopLineState& state_;
};
}  // namespace velocity_planner
