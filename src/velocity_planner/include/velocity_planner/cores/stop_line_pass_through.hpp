#pragma once
#include <velocity_planner/banks/stop_line_state.hpp>
#include <interfaces/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <utility>
#include <algorithm>
#include <cmath>

namespace velocity_planner {
class StopLinePassThrough {
 public:
  explicit StopLinePassThrough(StopLineState& state) : state_(state) {}
  std::pair<interfaces::Path, tf2_msgs::TFMessage> process(
      const interfaces::Path& path, const tf2_msgs::TFMessage& tf) {
    interfaces::Path output = path;
    releaseAfterJump(path);
    limitIntersectionSpeed(output);
    state_.green_frames = 0;
    if (state_.stopping && path.header.stamp.toSec() - state_.stopping_started_sec >=
        state_.parameters.stop_timeout_sec) {
      state_.released_stop_line_id = state_.stop_line_id;
      state_.stopping = false;
      state_.stop_line_id = 0;
    }
    if (state_.stopping) {
      for (auto& point : output.points) {
        point.velocity = 0;
        point.acceleration = 0;
      }
    }
    return {std::move(output), tf};
  }
 private:
  // A teleport or checkpoint return moves the path origin farther than driving allows since the
  // previous cutoff call; a stop latched at the previous place does not apply to the new one.
  bool releaseAfterJump(const interfaces::Path& path) {
    if (path.points.empty()) return false;
    constexpr double kMaximumElapsedSec = 2.0;
    const auto& p = state_.parameters;
    const auto& origin = path.points.front().pose.pose.position;
    const double stamp_sec = path.header.stamp.toSec();
    bool jumped = false;
    if (state_.previous_path_stamp_sec >= 0) {
      const double elapsed_sec = std::max(0.0, std::min(kMaximumElapsedSec, stamp_sec - state_.previous_path_stamp_sec));
      const double moved_m = std::hypot(origin.x - state_.previous_path_x_m, origin.y - state_.previous_path_y_m);
      jumped = stamp_sec < state_.previous_path_stamp_sec || moved_m > p.jump_distance_m + p.jump_speed_mps * elapsed_sec;
      if (jumped) {
        state_.stopping = false;
        state_.stop_line_id = 0;
        state_.released_stop_line_id = 0;
        state_.green_frames = 0;
        // A rewound clock gives older image stamps; GO confirmation must count them again.
        state_.last_light_stamp_ns = 0;
        state_.intersection_remaining_m = -1.0;
      }
    }
    state_.previous_path_x_m = origin.x;
    state_.previous_path_y_m = origin.y;
    state_.previous_path_stamp_sec = stamp_sec;
    return jumped;
  }
  // Keep this calculation in each selected Core; the Bank carries only state.
  void limitIntersectionSpeed(interfaces::Path& path) {
    const auto& p = state_.parameters;
    if (p.intersection_speed_mps <= 0 || path.points.empty()) return;
    const auto& origin = path.points.front().pose.pose.position;
    if (state_.intersection_remaining_m >= 0) {
      state_.intersection_remaining_m -= std::hypot(origin.x - state_.intersection_previous_x_m,
                                                   origin.y - state_.intersection_previous_y_m);
    }
    state_.intersection_previous_x_m = origin.x;
    state_.intersection_previous_y_m = origin.y;
    if (state_.intersection_remaining_m < 0) return;
    const double distance = std::max(0.0, state_.intersection_remaining_m - p.intersection_after_m - p.intersection_before_m);
    const double limit = std::sqrt(p.intersection_speed_mps * p.intersection_speed_mps +
                                   2 * p.intersection_deceleration_mps2 * distance);
    for (auto& point : path.points) {
      if (point.velocity > limit) {
        point.velocity = limit;
        point.acceleration = std::min(point.acceleration, distance > 0 ? -p.intersection_deceleration_mps2 : 0.0);
      }
    }
  }

  StopLineState& state_;
};
}  // namespace velocity_planner
