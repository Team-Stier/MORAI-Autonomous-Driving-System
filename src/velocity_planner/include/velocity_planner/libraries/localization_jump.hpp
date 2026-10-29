#pragma once
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <cmath>

namespace velocity_planner {
// A localization jump (checkpoint return, teleport) moves map->base_link farther than driving allows since the
// previous cutoff call, or rewinds the clock. A stop latched at the old place does not apply at the new one, so the
// latch, the green count and the light stamp are cleared. Returns whether a jump was seen.
template <class State>
bool releaseStopAfterJump(State& state, const tf2_msgs::TFMessage& tf, double stamp_sec) {
  const auto match = std::find_if(tf.transforms.begin(), tf.transforms.end(),
    [](const geometry_msgs::TransformStamped& transform) {
      return transform.header.frame_id == "map" && transform.child_frame_id == "base_link";
    });
  if (match == tf.transforms.end()) return false;
  constexpr double kMaximumElapsedSec = 2.0;
  const double x = match->transform.translation.x, y = match->transform.translation.y;
  bool jumped = false;
  if (state.localization_previous_sec >= 0.0) {
    const double elapsed_sec = std::max(0.0, std::min(kMaximumElapsedSec, stamp_sec - state.localization_previous_sec));
    const double moved_m = std::hypot(x - state.localization_previous_x_m, y - state.localization_previous_y_m);
    jumped = stamp_sec < state.localization_previous_sec ||
             moved_m > state.parameters.jump_distance_m + state.parameters.jump_speed_mps * elapsed_sec;
  }
  if (jumped) {
    state.stopping = false;
    state.stop_line_id = 0;
    state.released_stop_line_id = 0;
    state.green_frames = 0;
    state.last_light_stamp_ns = 0;   // a rewound clock gives older image stamps; GO frames must count again
    state.intersection_remaining_m = -1.0;
  }
  state.localization_previous_x_m = x;
  state.localization_previous_y_m = y;
  state.localization_previous_sec = stamp_sec;
  return jumped;
}
}  // namespace velocity_planner
