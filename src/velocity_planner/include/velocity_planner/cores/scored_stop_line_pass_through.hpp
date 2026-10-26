#pragma once
#include <velocity_planner/banks/stop_line_state.hpp>
#include <interfaces/Path.h>
#include <interfaces/TrafficLight.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <array>
#include <vector>
#include <utility>

namespace velocity_planner {
struct ScoredStopLinePassThroughParameters {
  std::vector<std::int64_t> scored_stop_line_ids;
  std::vector<std::int64_t> left_signal_stop_line_ids;
};

class ScoredStopLinePassThrough {
 public:
  explicit ScoredStopLinePassThrough(ScoredStopLinePassThroughParameters value, StopLineState& state)
      : parameters_(std::move(value)), state_(state) {}
  std::pair<interfaces::Path, tf2_msgs::TFMessage> process(
      const interfaces::Path& path, const interfaces::TrafficLight& light,
      const tf2_msgs::TFMessage& tf) {
    interfaces::Path output = path;
    const auto id = state_.stop_line_id;
    const bool scored = std::find(parameters_.scored_stop_line_ids.begin(),
        parameters_.scored_stop_line_ids.end(), id) != parameters_.scored_stop_line_ids.end();
    if (state_.stopping && scored) {
      const bool straight = std::find(parameters_.left_signal_stop_line_ids.begin(),
          parameters_.left_signal_stop_line_ids.end(), id) == parameters_.left_signal_stop_line_ids.end();
      if (state_.accepts_circular_green != straight) {
        state_.accepts_circular_green = straight;
        state_.green_frames = 0;
      }
      static const std::array<bool, 17> straight_go{{
        false, false, false, false, true, false, false, true, false,
        true, false, true, true, false, true, true, true
      }};
      static const std::array<bool, 17> left_go{{
        false, false, false, false, false, true, false, false, true,
        false, true, true, false, true, true, true, true
      }};
      const bool go = straight ? straight_go[light.state] : left_go[light.state];
      if (!go) state_.green_frames = 0;
      const auto stamp_ns = light.header.stamp.toNSec();
      if (stamp_ns > state_.last_light_stamp_ns) {
        state_.last_light_stamp_ns = stamp_ns;
        if (go && state_.green_frames < state_.parameters.n_frames) ++state_.green_frames;
      }
      state_.released_stop_line_id = 0;
      if (go && state_.green_frames >= state_.parameters.n_frames) state_.stopping = false;
    } else {
      state_.green_frames = 0;
      if (state_.stopping && path.header.stamp.toSec() - state_.stopping_started_sec >=
          state_.parameters.stop_timeout_sec) {
        state_.released_stop_line_id = state_.stop_line_id;
        state_.stopping = false;
        state_.stop_line_id = 0;
      }
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
  ScoredStopLinePassThroughParameters parameters_;
  StopLineState& state_;
};
}  // namespace velocity_planner
