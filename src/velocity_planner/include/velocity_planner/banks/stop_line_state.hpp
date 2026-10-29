#pragma once
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace velocity_planner {
struct StopLineStateParameters {
  struct StopLine {
    std::int64_t id;
    std::vector<std::array<double, 2>> points;
    bool directional_extension = false;
  };
  double stop_line_extension_m = 0.0;
  double x_m = 5.0;
  double y_m = 100.0;
  double z_mps2 = 3.0;
  int n_frames = 3;
  double stop_timeout_sec = 100.0;
  double stop_response_time_sec = 1.0;
  double stop_position_tolerance_m = 0.2;
  double intersection_speed_mps = 0.0;
  double intersection_before_m = 25.0;
  double intersection_after_m = 40.0;
  double intersection_deceleration_mps2 = 2.5;
  double jump_distance_m = 5.0;
  double jump_speed_mps = 30.0;
  std::vector<StopLine> stop_lines;
};

struct StopLineState {
  explicit StopLineState(StopLineStateParameters parameters) : parameters(std::move(parameters)) {}
  StopLineStateParameters parameters;
  bool stopping = false;
  std::int64_t stop_line_id = 0;
  double stopping_started_sec = 0.0;
  std::int64_t released_stop_line_id = 0;
  std::uint64_t last_light_stamp_ns = 0;
  int green_frames = 0;
  bool accepts_circular_green = true;
  double intersection_remaining_m = -1.0;
  double intersection_previous_x_m = 0.0;
  double intersection_previous_y_m = 0.0;
  double previous_path_x_m = 0.0;
  double previous_path_y_m = 0.0;
  double previous_path_stamp_sec = -1.0;
};
}  // namespace velocity_planner
