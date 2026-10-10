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
  };
  double x_m = 5.0;
  double y_m = 100.0;
  double z_mps2 = 3.0;
  int n_frames = 3;
  std::vector<StopLine> stop_lines;
};

struct StopLineState {
  explicit StopLineState(StopLineStateParameters parameters) : parameters(std::move(parameters)) {}
  StopLineStateParameters parameters;
  bool stopping = false;
  std::int64_t stop_line_id = 0;
  std::uint64_t last_light_stamp_ns = 0;
  int green_frames = 0;
  bool accepts_circular_green = true;
};
}  // namespace velocity_planner
