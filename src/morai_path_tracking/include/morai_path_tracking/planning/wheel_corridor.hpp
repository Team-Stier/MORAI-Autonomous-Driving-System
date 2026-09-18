#pragma once

#include <array>
#include <string>
#include <vector>

#include "morai_path_tracking/controllers/lateral/pure_pursuit.hpp"

namespace morai_path_tracking {

struct WheelCorridorConfig {
  double lane_half_width_m{1.30};
  double vehicle_width_m{1.892};
  double wheelbase_m{3.0};
};

struct WheelCorridorResult {
  bool valid{false};
  std::array<double, 4U> signed_wheel_offsets_m{{0.0, 0.0, 0.0, 0.0}};
  double maximum_wheel_offset_m{0.0};
  double minimum_clearance_m{0.0};
  std::string error;
};

struct LaneClearanceSpeedConfig {
  double recovery_start_m{0.18};
  double recovery_full_m{0.05};
  double minimum_speed_mps{10.0 / 3.6};
};

struct LaneClearanceSpeedLimit {
  bool valid{false};
  double urgency{0.0};
  double speed_limit_mps{0.0};
  std::string error;
};

struct HeadingErrorSpeedConfig {
  double recovery_start_rad{4.0 * 3.14159265358979323846 / 180.0};
  double recovery_full_rad{10.0 * 3.14159265358979323846 / 180.0};
  double minimum_speed_mps{10.0 / 3.6};
};

struct HeadingErrorSpeedLimit {
  bool valid{false};
  double urgency{0.0};
  double speed_limit_mps{0.0};
  std::string error;
};

WheelCorridorResult estimateWheelCorridor(
    const std::vector<Point2d>& path_in_vehicle_frame,
    const WheelCorridorConfig& config);

LaneClearanceSpeedLimit computeLaneClearanceSpeedLimit(
    double minimum_wheel_clearance_m, double configured_target_speed_mps,
    const LaneClearanceSpeedConfig& config);

HeadingErrorSpeedLimit computeHeadingErrorSpeedLimit(
    double heading_error_rad, double current_target_speed_mps,
    const HeadingErrorSpeedConfig& config);

}  // namespace morai_path_tracking
