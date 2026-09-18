#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "morai_path_tracking/controllers/lateral/pure_pursuit.hpp"

namespace morai_path_tracking {

struct StanleyConfig {
  double wheelbase_m{3.0};
  double gain{1.0};
  double softening_speed_mps{2.0};
  double minimum_control_speed_mps{1.0};
  double heading_window_m{4.0};
  double heading_error_gain{1.0};
  double curvature_feedforward_gain{0.0};
  double curvature_preview_distance_m{8.0};
  double yaw_rate_damping_gain_sec{0.0};
  double yaw_rate_damping_nonlinear_gain_sec2{0.0};
  double maximum_steering_angle_rad{0.6981317008};
  double maximum_steering_rate_rad_per_sec{1.5707963268};
};

struct StanleyResult {
  bool valid{false};
  Point2d target;
  double steering_angle_rad{0.0};
  double cross_track_error_m{0.0};
  double heading_error_rad{0.0};
  double reference_curvature_m_inv{0.0};
  double reference_yaw_rate_radps{0.0};
  double yaw_rate_error_radps{0.0};
  double curvature_feedforward_steering_rad{0.0};
  double heading_feedback_steering_rad{0.0};
  double cross_track_feedback_steering_rad{0.0};
  double applied_yaw_rate_damping_gain_sec{0.0};
  double yaw_rate_damping_steering_rad{0.0};
  double requested_steering_angle_rad{0.0};
  std::size_t target_segment_index{0U};
  std::string error;
};

class StanleyController {
 public:
  explicit StanleyController(const StanleyConfig& config);

  StanleyResult calculate(const std::vector<Point2d>& path_in_vehicle_frame,
                          double speed_mps,
                          double measured_yaw_rate_radps,
                          double previous_steering_angle_rad,
                          double dt_sec) const;

  StanleyResult calculate(const std::vector<Point2d>& path_in_vehicle_frame,
                          double speed_mps,
                          double previous_steering_angle_rad,
                          double dt_sec) const {
    return calculate(path_in_vehicle_frame, speed_mps, 0.0,
                     previous_steering_angle_rad, dt_sec);
  }

 private:
  StanleyConfig config_;
  std::string config_error_;
};

}  // namespace morai_path_tracking
