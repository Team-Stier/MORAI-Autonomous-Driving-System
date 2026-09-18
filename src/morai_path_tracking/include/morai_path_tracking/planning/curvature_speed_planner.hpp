#pragma once

#include <vector>

#include "morai_path_tracking/controllers/lateral/pure_pursuit.hpp"

namespace morai_path_tracking {

struct CurvatureSpeedPlannerConfig {
  double configured_target_speed_mps{10.0};
  double minimum_curve_speed_mps{0.0};
  double maximum_lateral_acceleration_mps2{1.5};
  double curvature_speed_reduction_gain_m{0.0};
  double preview_distance_m{20.0};
  double lookahead_curvature_preview_distance_m{8.0};
  double curvature_sample_spacing_m{2.0};
  double curve_approach_deceleration_mps2{3.0};
  double curvature_epsilon_m_inv{0.001};
  double target_speed_filter_time_constant_sec{0.0};
  double target_speed_acceleration_limit_mps2{1.0};
  double curve_target_speed_acceleration_limit_mps2{1.0};
  double target_speed_deceleration_limit_mps2{2.0};
};

struct CurvatureSpeedPlan {
  double preview_curvature_m_inv{0.0};
  double speed_limiting_curve_distance_m{-1.0};
  double lookahead_curvature_m_inv{0.0};
  double curvature_speed_limit_mps{0.0};
  double raw_target_speed_mps{0.0};
  double filtered_target_speed_mps{0.0};
  double target_speed_mps{0.0};
};

class CurvatureSpeedPlanner {
 public:
  explicit CurvatureSpeedPlanner(const CurvatureSpeedPlannerConfig& config);

  CurvatureSpeedPlan update(const std::vector<Point2d>& path_in_vehicle_frame,
                            double dt_sec);
  void reset() noexcept;

 private:
  CurvatureSpeedPlannerConfig config_;
  bool has_last_target_{false};
  double filtered_target_speed_mps_{0.0};
  double last_target_speed_mps_{0.0};
};

}  // namespace morai_path_tracking
