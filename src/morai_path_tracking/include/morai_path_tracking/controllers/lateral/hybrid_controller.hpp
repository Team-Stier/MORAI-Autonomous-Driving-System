#pragma once

#include <string>
#include <vector>

#include "morai_path_tracking/controllers/lateral/imm_two_model_filter.hpp"
#include "morai_path_tracking/controllers/lateral/pure_pursuit.hpp"
#include "morai_path_tracking/controllers/lateral/stanley_controller.hpp"

namespace morai_path_tracking {

struct HybridConfig {
  PurePursuitConfig pure_pursuit;
  StanleyConfig stanley;
  ImmConfig imm;
  double pure_pursuit_cross_track_correction_gain{0.0};
  double candidate_conflict_curvature_threshold_m_inv{0.02};
  double candidate_conflict_cross_track_threshold_m{0.5};
  double cross_track_recovery_full_scale_m{0.50};
  double cross_track_recovery_heading_error_suppression_start_rad{
      0.2617993878};
  double cross_track_recovery_heading_error_suppression_full_rad{
      0.3054326191};
  double cross_track_recovery_heading_error_maximum_suppression_ratio{
      0.30};
  double lane_clearance_recovery_start_m{0.18};
  double lane_clearance_recovery_full_m{0.05};
  double curve_preview_stanley_weight_start_m_inv{0.02};
  double curve_preview_stanley_weight_full_m_inv{0.06};
  double curve_preview_stanley_minimum_weight{0.0};
  double heading_lag_stanley_weight_start_rad{0.1396263402};
  double heading_lag_stanley_weight_full_rad{0.2443460953};
  double heading_lag_stanley_minimum_weight{0.0};
  double maximum_steering_angle_rad{0.6981317008};
  double maximum_steering_rate_rad_per_sec{1.0471975512};
  // Zero preserves legacy behavior by inheriting the overall steering-rate
  // limit. Production configurations may set a smaller positive value.
  double low_curvature_maximum_steering_rate_rad_per_sec{0.0};
  double full_steering_rate_curvature_m_inv{0.02};
  double steering_return_rate_multiplier{1.0};
};

struct HybridResult {
  bool valid{false};
  PurePursuitResult pure_pursuit;
  StanleyResult stanley;
  ImmResult imm;
  Point2d pure_pursuit_target;
  Point2d stanley_projection;
  double cg_lateral_velocity_mps{0.0};
  double measured_sideslip_angle_rad{0.0};
  double corrected_pure_pursuit_steering_angle_rad{0.0};
  double effective_pure_pursuit_weight{0.0};
  double effective_stanley_weight{0.0};
  bool candidate_conflict_guard_active{false};
  bool candidate_conflict_stanley_override_active{false};
  bool cross_track_recovery_active{false};
  double cross_track_recovery_weight{0.0};
  bool cross_track_recovery_heading_suppression_active{false};
  double cross_track_recovery_heading_suppression_weight{0.0};
  bool lane_clearance_recovery_active{false};
  double lane_clearance_recovery_urgency{0.0};
  bool curve_preview_stanley_recovery_active{false};
  double curve_preview_stanley_recovery_weight{0.0};
  bool heading_lag_stanley_recovery_active{false};
  double heading_lag_stanley_recovery_weight{0.0};
  double requested_steering_angle_rad{0.0};
  double applied_maximum_steering_rate_rad_per_sec{0.0};
  double steering_angle_rad{0.0};
  std::string error;
};

class HybridController {
 public:
  explicit HybridController(const HybridConfig& config);

  HybridResult calculate(const std::vector<Point2d>& path_in_vehicle_frame,
                         double longitudinal_speed_mps,
                         double rear_axle_lateral_velocity_mps,
                         double measured_yaw_rate_radps,
                         double preview_curvature_m_inv,
                         double previous_steering_angle_rad,
                         double dt_sec);

  HybridResult calculate(const std::vector<Point2d>& path_in_vehicle_frame,
                         double longitudinal_speed_mps,
                         double rear_axle_lateral_velocity_mps,
                         double measured_yaw_rate_radps,
                         double preview_curvature_m_inv,
                         double minimum_wheel_clearance_m,
                         double previous_steering_angle_rad,
                         double dt_sec);

  void reset();

 private:
  HybridConfig config_;
  std::string config_error_;
  StanleyController stanley_controller_;
  ImmTwoModelFilter imm_filter_;
};

}  // namespace morai_path_tracking
