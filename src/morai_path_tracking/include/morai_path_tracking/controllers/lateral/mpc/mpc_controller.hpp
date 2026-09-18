#pragma once
#include "morai_path_tracking/controllers/lateral/mpc/optimizer.hpp"
#include "morai_path_tracking/controllers/lateral/mpc/path.hpp"
#include <memory>
#include <deque>
namespace morai_mpc {
class MpcController {
 public:
  explicit MpcController(const MpcConfig& config, std::unique_ptr<Optimizer> optimizer = nullptr);
  MpcCommand compute(const VehicleState& vehicle, const ReferencePath& path,
                     double measurement_dt_sec);
  const Projection& lastProjection() const { return last_projection_; }
  const std::vector<double>& warmStart() const { return warm_start_; }
  const ProgressProjection& lastProgressProjection() const { return last_progress_; }
  int activeHorizon() const { return active_horizon_; }
  double lastRawLateralError() const { return last_raw_lateral_error_; }
  void resetPathTracker() { progress_tracker_.reset(); }
  std::vector<ReferencePoint> referenceHorizon(double start_s, const ReferencePath& path) const;
 private:
  double rolloutCost(const MpcState& initial, double start_s, const ReferencePath& path,
                     const std::vector<double>& controls, double previous_control,
                     double measured_speed) const;
  OptimizationResult solveLinearizedOsqp(
      const MpcState& initial, double start_s, const ReferencePath& path,
      const std::vector<double>& seed, double previous_control,
      double measured_speed) const;
  MpcConfig config_; std::unique_ptr<Optimizer> optimizer_; std::vector<double> warm_start_;
  double active_w_lateral_{0.0}, active_w_heading_{0.0}, active_w_steering_{0.0};
  double active_w_rate_{0.0}, active_w_rate_change_{0.0};
  double active_w_terminal_lateral_{0.0}, active_w_terminal_heading_{0.0};
  bool active_dynamic_model_{false};
  Projection last_projection_;
  PathProgressTracker progress_tracker_;
  ProgressProjection last_progress_;
  int active_horizon_{0};
  double previous_control_{0.0};
  std::deque<double> actuator_delay_line_;
  double predicted_applied_rate_{0.0};
  bool has_filtered_lateral_error_{false};
  double filtered_lateral_error_{0.0};
  double last_raw_lateral_error_{0.0};
  bool has_filtered_steering_rate_{false};
  double filtered_steering_rate_{0.0};
};
}  // namespace morai_mpc
