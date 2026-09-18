#pragma once

namespace morai_localization {

struct VelocityEstimatorConfig {
  double minimum_dt_sec{0.005};
  double maximum_dt_sec{0.25};
  double maximum_speed_mps{50.0};
  double filter_time_constant_sec{0.10};
};

struct VelocityEstimate {
  bool valid{false};
  double longitudinal_mps{0.0};
  double lateral_mps{0.0};
};

class VelocityEstimator {
 public:
  explicit VelocityEstimator(const VelocityEstimatorConfig& config);
  VelocityEstimate update(double x_m, double y_m, double stamp_sec,
                          double yaw_rad);
  void reset();

 private:
  void setBaseline(double x_m, double y_m, double stamp_sec);
  void clearFilter();

  VelocityEstimatorConfig config_;
  bool has_previous_sample_{false};
  double previous_x_m_{0.0};
  double previous_y_m_{0.0};
  double previous_stamp_sec_{0.0};
  bool has_filtered_velocity_{false};
  double filtered_longitudinal_mps_{0.0};
  double filtered_lateral_mps_{0.0};
};

}  // namespace morai_localization
