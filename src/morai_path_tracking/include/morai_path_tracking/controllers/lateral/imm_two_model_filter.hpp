#pragma once

#include <array>
#include <string>

namespace morai_path_tracking {

struct ImmConfig {
  double mass_kg{2000.0};
  double yaw_inertia_kgm2{4000.0};
  double front_cornering_stiffness_n_per_rad{60000.0};
  double rear_cornering_stiffness_n_per_rad{60000.0};
  double front_axle_to_cg_m{1.5};
  double rear_axle_to_cg_m{1.5};
  double process_noise_sideslip{0.1};
  double process_noise_yaw_rate{0.01};
  double measurement_noise_sideslip{0.001};
  double measurement_noise_yaw_rate{0.001};
  double initial_covariance_sideslip{0.1};
  double initial_covariance_yaw_rate{0.01};
  double initial_pure_pursuit_probability{0.8};
  double initial_stanley_probability{0.2};
  double stanley_probability_min{0.05};
  double stanley_probability_max{0.95};
  double transition_pure_pursuit_to_pure_pursuit{0.9};
  double transition_pure_pursuit_to_stanley{0.1};
  double transition_stanley_to_pure_pursuit{0.95};
  double transition_stanley_to_stanley{0.05};
  double transition_speed_gain{0.1};
  double transition_reference_speed_mps{60.0 / 3.6};
  double minimum_model_speed_mps{1.0};
};

struct ImmModelState {
  double sideslip_rad{0.0};
  double yaw_rate_radps{0.0};
};

struct ImmResult {
  bool valid{false};
  double pure_pursuit_probability{0.0};
  double stanley_probability{0.0};
  double pure_pursuit_innovation_norm{0.0};
  double stanley_innovation_norm{0.0};
  ImmModelState pure_pursuit_state;
  ImmModelState stanley_state;
  std::string error;
};

class ImmTwoModelFilter {
 public:
  explicit ImmTwoModelFilter(const ImmConfig& config);

  ImmResult update(double longitudinal_speed_mps,
                   double measured_sideslip_rad,
                   double measured_yaw_rate_radps,
                   double pure_pursuit_steering_rad,
                   double stanley_steering_rad,
                   double dt_sec);

  void reset();

 private:
  struct Matrix2 {
    double m00{0.0};
    double m01{0.0};
    double m10{0.0};
    double m11{0.0};
  };

  struct Branch {
    ImmModelState state;
    Matrix2 covariance;
  };

  ImmConfig config_;
  std::string config_error_;
  std::array<Branch, 2> branches_;
  std::array<double, 2> probabilities_;
};

}  // namespace morai_path_tracking
