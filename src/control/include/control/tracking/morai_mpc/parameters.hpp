#pragma once
#include <control/tracking/morai_mpc/longitudinal_mpc.hpp>

namespace control { namespace morai_mpc {
// Operating values from MOLIT2026-stier1, including launch overrides.
// The supported lateral model/solver are kinematics + qpoases_hotstart.
struct LateralParameters {
  double ctrl_period = 1.0 / 30.0;
  bool enable_path_smoothing = false;
  bool enable_yaw_recalculation = true;
  int path_filter_moving_ave_num = 35;
  int path_smoothing_times = 1;
  int curvature_smoothing_num = 14;
  int morai_yaw_smoothing_num = 14;
  double traj_resample_dist = 0.25;
  double vehicle_model_wheelbase = 3.0;
  double steer_lim_deg = 40.0;
  double morai_steer_rate_limit_deg = 60.0;
  int mpc_prediction_horizon = 45;
  double mpc_prediction_sampling_time = 0.1;
  double mpc_weight_lat_error = 10.0;
  double mpc_weight_heading_error = 3.0;
  double mpc_weight_heading_error_squared_vel_coeff = 0.3;
  double mpc_weight_steering_input = 1.0;
  double mpc_weight_steering_input_squared_vel_coeff = 1.0;
  double mpc_weight_lat_jerk = 1.0;
  double mpc_weight_terminal_lat_error = 10.0;
  double mpc_weight_terminal_heading_error = 0.1;
  double mpc_zero_ff_steer_deg = 0.0;
  double steering_lpf_cutoff_hz = 3.0;
  int qpoases_max_iter = 500;
  double morai_min_prediction_length_m = 20.0;
  int morai_max_prediction_horizon = 90;
  bool morai_use_output_steering_filter = false;
  double morai_straight_lat_weight_scale = 0.2;
  double morai_curve_steer_weight_scale = 0.25;
  double morai_weight_curvature_low = 0.005;
  double morai_weight_curvature_high = 0.03;
  double morai_weight_first_steer_change = 2.0;
  double morai_weight_front_point_error = 0.0;
  double morai_front_overhang_m = 0.845;
};

// Shared by the MPC model and command-history predictor, as in the source adapter.
struct SteeringPredictionParameters {
  double delay_sec = 1.0 / 6.0;
  double time_constant_sec = 0.05;
  double gain = 0.77;
};

struct Parameters {
  LateralParameters lateral;
  LongitudinalMpcConfig longitudinal;
  SteeringPredictionParameters steering_prediction;
  double minimum_control_dt_sec = 0.005;
  double maximum_control_dt_sec = 0.10;
};

constexpr double kPi = 3.14159265358979323846;
// The Bridge contract is fixed independently of a lower optimizer steering limit.
constexpr double kBridgeSteeringLimitRad = 40.0 * kPi / 180.0;
void validateParameters(const Parameters& parameters);
}}  // namespace control::morai_mpc
