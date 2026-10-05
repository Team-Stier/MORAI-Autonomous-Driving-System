#include <control/tracking/morai_mpc/parameters.hpp>
#include <control/tracking/morai_mpc/autoware/curvature_weight_blend.h>
#include <control/tracking/morai_mpc/autoware/prediction_length.h>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace control { namespace morai_mpc {
namespace {
void positive(std::initializer_list<double> values) {
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("morai_mpc parameter must be finite and positive");
  }
}
void nonnegative(std::initializer_list<double> values) {
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("morai_mpc parameter must be finite and nonnegative");
  }
}
}  // namespace

void validateParameters(const Parameters& parameters) {
  const auto& p = parameters.lateral;
  const auto& s = parameters.steering_prediction;
  positive({p.ctrl_period, p.traj_resample_dist, p.vehicle_model_wheelbase,
      p.steer_lim_deg, p.morai_steer_rate_limit_deg, p.mpc_prediction_sampling_time,
      p.steering_lpf_cutoff_hz, p.morai_straight_lat_weight_scale,
      p.morai_curve_steer_weight_scale, s.time_constant_sec, s.gain,
      parameters.minimum_control_dt_sec, parameters.maximum_control_dt_sec});
  nonnegative({p.mpc_weight_lat_error, p.mpc_weight_heading_error,
      p.mpc_weight_heading_error_squared_vel_coeff, p.mpc_weight_steering_input,
      p.mpc_weight_steering_input_squared_vel_coeff, p.mpc_weight_lat_jerk,
      p.mpc_weight_terminal_lat_error, p.mpc_weight_terminal_heading_error,
      p.mpc_zero_ff_steer_deg, p.morai_weight_first_steer_change,
      p.morai_weight_front_point_error, p.morai_front_overhang_m, s.delay_sec});
  if (p.ctrl_period < parameters.minimum_control_dt_sec ||
      p.ctrl_period > parameters.maximum_control_dt_sec ||
      p.steering_lpf_cutoff_hz * p.ctrl_period >= 0.5 ||
      p.steer_lim_deg > 40.0 || p.morai_max_prediction_horizon > 150 ||
      p.mpc_prediction_horizon > 150 || p.morai_min_prediction_length_m > 50.0 ||
      p.qpoases_max_iter <= 0 || p.path_filter_moving_ave_num < 1 ||
      p.path_smoothing_times < 1 || p.curvature_smoothing_num < 1 ||
      p.morai_yaw_smoothing_num < 1 || p.morai_yaw_smoothing_num > 50 ||
      p.morai_straight_lat_weight_scale > 1.0 || p.morai_curve_steer_weight_scale > 1.0 ||
      p.morai_weight_first_steer_change > 100.0 ||
      p.morai_weight_front_point_error > 100.0 || p.morai_front_overhang_m > 3.0 ||
      p.vehicle_model_wheelbase + p.morai_front_overhang_m > 8.0 ||
      s.delay_sec > 0.5 || s.time_constant_sec < 0.01 || s.time_constant_sec > 1.0 ||
      s.gain < 0.5 || s.gain > 1.5 || parameters.longitudinal.horizon_steps < 2 ||
      parameters.longitudinal.solver != "osqp") {
    throw std::invalid_argument("morai_mpc parameter outside supported range");
  }
  autoware::moraiPredictionSteps(p.mpc_prediction_horizon, p.morai_max_prediction_horizon,
      p.mpc_prediction_sampling_time, 0.0, p.morai_min_prediction_length_m);
  autoware::moraiCurvatureWeightBlend(0.0, p.morai_weight_curvature_low,
                                     p.morai_weight_curvature_high);
  // Reuse the source numerical controller's configuration validation.
  LongitudinalMpc longitudinal(parameters.longitudinal);
}
}}  // namespace control::morai_mpc
