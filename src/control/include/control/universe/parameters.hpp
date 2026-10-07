#pragma once
#include <control/universe/mpc/mpc.hpp>
#include <control/universe/pid/pid_longitudinal_controller.hpp>
namespace control { namespace universe {
namespace lateral = autoware::motion::control::mpc_lateral_controller;
namespace longitudinal = autoware::motion::control::pid_longitudinal_controller;
struct Parameters {
  lateral::MPCParam lateral;
  lateral::TrajectoryFilteringParam trajectory_filter;
  longitudinal::PidLongitudinalControllerConfig longitudinal;
  double wheelbase_m = 3.0, maximum_steering_angle_rad = 0.6981317007977318;
  double nominal_dt_sec = 0.05, minimum_control_dt_sec = 0.005, maximum_control_dt_sec = 0.10;
  double steering_lpf_cutoff_hz = 3.0, error_deriv_lpf_cutoff_hz = 5.0;
  double stop_state_entry_ego_speed = 0.001, stop_state_entry_target_speed = 0.001;
  double converged_steer_rad = 0.1, new_traj_duration_time = 1.0, new_traj_end_dist = 0.3;
  double mpc_converged_threshold_rps = 0.01;
  bool use_steer_prediction = false, keep_steer_control_until_converged = true;
  double acceleration_to_pedal_mps2 = 3.0, braking_to_pedal_mps2 = 6.0;
  double maximum_accel_pedal = 0.7, maximum_brake_pedal = 0.6;
  std::vector<std::pair<double,double>> steer_rate_by_curvature{{0.001,40*M_PI/180},{0.002,50*M_PI/180},{0.01,60*M_PI/180}};
  std::vector<std::pair<double,double>> steer_rate_by_velocity{{10,60*M_PI/180},{15,50*M_PI/180},{20,40*M_PI/180}};
};
Parameters defaultParameters();
}}
