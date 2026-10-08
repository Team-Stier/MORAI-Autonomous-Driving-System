#pragma once
#include <control/libraries/tracking/controllers/lateral/hybrid_controller.hpp>
#include <control/libraries/tracking/controllers/lateral/mpc/mpc_lateral_controller.hpp>
#include <control/libraries/tracking/controllers/longitudinal/mpc/longitudinal_mpc.hpp>
#include <control/libraries/tracking/planning/wheel_corridor.hpp>
namespace control { namespace tracking {
struct Parameters {
  std::string lateral_controller = "mpc";
  std::string longitudinal_controller = "mpc";
  double nominal_dt_sec = 0.05;
  double minimum_dt_sec = 0.005;
  double maximum_dt_sec = 0.10;
  double speed_filter_time_constant_sec = 0;
  double curvature_preview_distance_m = 8;
  double curvature_sample_spacing_m = 2;
  PurePursuitConfig pure_pursuit;
  StanleyConfig stanley;
  HybridConfig hybrid;
  morai_mpc::MpcConfig mpc;
  WheelCorridorConfig wheel_corridor;
  PidConfig pid;
  LongitudinalMpcConfig longitudinal_mpc;
};
} }  // namespace control::tracking
