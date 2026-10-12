#pragma once
#include <control/cores/pp.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace control {
inline PpParameters loadPpParameters(const node_pipeline::ParamReader& params) {
  PpParameters parameters;
  parameters.wheelbase_m = params.number("wheelbase_m");
  parameters.lookahead_min_m = params.number("lookahead_min_m");
  parameters.lookahead_max_m = params.number("lookahead_max_m");
  parameters.lookahead_time_sec = params.number("lookahead_time_sec");
  parameters.steering_rate_limit_radps = params.number("steering_rate_limit_radps");
  parameters.control_period_sec = params.number("control_period_sec");
  parameters.speed_kp = params.number("speed_kp");
  parameters.speed_ki = params.number("speed_ki");
  parameters.speed_kd = params.number("speed_kd");
  parameters.integral_limit_mps_sec = params.number("integral_limit_mps_sec");
  parameters.acceleration_gain_mps2 = params.number("acceleration_gain_mps2");
  parameters.braking_gain_mps2 = params.number("braking_gain_mps2");
  parameters.maximum_accel = params.number("maximum_accel");
  parameters.maximum_brake = params.number("maximum_brake");
  parameters.stop_speed_mps = params.number("stop_speed_mps");
  parameters.hold_brake = params.number("hold_brake");

  for (double value : {parameters.wheelbase_m, parameters.lookahead_min_m,
                      parameters.lookahead_max_m, parameters.steering_rate_limit_radps,
                      parameters.control_period_sec, parameters.integral_limit_mps_sec,
                      parameters.acceleration_gain_mps2, parameters.braking_gain_mps2,
                      parameters.maximum_accel, parameters.maximum_brake}) {
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("pp parameter must be finite and positive");
  }
  for (double value : {parameters.lookahead_time_sec, parameters.speed_kp, parameters.speed_ki,
                      parameters.speed_kd, parameters.stop_speed_mps, parameters.hold_brake}) {
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("pp parameter must be finite and nonnegative");
  }
  if (parameters.lookahead_min_m > parameters.lookahead_max_m ||
      parameters.maximum_accel > 1 || parameters.maximum_brake > 1 ||
      parameters.hold_brake > parameters.maximum_brake)
    throw std::invalid_argument("pp parameter outside supported range");
  return parameters;
}
}  // namespace control
