#include <control/libraries/tracking/morai_mpc/steering_prediction.hpp>
#include <cmath>
#include <stdexcept>

namespace control { namespace morai_mpc {
SteeringPrediction::SteeringPrediction(double delay_sec, double time_constant_sec)
    : delay_sec_(delay_sec), time_constant_sec_(time_constant_sec) {
  if (!std::isfinite(delay_sec_) || delay_sec_ < 0.0 || delay_sec_ > 0.5 ||
      !std::isfinite(time_constant_sec_) || time_constant_sec_ < 0.01 ||
      time_constant_sec_ > 1.0) {
    throw std::invalid_argument("invalid steering actuator model");
  }
}

double SteeringPrediction::advance(double time_sec) {
  while (!events_.empty() && events_.front().first <= time_sec) {
    const auto event = events_.front();
    events_.pop_front();
    angle_rad_ = command_rad_ + (angle_rad_ - command_rad_) *
        std::exp(-(event.first - time_sec_) / time_constant_sec_);
    time_sec_ = event.first;
    command_rad_ = event.second;
  }
  angle_rad_ = command_rad_ + (angle_rad_ - command_rad_) *
      std::exp(-(time_sec - time_sec_) / time_constant_sec_);
  time_sec_ = time_sec;
  return angle_rad_;
}

void SteeringPrediction::sent(double time_sec, double effective_steering_rad) {
  advance(time_sec);
  events_.emplace_back(time_sec + delay_sec_, effective_steering_rad);
}
}}  // namespace control::morai_mpc
