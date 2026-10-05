#pragma once
#include <deque>
#include <utility>

namespace control { namespace morai_mpc {
// A command-history estimate, never a measured wheel-angle report.
class SteeringPrediction {
 public:
  SteeringPrediction(double delay_sec, double time_constant_sec);
  double advance(double time_sec);
  void sent(double time_sec, double effective_steering_rad);

 private:
  double delay_sec_;
  double time_constant_sec_;
  double time_sec_ = 0.0;
  double angle_rad_ = 0.0;
  double command_rad_ = 0.0;
  std::deque<std::pair<double, double>> events_;
};
}}  // namespace control::morai_mpc
