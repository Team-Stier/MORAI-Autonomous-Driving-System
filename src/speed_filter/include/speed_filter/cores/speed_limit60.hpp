#pragma once
#include <speed_filter/libraries/speed_limiter.hpp>

namespace speed_filter {
class SpeedLimit60 {
 public:
  explicit SpeedLimit60(SpeedFilterParameters parameters = {}) : limiter_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    return limiter_.process(command, status);
  }

 private:
  SpeedLimiter limiter_;
};
}  // namespace speed_filter
