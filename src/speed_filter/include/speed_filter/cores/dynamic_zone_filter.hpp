#pragma once
#include <speed_filter/libraries/speed_limiter.hpp>

namespace speed_filter {
class DynamicZoneFilter {
 public:
  explicit DynamicZoneFilter(SpeedFilterParameters parameters = {}) : limiter_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    if (command.brake >= 1.0) {auto output=command;output.accel=0;return output;}
    return limiter_.process(command, status);
  }

 private:
  SpeedLimiter limiter_;
};
}  // namespace speed_filter
