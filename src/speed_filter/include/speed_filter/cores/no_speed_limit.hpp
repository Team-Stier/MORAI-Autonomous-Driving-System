#pragma once
#include <speed_filter/calculation.hpp>

namespace speed_filter {
class NoSpeedLimit {
 public:
  explicit NoSpeedLimit(SpeedFilterParameters parameters = {0}) : filter_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    return filter_.calculate(command, status);
  }

 private:
  SpeedFilterCalculation filter_;
};
}  // namespace speed_filter
