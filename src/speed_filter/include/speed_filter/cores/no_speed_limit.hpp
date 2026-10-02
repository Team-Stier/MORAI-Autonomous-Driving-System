#pragma once
#include <speed_filter/cores/speed_limit60.hpp>

namespace speed_filter {
class NoSpeedLimit {
 public:
  explicit NoSpeedLimit(SpeedFilterParameters parameters = {0}) : filter_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    return filter_.process(command, status);
  }

 private:
  SpeedLimit60 filter_;
};
}  // namespace speed_filter
