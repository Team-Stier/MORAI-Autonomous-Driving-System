#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <std_msgs/Bool.h>
#include <utility>

namespace speed_filter {
// /estop true replaces the command with full brake until a false arrives; the latest value received holds.
class EstopBrake {
 public:
  std::pair<morai_msgs::CtrlCmd, interfaces::CompetitionVehicleStatus> process(
      const morai_msgs::CtrlCmd& command, const interfaces::CompetitionVehicleStatus& competition,
      const std_msgs::Bool& estop) {
    morai_msgs::CtrlCmd output = command;
    if (estop.data) {
      output.accel = 0.0;
      output.brake = 1.0;
    }
    return {output, competition};
  }
};
}  // namespace speed_filter
