#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <utility>

namespace speed_filter {
// No /estop received yet: the command passes unchanged.
class EstopPass {
 public:
  std::pair<morai_msgs::CtrlCmd, interfaces::CompetitionVehicleStatus> process(
      const morai_msgs::CtrlCmd& command, const interfaces::CompetitionVehicleStatus& competition) {
    return {command, competition};
  }
};
}  // namespace speed_filter
