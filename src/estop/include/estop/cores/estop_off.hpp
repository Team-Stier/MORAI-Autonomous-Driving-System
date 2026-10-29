#pragma once
#include <interfaces/Path.h>
#include <std_msgs/Bool.h>

namespace estop {
// Sections without the e-stop: always false, so speed_filter passes the command.
class EstopOff {
 public:
  std_msgs::Bool process(const interfaces::Path&) { return std_msgs::Bool{}; }
};
}  // namespace estop
