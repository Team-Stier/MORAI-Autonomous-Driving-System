#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <cmath>

namespace speed_filter {
struct SpeedLimit59Parameters {
  double stop_tolerance_sec = 30.0;
  double start_duration_sec = 1.0;
  double start_accel = 0.2;
  double stop_speed_mps = 0.1;
};

class SpeedLimit59 {
 public:
  explicit SpeedLimit59(SpeedLimit59Parameters parameters = {}) : parameters_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    const double now_sec = status.header.stamp.toSec();
    const double speed_mps = std::fabs(status.signed_velocity_mps);
    if (now_sec < previous_stamp_sec_) {
      stopped_since_sec_ = -1;
      start_until_sec_ = -1;
    }
    previous_stamp_sec_ = now_sec;
    auto output = command;
    if (speed_mps > 59.0 / 3.6) {
      stopped_since_sec_ = -1;
      start_until_sec_ = -1;
      output.accel = 0;
      output.brake = 1;
      return output;
    }
    if (command.accel > 0) {
      stopped_since_sec_ = -1;
      start_until_sec_ = -1;
      return output;
    }
    const bool stopped = speed_mps <= parameters_.stop_speed_mps;
    if (start_until_sec_ >= 0 && now_sec >= start_until_sec_) {
      start_until_sec_ = -1;
      stopped_since_sec_ = stopped ? now_sec : -1;
    } else if (start_until_sec_ < 0) {
      if (!stopped) stopped_since_sec_ = -1;
      else if (stopped_since_sec_ < 0) stopped_since_sec_ = now_sec;
      if (stopped_since_sec_ >= 0 && now_sec - stopped_since_sec_ >= parameters_.stop_tolerance_sec) {
        start_until_sec_ = now_sec + parameters_.start_duration_sec;
        stopped_since_sec_ = -1;
      }
    }
    if (start_until_sec_ > now_sec) {
      output.accel = parameters_.start_accel;
      output.brake = 0;
      output.steering = 0;
    }
    return output;
  }

 private:
  SpeedLimit59Parameters parameters_;
  double previous_stamp_sec_ = 0;
  double stopped_since_sec_ = -1;
  double start_until_sec_ = -1;
};
}  // namespace speed_filter
