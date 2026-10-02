#pragma once
#include <speed_filter/types.hpp>
#include <cmath>

namespace speed_filter {
class SpeedLimit60 {
 public:
  explicit SpeedLimit60(SpeedFilterParameters parameters = {}) : parameters_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    const double now_sec = status.header.stamp.toSec();
    const double speed_mps = std::fabs(status.signed_velocity_mps);
    // ROS 시각이 되돌아가면 계산 중인 정지·출발 구간의 기준도 다시 잡는다.
    if (now_sec < previous_stamp_sec_) {
      stopped_since_sec_ = -1;
      start_until_sec_ = -1;
    }
    previous_stamp_sec_ = now_sec;
    auto output = command;
    if (parameters_.speed_limit_kph > 0 && speed_mps > parameters_.speed_limit_kph / 3.6) {
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
  SpeedFilterParameters parameters_;
  double previous_stamp_sec_ = 0;
  double stopped_since_sec_ = -1;
  double start_until_sec_ = -1;
};
}  // namespace speed_filter
