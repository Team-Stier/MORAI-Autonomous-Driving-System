#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <cmath>

namespace speed_filter {
struct SpeedFilterParameters {
  double speed_limit_kph = 60.0;  // 0이면 속도 제한 비활성.
  double stop_tolerance_sec = 3.0;
  double start_duration_sec = 1.0;
  double start_accel = 0.2;
  double stop_speed_mps = 0.1;
};

// 속도 제한과 장기 정지 후 출발 보조를 계산한다. 두 Core가 각자 소유한다.
class SpeedLimiter {
 public:
  explicit SpeedLimiter(SpeedFilterParameters parameters = {}) : parameters_(parameters) {}

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
    // 상위 제어기의 명시적 제동은 출발 보조가 덮어쓰지 않는다.
    if (command.brake > 0) {
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
