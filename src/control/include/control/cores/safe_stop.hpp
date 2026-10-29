#pragma once
#include <morai_msgs/CtrlCmd.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <algorithm>

namespace control {
struct SafeStopParameters {
  double brake = 0.5;
  double steering_limit_rad = 40.0 * 3.14159265358979323846 / 180.0;
};

// #BRAKING_STEERING_CONTINUITY
// 곡선에서 비상제동과 동시에 조향을 0으로 만들면 정지 전 남은 이동 동안
// 도로 밖으로 진행할 수 있다. 같은 차량 상태의 MORAI CW 조향각[rad]을
// CtrlCmd의 CCW 정규화 값으로 변환해 현재 조향을 유지하며 제동한다.
// 새 회피 경로나 방향을 만들지 않는다. 상태 누락·비유한 값은 Core 밖의
// 계약 검사에서 거부하며 Bridge Watchdog은 기존대로 적용한다.
class SafeStop {
 public:
  explicit SafeStop(SafeStopParameters parameters) : parameters_(parameters) {}
  morai_msgs::CtrlCmd process(const interfaces::CompetitionVehicleStatus& status) {
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.accel = 0;
    command.steering = std::max(-1.0, std::min(1.0,
        -static_cast<double>(status.steering_angle_rad) / parameters_.steering_limit_rad));
    command.brake = parameters_.brake;
    return command;
  }

 private:
  SafeStopParameters parameters_;
};
}  // namespace control
