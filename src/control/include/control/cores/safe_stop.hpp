#pragma once
#include <morai_msgs/CtrlCmd.h>

namespace control {
struct SafeStopParameters {
  double brake = 0.5;
};

// 입력 없이 accel=0·steering=0·설정 brake 명령을 만든다.
class SafeStop {
 public:
  explicit SafeStop(SafeStopParameters parameters) : parameters_(parameters) {}
  morai_msgs::CtrlCmd process() {
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.accel = 0;
    command.steering = 0;
    command.brake = parameters_.brake;
    return command;
  }

 private:
  SafeStopParameters parameters_;
};
}  // namespace control
