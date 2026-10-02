#pragma once
#include <node_pipeline/components.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/SectionAnnotation.h>
#include <morai_msgs/CtrlCmd.h>
namespace speed_filter {
struct SpeedFilterParameters {
  double speed_limit_kph = 60.0;  // 0이면 속도 제한 비활성.
  double stop_tolerance_sec = 3.0;
  double start_duration_sec = 1.0;
  double start_accel = 0.2;
  double stop_speed_mps = 0.1;
};
using Data = node_pipeline::Data<morai_msgs::CtrlCmd, interfaces::CompetitionVehicleStatus>;
using Message = morai_msgs::CtrlCmd;
}  // namespace speed_filter
