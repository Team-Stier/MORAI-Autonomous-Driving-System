#pragma once
#include <nav_msgs/Odometry.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <morai_msgs/CtrlCmd.h>
#include <sensor_msgs/Imu.h>
namespace control {
using Message = morai_msgs::CtrlCmd;
}  // namespace control
