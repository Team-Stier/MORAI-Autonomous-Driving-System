#pragma once
#include <node_pipeline/components.hpp>
#include <nav_msgs/Odometry.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <morai_msgs/CtrlCmd.h>
namespace control {
using Data = node_pipeline::Data<interfaces::Path, nav_msgs::Odometry, interfaces::CompetitionVehicleStatus>;
using Message = morai_msgs::CtrlCmd;
}  // namespace control
