#pragma once
#include <node_pipeline/components.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/SectionAnnotation.h>
#include <morai_msgs/CtrlCmd.h>
namespace speed_filter {
using Data = node_pipeline::Data<morai_msgs::CtrlCmd, interfaces::CompetitionVehicleStatus>;
using Message = morai_msgs::CtrlCmd;
}  // namespace speed_filter
