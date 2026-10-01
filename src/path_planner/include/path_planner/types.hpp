#pragma once
#include <node_pipeline/components.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Objects.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <tf2_msgs/TFMessage.h>
namespace path_planner {
using Data = node_pipeline::Data<geometry_msgs::PoseStamped, interfaces::CompetitionVehicleStatus, interfaces::Objects, tf2_msgs::TFMessage>;
using Message = interfaces::Path;
}  // namespace path_planner
