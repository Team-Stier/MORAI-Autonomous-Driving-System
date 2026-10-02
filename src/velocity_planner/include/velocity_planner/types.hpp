#pragma once
#include <node_pipeline/components.hpp>
#include <nav_msgs/Odometry.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <interfaces/TrafficLight.h>
#include <tf2_msgs/TFMessage.h>
namespace velocity_planner {
using Data = node_pipeline::Data<interfaces::Path, nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::TrafficLight, tf2_msgs::TFMessage>;
using Message = interfaces::Path;
}  // namespace velocity_planner
