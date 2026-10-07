#pragma once
#include <nav_msgs/Odometry.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <interfaces/TrafficLight.h>
#include <tf2_msgs/TFMessage.h>
namespace velocity_planner {
using Message = interfaces::Path;
}  // namespace velocity_planner
