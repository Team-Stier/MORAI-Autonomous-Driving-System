#pragma once
#include <node_pipeline/components.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/SectionAnnotation.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
namespace lidar_leveler {
using Data = node_pipeline::Data<sensor_msgs::PointCloud2, interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage>;
using Message = sensor_msgs::PointCloud2;
}  // namespace lidar_leveler
