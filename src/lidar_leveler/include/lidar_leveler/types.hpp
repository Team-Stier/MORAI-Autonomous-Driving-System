#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/SectionAnnotation.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
namespace lidar_leveler {
struct PassThroughParameters {
  double z_min_m = -0.5;
  double z_max_m = 4.5;
};
using Message = sensor_msgs::PointCloud2;
}  // namespace lidar_leveler
