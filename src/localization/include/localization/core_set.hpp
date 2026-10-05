#pragma once
#include <localization/cores/full_ekf.hpp>
#include <node_pipeline/components.hpp>
#include <sensor_msgs/PointCloud2.h>
namespace localization {
enum class CoreId { FullEkf };
using Data = node_pipeline::Data<sensor_msgs::NavSatFix, sensor_msgs::Imu,
                                 interfaces::CompetitionVehicleStatus, sensor_msgs::PointCloud2>;
struct CoreSet {
  FullEkf full_ekf;
};
}
