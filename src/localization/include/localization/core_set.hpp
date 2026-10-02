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
  std::array<std::size_t, 3> requiredInputs(CoreId) const { return {{0, 1, 2}}; }
  nav_msgs::Odometry process(CoreId, const Data& data) {
    return full_ekf.process(data.get<0>().message(), data.get<1>().message(), data.get<2>().message());
  }
};
}
