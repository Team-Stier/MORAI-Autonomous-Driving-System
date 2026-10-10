#pragma once
#include <Eigen/Dense>
#include <cstdint>

namespace localization {
struct EkfState {
  Eigen::Matrix<double, 6, 1> state = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
  bool initialized = false;
  std::uint64_t previous_imu_stamp = 0;
  std::uint64_t previous_gps_stamp = 0;
  std::uint64_t previous_status_stamp = 0;
  std::uint64_t previous_lidar_stamp = 0;
};
}  // namespace localization
