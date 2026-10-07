#pragma once
#include <Eigen/Dense>
#include <cstdint>
namespace localization {
struct StateBank {
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  Vector6 state = Vector6::Zero();
  Matrix6 covariance = Matrix6::Identity();
  bool initialized = false;
  double altitude_origin_m = 0;
  std::uint64_t previous_imu_stamp = 0;
  std::uint64_t previous_gps_stamp = 0;
  std::uint64_t previous_status_stamp = 0;
  std::uint64_t previous_lidar_stamp = 0;
};
}  // namespace localization
