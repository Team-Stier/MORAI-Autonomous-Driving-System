#pragma once
#include <Eigen/Dense>
#include <cstdint>

namespace localization {
// FullEkf와 TunnelWall이 공유하는 map 위치·속도 6상태 필터 상태와 입력별 마지막 반영 시각이다.
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
