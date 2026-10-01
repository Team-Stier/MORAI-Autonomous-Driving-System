#pragma once
namespace localization {
struct EkfParameters {
  double acceleration_noise = 1.0;   // m/s² 표준편차
  double yaw_rate_noise = 0.02;      // rad/s 표준편차
  double gps_position_noise = 1.5;   // m 표준편차
  double velocity_noise = 0.05;      // m/s 표준편차
  double yaw_noise = 0.001;          // rad 표준편차
  double gps_height_m = 1.2;        // base_link 위 GPS 장착 높이
  double yaw_offset_rad = -0.023509; // GT heading→UTM 격자 heading
};
}
