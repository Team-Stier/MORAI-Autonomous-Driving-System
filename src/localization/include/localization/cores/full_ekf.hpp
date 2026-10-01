#pragma once
#include <localization/types.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <localization/projection.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <Eigen/Dense>
#include <cmath>
namespace localization {
class FullEkf {
 public:
  explicit FullEkf(EkfParameters parameters = {}) : parameters_(parameters) {}
  geometry_msgs::PoseStamped process(const sensor_msgs::NavSatFix& gps,
                     const sensor_msgs::Imu& imu,
                     const interfaces::CompetitionVehicleStatus& status) {
    const double roll = status.roll_rad, pitch = status.pitch_rad;
    const double yaw = status.yaw_rad + parameters_.yaw_offset_rad;
    auto position = projectGps(gps.latitude, gps.longitude);
    // GPS 안테나 위치를 차량 기준 뒤차축 중심으로 옮긴다.
    position[0] -= parameters_.gps_height_m * (std::cos(yaw) * std::sin(pitch) * std::cos(roll) + std::sin(yaw) * std::sin(roll));
    position[1] -= parameters_.gps_height_m * (std::sin(yaw) * std::sin(pitch) * std::cos(roll) - std::cos(yaw) * std::sin(roll));
    const auto stamp = imu.header.stamp.toNSec();
    // 시작 상태는 첫 측정으로 설정한다. 입력 정상 여부를 판단하는 분기가 아니다.
    if (!initialized_) {
      state_ << position[0], position[1], status.velocity_x_mps, yaw;
      covariance_ = Eigen::Matrix4d::Identity();
      previous_imu_stamp_ = stamp;
      initialized_ = true;
    }
    // 같은 IMU를 timer가 다시 선택하면 predict를 중복하지 않는다.
    if (stamp != previous_imu_stamp_) {
      const double dt = (static_cast<double>(stamp) - previous_imu_stamp_) * 1e-9;
      previous_imu_stamp_ = stamp;
      const double cr = std::cos(roll), sr = std::sin(roll);
      const double cp = std::cos(pitch), sp = std::sin(pitch);
      // GT 자세로 IMU 가속도를 수평 전방 성분으로 회전한다.
      const double acceleration = cp * imu.linear_acceleration.x +
        sp * sr * imu.linear_acceleration.y + sp * cr * imu.linear_acceleration.z;
      const double heading = state_[3], c = std::cos(heading), s = std::sin(heading);
      const double distance = state_[2] * dt + 0.5 * acceleration * dt * dt;
      Eigen::Matrix4d jacobian = Eigen::Matrix4d::Identity();
      jacobian(0, 2) = c * dt; jacobian(1, 2) = s * dt;
      jacobian(0, 3) = -s * distance; jacobian(1, 3) = c * distance;
      state_[0] += c * distance; state_[1] += s * distance;
      state_[2] += acceleration * dt;
      state_[3] += status.angular_velocity_z_radps * dt;
      Eigen::Matrix<double, 4, 2> noise;
      noise << 0.5 * c * dt * dt, 0, 0.5 * s * dt * dt, 0, dt, 0, 0, dt;
      Eigen::Matrix2d q = Eigen::Matrix2d::Zero();
      q(0, 0) = std::pow(parameters_.acceleration_noise, 2);
      q(1, 1) = std::pow(parameters_.yaw_rate_noise, 2);
      covariance_ = jacobian * covariance_ * jacobian.transpose() + noise * q * noise.transpose();
    }
    // 원래 stamp가 같은 관측은 한 번만 융합한다. 센서 갱신 주기가 달라도 유지된다.
    if (gps.header.stamp.toNSec() != previous_gps_stamp_) {
      previous_gps_stamp_ = gps.header.stamp.toNSec();
      for (int i = 0; i < 2; ++i)
        correct(i, position[i], std::pow(parameters_.gps_position_noise, 2));
    }
    if (status.header.stamp.toNSec() != previous_status_stamp_) {
      previous_status_stamp_ = status.header.stamp.toNSec();
      correct(2, status.velocity_x_mps, std::pow(parameters_.velocity_noise, 2));
      const double delta = std::atan2(std::sin(yaw - state_[3]), std::cos(yaw - state_[3]));
      correct(3, state_[3] + delta, std::pow(parameters_.yaw_noise, 2));
    }
    // 공개 자세는 competition_status의 GT만 사용한다.
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    geometry_msgs::PoseStamped message;
    message.pose.position.x = state_[0]; message.pose.position.y = state_[1]; message.pose.position.z = 0;
    message.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    message.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    message.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    message.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    return message;
  }
 private:
  void correct(int component, double observation, double variance) {
    const Eigen::Vector4d gain = covariance_.col(component) / (covariance_(component, component) + variance);
    state_ += gain * (observation - state_[component]);
    Eigen::Matrix4d residual = Eigen::Matrix4d::Identity();
    residual.col(component) -= gain;
    // Joseph 식으로 공분산 대칭·양의 준정부호를 보존한다.
    covariance_ = residual * covariance_ * residual.transpose() + variance * gain * gain.transpose();
  }
  EkfParameters parameters_;
  Eigen::Vector4d state_ = Eigen::Vector4d::Zero();
  Eigen::Matrix4d covariance_ = Eigen::Matrix4d::Identity();
  bool initialized_ = false;
  std::uint64_t previous_imu_stamp_ = 0, previous_gps_stamp_ = 0, previous_status_stamp_ = 0;
};
}
