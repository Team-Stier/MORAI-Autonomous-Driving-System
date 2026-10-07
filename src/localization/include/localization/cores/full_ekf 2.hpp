#pragma once
#include <localization/types.hpp>
#include <nav_msgs/Odometry.h>
#include <localization/projection.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
namespace localization {
class FullEkf {
 public:
  explicit FullEkf(EkfParameters parameters = {}) : parameters_(parameters) {}
  nav_msgs::Odometry process(const sensor_msgs::NavSatFix& gps,
                            const sensor_msgs::Imu& imu,
                            const interfaces::CompetitionVehicleStatus& status) {
    const Eigen::Quaterniond orientation =
      Eigen::AngleAxisd(status.yaw_rad + parameters_.yaw_offset_rad, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(status.pitch_rad, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(status.roll_rad, Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
    const auto xy = projectGps(gps.latitude, gps.longitude);
    Eigen::Vector3d position(xy[0], xy[1], gps.altitude);
    position -= rotation * Eigen::Vector3d(0, 0, parameters_.gps_height_m);
    const auto stamp = imu.header.stamp.toNSec();
    if (!initialized_) {
      altitude_origin_m_ = position.z();
      state_.head<3>() << position.x(), position.y(), 0;
      state_.tail<3>() = rotation * Eigen::Vector3d(status.velocity_x_mps, 0, 0);
      previous_imu_stamp_ = stamp;
      initialized_ = true;
    }
    position.z() -= altitude_origin_m_;
    // 같은 IMU를 timer가 다시 선택하면 예측을 중복하지 않는다.
    if (stamp != previous_imu_stamp_) {
      const double dt = (static_cast<double>(stamp) - previous_imu_stamp_) * 1e-9;
      previous_imu_stamp_ = stamp;
      Eigen::Vector3d acceleration = rotation * Eigen::Vector3d(
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
      // ponytail: 수직은 등속 모델, 중력 계약 확정 후 수직 IMU 예측을 연결한다.
      acceleration.z() = 0;
      state_.head<3>() += state_.tail<3>() * dt + 0.5 * acceleration * dt * dt;
      state_.tail<3>() += acceleration * dt;
      Matrix6 transition = Matrix6::Identity();
      transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
      Eigen::Matrix<double, 6, 3> noise;
      noise.topRows<3>() = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
      noise.bottomRows<3>() = Eigen::Matrix3d::Identity() * dt;
      covariance_ = transition * covariance_ * transition.transpose() +
        std::pow(parameters_.acceleration_noise, 2) * noise * noise.transpose();
    }
    if (gps.header.stamp.toNSec() != previous_gps_stamp_) {
      previous_gps_stamp_ = gps.header.stamp.toNSec();
      for (int i = 0; i < 3; ++i) {
        Vector6 observation = Vector6::Zero();
        observation[i] = 1;
        correct(observation, position[i], std::pow(parameters_.gps_position_noise, 2));
      }
    }
    if (status.header.stamp.toNSec() != previous_status_stamp_) {
      previous_status_stamp_ = status.header.stamp.toNSec();
      Vector6 observation = Vector6::Zero();
      observation.tail<3>() = rotation.col(0);
      correct(observation, status.velocity_x_mps, std::pow(parameters_.velocity_noise, 2));
    }
    const Eigen::Vector3d velocity = rotation.transpose() * state_.tail<3>();
    nav_msgs::Odometry message;
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = state_[0];
    message.pose.pose.position.y = state_[1];
    message.pose.pose.position.z = 0;
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    message.twist.twist.linear.x = velocity.x();
    message.twist.twist.linear.y = velocity.y();
    message.twist.twist.linear.z = velocity.z();
    message.twist.twist.angular.x = status.angular_velocity_x_radps;
    message.twist.twist.angular.y = status.angular_velocity_y_radps;
    message.twist.twist.angular.z = status.angular_velocity_z_radps;
    return message;
  }
 private:
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  void correct(const Vector6& observation, double value, double variance) {
    const Vector6 gain = covariance_ * observation /
      (observation.dot(covariance_ * observation) + variance);
    state_ += gain * (value - observation.dot(state_));
    const Matrix6 residual = Matrix6::Identity() - gain * observation.transpose();
    covariance_ = residual * covariance_ * residual.transpose() + variance * gain * gain.transpose();
  }
  EkfParameters parameters_;
  Vector6 state_ = Vector6::Zero();
  Matrix6 covariance_ = Matrix6::Identity();
  bool initialized_ = false;
  double altitude_origin_m_ = 0;
  std::uint64_t previous_imu_stamp_ = 0, previous_gps_stamp_ = 0, previous_status_stamp_ = 0;
};
}
