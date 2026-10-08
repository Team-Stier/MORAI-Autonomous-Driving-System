#pragma once
#include <localization/banks/state_bank.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace localization {
struct TunnelWallParameters {
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  double axis_yaw_rad = 0.0;
  double left_wall_m = 5.0;
  double right_wall_m = -5.0;
  double wall_normal_z_ratio = 0.0;
  double yaw_offset_rad = -0.023509;
  double acceleration_noise_mps2 = 0.3;
  double velocity_noise_mps = 0.01;
  double lateral_velocity_noise_mps = 0.05;
  double wall_noise_m = 0.01;
  double wall_fit_band_m = 0.12;
  int wall_min_points = 20;
  double wall_min_side_m = 0.8;
  double wall_max_range_m = 20.0;
  double wall_min_station_m = -60.0;
  double wall_max_station_m = 60.0;
  double wall_min_height_m = 0.3;
  double wall_max_height_m = 3.5;
};
class TunnelWall {
 public:
  // bank는 FullEkf가 초기화한 상태를 이어받는다. 점군의 x·y·z·time(점군 stamp 기준 초) 필드는
  // Validator timed_cloud.hpp가 선언한다. 측량한 터널 벽 평면과 축 방향은 map 좌표다.
  TunnelWall(TunnelWallParameters parameters, StateBank& bank)
      : parameters_(parameters), bank_(bank),
        tangent_(std::cos(parameters.axis_yaw_rad), std::sin(parameters.axis_yaw_rad), 0),
        normal_(-std::sin(parameters.axis_yaw_rad), std::cos(parameters.axis_yaw_rad), 0) {}

  nav_msgs::Odometry process(const sensor_msgs::Imu& imu,
                            const interfaces::CompetitionVehicleStatus& status,
                            const sensor_msgs::PointCloud2& cloud) {
    const Eigen::Quaterniond orientation =
      Eigen::AngleAxisd(status.yaw_rad + parameters_.yaw_offset_rad, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(status.pitch_rad, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(status.roll_rad, Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
    Eigen::Vector3d acceleration = rotation * Eigen::Vector3d(
      imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    acceleration.z() = 0;  // Same vertical constant-velocity model as FullEkf.
    const auto stamp = imu.header.stamp.toNSec();
    if (stamp != bank_.previous_imu_stamp) {
      const double dt = (static_cast<double>(stamp) - bank_.previous_imu_stamp) * 1e-9;
      bank_.previous_imu_stamp = stamp;
      bank_.state.head<3>() += bank_.state.tail<3>() * dt + 0.5 * acceleration * dt * dt;
      bank_.state.tail<3>() += acceleration * dt;
      Matrix6 transition = Matrix6::Identity();
      transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
      Eigen::Matrix<double, 6, 3> noise;
      noise.topRows<3>() = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
      noise.bottomRows<3>() = Eigen::Matrix3d::Identity() * dt;
      bank_.covariance = transition * bank_.covariance * transition.transpose() +
        std::pow(parameters_.acceleration_noise_mps2, 2) * noise * noise.transpose();
    }
    if (status.header.stamp.toNSec() != bank_.previous_status_stamp) {
      bank_.previous_status_stamp = status.header.stamp.toNSec();
      Vector6 observation = Vector6::Zero();
      observation.tail<3>() = rotation.col(0);
      Vector6 gain = bank_.covariance * observation /
        (observation.dot(bank_.covariance * observation) +
         std::pow(parameters_.velocity_noise_mps, 2));
      // Velocity observations constrain motion; absolute position is anchored separately.
      gain.head<3>().setZero();
      const double status_age_sec = imu.header.stamp.toSec() - status.header.stamp.toSec();
      const double forward_velocity_mps = status.velocity_x_mps +
        status_age_sec * rotation.col(0).dot(acceleration);
      correct(observation, forward_velocity_mps, gain,
              std::pow(parameters_.velocity_noise_mps, 2));
      // Rear-axle rolling model: body lateral velocity is near zero during road driving.
      Vector6 lateral_observation = Vector6::Zero();
      lateral_observation.tail<3>() = rotation.col(1);
      const double lateral_variance = std::pow(parameters_.lateral_velocity_noise_mps, 2);
      Vector6 lateral_gain = bank_.covariance * lateral_observation /
        (lateral_observation.dot(bank_.covariance * lateral_observation) + lateral_variance);
      lateral_gain.head<3>().setZero();
      lateral_gain.tail<3>() = normal_ * normal_.dot(lateral_gain.tail<3>());
      correct(lateral_observation, 0, lateral_gain, lateral_variance);
    }
    if (cloud.header.stamp.toNSec() != bank_.previous_lidar_stamp) {
      bank_.previous_lidar_stamp = cloud.header.stamp.toNSec();
      const double orientation_dt = (cloud.header.stamp.toSec() - status.header.stamp.toSec());
      const Eigen::Vector3d angular_velocity(status.angular_velocity_x_radps,
        status.angular_velocity_y_radps, status.angular_velocity_z_radps);
      const double angle = angular_velocity.norm() * orientation_dt;
      Eigen::Quaterniond scan_orientation = orientation;
      if (angle != 0) scan_orientation = orientation * Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, angular_velocity.normalized()));
      const double age = imu.header.stamp.toSec() - cloud.header.stamp.toSec();
      const auto wall = wallInformation(cloud, scan_orientation, angular_velocity,
        bank_.state.tail<3>() - acceleration * age, acceleration,
        bank_.state.head<3>() - bank_.state.tail<3>() * age + 0.5 * acceleration * age * age);
      Vector6 observation = Vector6::Zero();
      observation.head<3>() = normal_;
      observation.tail<3>() = -age * normal_;
      const double innovation_variance = observation.dot(bank_.covariance * observation);
      Vector6 projected_covariance = bank_.covariance * observation;
      // Walls add no station/height information, including with correlated GPS covariance.
      projected_covariance.head<3>() = normal_ * normal_.dot(projected_covariance.head<3>());
      projected_covariance.tail<3>() = normal_ * normal_.dot(projected_covariance.tail<3>());
      const Vector6 scaled_covariance = projected_covariance /
        (1 + innovation_variance * wall.precision);
      const double acceleration_offset = 0.5 * normal_.dot(acceleration) * age * age;
      bank_.state += scaled_covariance * (wall.weighted_position -
        wall.precision * (observation.dot(bank_.state) + acceleration_offset));
      const Matrix6 residual = Matrix6::Identity() -
        wall.precision * scaled_covariance * observation.transpose();
      bank_.covariance = residual * bank_.covariance * residual.transpose() +
        wall.precision * scaled_covariance * scaled_covariance.transpose();
    }
    const Eigen::Vector3d velocity = rotation.transpose() * bank_.state.tail<3>();
    nav_msgs::Odometry message;
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = bank_.state[0];
    message.pose.pose.position.y = bank_.state[1];
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
  using Vector6 = StateBank::Vector6;
  using Matrix6 = StateBank::Matrix6;
  struct WallInformation { double precision = 0, weighted_position = 0; };
  void addConsensus(std::vector<double> values, WallInformation& information) const {
    std::sort(values.begin(), values.end());
    std::size_t first = 0, best_first = 0, best_count = 0;
    for (std::size_t last = 0; last < values.size(); ++last) {
      while (values[last] - values[first] > 2 * parameters_.wall_fit_band_m) ++first;
      if (last - first + 1 > best_count) {
        best_first = first;
        best_count = last - first + 1;
      }
    }
    // Geometric consensus, not a test/replacement of the native input message.
    if (best_count >= static_cast<std::size_t>(parameters_.wall_min_points)) {
      const std::size_t last = best_first + best_count - 1;
      const double value = 0.5 * (values[(best_first + last) / 2] + values[(best_first + last + 1) / 2]);
      const double precision = 1 / std::pow(parameters_.wall_noise_m, 2);
      information.precision += precision;
      information.weighted_position += precision * value;
    }
  }
  WallInformation wallInformation(const sensor_msgs::PointCloud2& cloud,
                         const Eigen::Quaterniond& orientation,
                         const Eigen::Vector3d& angular_velocity,
                         const Eigen::Vector3d& velocity,
                         const Eigen::Vector3d& acceleration,
                         const Eigen::Vector3d& position) const {
    std::uint32_t offsets[4];
    const char* names[] = {"x", "y", "z", "time"};
    for (std::size_t i = 0; i < 4; ++i) offsets[i] = std::find_if(
      cloud.fields.begin(), cloud.fields.end(), [&](const sensor_msgs::PointField& field) {
        return field.name == names[i];
      })->offset;
    const double angular_speed = angular_velocity.norm();
    const double scan_station = tangent_.dot(position -
      Eigen::Vector3d(parameters_.origin_x_m, parameters_.origin_y_m, 0));
    std::vector<double> left, right;
    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        const auto* record = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
          col * cloud.point_step;
        float coordinates[4];
        for (std::size_t i = 0; i < 4; ++i)
          std::memcpy(&coordinates[i], record + offsets[i], sizeof(float));
        const double point_dt = coordinates[3];
        Eigen::Quaterniond point_orientation = orientation;
        if (angular_speed != 0) point_orientation = orientation * Eigen::Quaterniond(
          Eigen::AngleAxisd(angular_speed * point_dt, angular_velocity / angular_speed));
        const Eigen::Vector3d point = point_orientation * Eigen::Vector3d(
          coordinates[0], coordinates[1], coordinates[2]) +
          velocity * point_dt + 0.5 * acceleration * point_dt * point_dt;
        const double station = scan_station + tangent_.dot(point);
        if (station >= parameters_.wall_min_station_m && station <= parameters_.wall_max_station_m &&
            std::abs(tangent_.dot(point)) <= parameters_.wall_max_range_m &&
            point.z() >= parameters_.wall_min_height_m && point.z() <= parameters_.wall_max_height_m) {
          const double lateral = normal_.dot(point) + parameters_.wall_normal_z_ratio * point.z();
          if (lateral >= parameters_.wall_min_side_m)
            left.push_back(parameters_.left_wall_m - lateral);
          if (lateral <= -parameters_.wall_min_side_m)
            right.push_back(parameters_.right_wall_m - lateral);
        }
      }
    }
    WallInformation information;
    addConsensus(std::move(left), information);
    addConsensus(std::move(right), information);
    information.weighted_position += information.precision *
      normal_.dot(Eigen::Vector3d(parameters_.origin_x_m, parameters_.origin_y_m, 0));
    return information;
  }
  void correct(const Vector6& observation, double value, const Vector6& gain, double variance) {
    bank_.state += gain * (value - observation.dot(bank_.state));
    const Matrix6 residual = Matrix6::Identity() - gain * observation.transpose();
    bank_.covariance = residual * bank_.covariance * residual.transpose() + variance * gain * gain.transpose();
  }
  TunnelWallParameters parameters_;
  StateBank& bank_;
  Eigen::Vector3d tangent_, normal_;
};
}  // namespace localization
