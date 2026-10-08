#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <utility>
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
  Eigen::Matrix<double, 6, 1> initial_state = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> initial_covariance = Eigen::Matrix<double, 6, 6>::Identity();
  std::uint64_t initial_previous_imu_stamp = 0;
  std::uint64_t initial_previous_status_stamp = 0;
  std::uint64_t initial_previous_lidar_stamp = 0;
};
class TunnelWall {
 public:
  // Contract: parameters carry an initialized map-state snapshot; cloud has finite little-endian
  // FLOAT32 XYZ and point time (seconds from cloud stamp) in base_link.
  // Wall consensus supplies lateral information.
  // The surveyed wall planes and heading are in the repository map frame.
  explicit TunnelWall(TunnelWallParameters parameters)
      : parameters_(parameters), state_(parameters.initial_state),
        covariance_(parameters.initial_covariance),
        previous_imu_stamp_(parameters.initial_previous_imu_stamp),
        previous_status_stamp_(parameters.initial_previous_status_stamp),
        previous_lidar_stamp_(parameters.initial_previous_lidar_stamp),
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
    if (stamp != previous_imu_stamp_) {
      const double dt = (static_cast<double>(stamp) - previous_imu_stamp_) * 1e-9;
      previous_imu_stamp_ = stamp;
      state_.head<3>() += state_.tail<3>() * dt + 0.5 * acceleration * dt * dt;
      state_.tail<3>() += acceleration * dt;
      Matrix6 transition = Matrix6::Identity();
      transition.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
      Eigen::Matrix<double, 6, 3> noise;
      noise.topRows<3>() = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
      noise.bottomRows<3>() = Eigen::Matrix3d::Identity() * dt;
      covariance_ = transition * covariance_ * transition.transpose() +
        std::pow(parameters_.acceleration_noise_mps2, 2) * noise * noise.transpose();
    }
    if (status.header.stamp.toNSec() != previous_status_stamp_) {
      previous_status_stamp_ = status.header.stamp.toNSec();
      Vector6 observation = Vector6::Zero();
      observation.tail<3>() = rotation.col(0);
      Vector6 gain = covariance_ * observation /
        (observation.dot(covariance_ * observation) +
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
      Vector6 lateral_gain = covariance_ * lateral_observation /
        (lateral_observation.dot(covariance_ * lateral_observation) + lateral_variance);
      lateral_gain.head<3>().setZero();
      lateral_gain.tail<3>() = normal_ * normal_.dot(lateral_gain.tail<3>());
      correct(lateral_observation, 0, lateral_gain, lateral_variance);
    }
    if (cloud.header.stamp.toNSec() != previous_lidar_stamp_) {
      previous_lidar_stamp_ = cloud.header.stamp.toNSec();
      const double orientation_dt = (cloud.header.stamp.toSec() - status.header.stamp.toSec());
      const Eigen::Vector3d angular_velocity(status.angular_velocity_x_radps,
        status.angular_velocity_y_radps, status.angular_velocity_z_radps);
      const double angle = angular_velocity.norm() * orientation_dt;
      Eigen::Quaterniond scan_orientation = orientation;
      if (angle != 0) scan_orientation = orientation * Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, angular_velocity.normalized()));
      const double age = imu.header.stamp.toSec() - cloud.header.stamp.toSec();
      const auto wall = wallInformation(cloud, scan_orientation, angular_velocity,
        state_.tail<3>() - acceleration * age, acceleration,
        state_.head<3>() - state_.tail<3>() * age + 0.5 * acceleration * age * age);
      Vector6 observation = Vector6::Zero();
      observation.head<3>() = normal_;
      observation.tail<3>() = -age * normal_;
      const double innovation_variance = observation.dot(covariance_ * observation);
      Vector6 projected_covariance = covariance_ * observation;
      // Walls add no station/height information, including with correlated GPS covariance.
      projected_covariance.head<3>() = normal_ * normal_.dot(projected_covariance.head<3>());
      projected_covariance.tail<3>() = normal_ * normal_.dot(projected_covariance.tail<3>());
      const Vector6 scaled_covariance = projected_covariance /
        (1 + innovation_variance * wall.precision);
      const double acceleration_offset = 0.5 * normal_.dot(acceleration) * age * age;
      state_ += scaled_covariance * (wall.weighted_position -
        wall.precision * (observation.dot(state_) + acceleration_offset));
      const Matrix6 residual = Matrix6::Identity() -
        wall.precision * scaled_covariance * observation.transpose();
      covariance_ = residual * covariance_ * residual.transpose() +
        wall.precision * scaled_covariance * scaled_covariance.transpose();
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
    state_ += gain * (value - observation.dot(state_));
    const Matrix6 residual = Matrix6::Identity() - gain * observation.transpose();
    covariance_ = residual * covariance_ * residual.transpose() + variance * gain * gain.transpose();
  }
  TunnelWallParameters parameters_;
  Vector6 state_;
  Matrix6 covariance_;
  std::uint64_t previous_imu_stamp_, previous_status_stamp_, previous_lidar_stamp_;
  Eigen::Vector3d tangent_, normal_;
};
}  // namespace localization
