#include <localization/cores/full_ekf.hpp>
#include <localization/cores/tunnel_wall.hpp>
#include <localization/parameters/full_ekf.hpp>
#include <localization/parameters/tunnel_wall.hpp>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace {
struct Point { float x, y, z, time; };

sensor_msgs::PointCloud2 makeCloud(const std::vector<Point>& points, double stamp_sec) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp.fromSec(stamp_sec);
  cloud.header.frame_id = "base_link_level";
  const char* names[] = {"x", "y", "z", "time"};
  for (std::uint32_t i = 0; i < 4; ++i) {
    sensor_msgs::PointField field;
    field.name = names[i];
    field.offset = i * sizeof(float);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.height = 1;
  cloud.width = points.size();
  cloud.point_step = sizeof(Point);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  if (!points.empty()) std::memcpy(cloud.data.data(), points.data(), cloud.row_step);
  return cloud;
}

// Wall faces at map y=+6/-6 seen from a vehicle at true lateral y_true_m.
void addWalls(std::vector<Point>& points, double y_true_m, bool left = true) {
  for (double x = -15; x <= 15; x += 0.25)
    for (double z : {0.5, 1.0, 1.5, 2.0}) {
      if (left) points.push_back({float(x), float(6.0 - y_true_m), float(z), 0});
      points.push_back({float(x), float(-6.0 - y_true_m), float(z), 0});
    }
}

// Dense obstacle side face parallel to the walls at body lateral offset y_body_m.
void addObstacle(std::vector<Point>& points, double y_body_m) {
  for (double x = 1; x <= 5; x += 0.02)
    for (double z : {0.5, 0.7, 0.9, 1.1, 1.3, 1.5})
      points.push_back({float(x), float(y_body_m), float(z), 0});
}

localization::TunnelWallParameters parameters() {
  localization::TunnelWallParameters config;
  config.left_wall_m = 6.0;
  config.right_wall_m = -6.0;
  return config;
}

// Runs one tunnel step at t=1 s with the estimate at y=0 and 10 m/s along +x.
double lateralAfterScan(const std::vector<Point>& points) {
  localization::EkfState shared;
  shared.initialized = true;
  shared.state << 0, 0, 0, 10, 0, 0;
  shared.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;
  shared.previous_imu_stamp = ros::Time(0.98).toNSec();
  localization::TunnelWall core(parameters(), shared);
  sensor_msgs::Imu imu;
  imu.header.stamp.fromSec(1.0);
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromSec(1.0);
  status.yaw_rad = -parameters().yaw_offset_rad;
  status.velocity_x_mps = 10;
  return core.process(imu, status, makeCloud(points, 1.0)).pose.pose.position.y;
}
}  // namespace

TEST(TunnelWall, CleanWallsCorrectLateralPosition) {
  std::vector<Point> points;
  addWalls(points, 0.2);
  EXPECT_NEAR(lateralAfterScan(points), 0.2, 0.02);
}

TEST(TunnelWall, DenseObstacleFaceOutsideGateIsIgnored) {
  std::vector<Point> points;
  addWalls(points, 0.2);
  addObstacle(points, 2.0);  // implies y=4 m on the left side
  EXPECT_NEAR(lateralAfterScan(points), 0.2, 0.02);
}

TEST(TunnelWall, DisagreeingSideNearWallFallsBackToNearerSide) {
  std::vector<Point> points;
  addWalls(points, 0.0);
  addObstacle(points, 5.5);  // denser than the left wall, implies y=0.5 m
  EXPECT_NEAR(lateralAfterScan(points), 0.0, 0.02);
}

TEST(TunnelWall, AgreeingWallsRecoverOffsetBeyondGate) {
  std::vector<Point> points;
  addWalls(points, 1.0);  // 1 m from the estimate: outside the 0.6 m gate
  EXPECT_NEAR(lateralAfterScan(points), 1.0, 0.02);
}

TEST(TunnelWall, SingleWallStillCorrects) {
  std::vector<Point> points;
  addWalls(points, 0.2, false);
  EXPECT_NEAR(lateralAfterScan(points), 0.2, 0.02);
}

TEST(TunnelWall, RepositoryParametersLoad) {
  const auto reader = node_pipeline::ParamReader::fromFile(
    std::string(NP_PACKAGE_DIR) + "/config/localization/parameters/tunnel_wall.yaml", NP_PACKAGE_DIR);
  const auto config = localization::loadTunnelWallParameters(reader);
  EXPECT_DOUBLE_EQ(config.wall_gate_m, 0.6);
  EXPECT_DOUBLE_EQ(config.wall_gate_sigma, 4.0);
  EXPECT_DOUBLE_EQ(config.wall_side_agreement_m, 0.3);
}

namespace {
node_pipeline::ParamReader repositoryParameters(const std::string& name) {
  return node_pipeline::ParamReader::fromFile(
    std::string(NP_PACKAGE_DIR) + "/config/localization/parameters/" + name + ".yaml", NP_PACKAGE_DIR);
}

// Newton inverse of projectGps for a map XY.
std::array<double, 2> latLonAt(double x_m, double y_m) {
  double latitude = 37.25, longitude = 126.77;
  for (int i = 0; i < 8; ++i) {
    const double step = 1e-6;
    const auto p = localization::projectGps(latitude, longitude);
    const auto p_lat = localization::projectGps(latitude + step, longitude);
    const auto p_lon = localization::projectGps(latitude, longitude + step);
    const double a = (p_lat[0] - p[0]) / step, b = (p_lon[0] - p[0]) / step;
    const double c = (p_lat[1] - p[1]) / step, d = (p_lon[1] - p[1]) / step;
    const double ex = x_m - p[0], ey = y_m - p[1], det = a * d - b * c;
    latitude += (d * ex - b * ey) / det;
    longitude += (a * ey - c * ex) / det;
  }
  return {{latitude, longitude}};
}
}  // namespace

// Section 14 -> 15 while driving: TunnelWall continues the Bank FullEkf left, through the
// surveyed tunnel, and FullEkf resumes on GPS recovery without a position jump.
TEST(TunnelWall, HandoverFromFullEkfWhileDrivingAndBackOnGpsRecovery) {
  const auto tunnel_config = localization::loadTunnelWallParameters(repositoryParameters("tunnel_wall"));
  const auto ekf_config = localization::loadFullEkfParameters(repositoryParameters("full_ekf"));
  localization::EkfState shared;
  localization::FullEkf ekf(ekf_config, shared);
  localization::TunnelWall tunnel(tunnel_config, shared);
  const Eigen::Vector2d origin(tunnel_config.origin_x_m, tunnel_config.origin_y_m);
  const Eigen::Vector2d tangent(std::cos(tunnel_config.axis_yaw_rad), std::sin(tunnel_config.axis_yaw_rad));
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  const double speed_mps = 15, lateral_m = 0.3, height_m = 28.4, dt_sec = 0.02;
  const auto station = [&](int k) { return -90 + speed_mps * dt_sec * k; };
  const auto truth = [&](int k) { return Eigen::Vector2d(origin + tangent * station(k) + normal * lateral_m); };

  double max_error_m = 0, recovery_jump_m = -1;
  Eigen::Vector2d previous(0, 0);
  bool previous_tunnel = false;
  for (int k = 0; station(k) < tunnel_config.wall_max_station_m + 5; ++k) {
    const double stamp_sec = 100 + dt_sec * k;
    sensor_msgs::Imu imu;
    imu.header.stamp.fromSec(stamp_sec);
    interfaces::CompetitionVehicleStatus status;
    status.header.stamp.fromSec(stamp_sec);
    status.yaw_rad = tunnel_config.axis_yaw_rad - tunnel_config.yaw_offset_rad;
    status.velocity_x_mps = speed_mps;
    // Section 15 starts at the checkpoint ~20 m before the portal; GPS returns after the exit.
    const bool in_tunnel = station(k) >= -75 && station(k) <= tunnel_config.wall_max_station_m + 2;
    nav_msgs::Odometry ego;
    if (in_tunnel) {
      // 10 Hz scans keep their own stamp and pose.
      const int scan_k = k - k % 5;
      std::vector<Point> points;
      for (double x = -15; x <= 15; x += 0.25) {
        const double point_station = station(scan_k) + x;
        if (point_station < tunnel_config.wall_min_station_m || point_station > tunnel_config.wall_max_station_m)
          continue;
        for (double z : {0.5, 1.0, 1.5, 2.0}) {
          const double slope_m = tunnel_config.wall_normal_z_ratio * z;
          points.push_back({float(x), float(tunnel_config.left_wall_m - lateral_m - slope_m), float(z), 0});
          points.push_back({float(x), float(tunnel_config.right_wall_m - lateral_m - slope_m), float(z), 0});
        }
      }
      ego = tunnel.process(imu, status, makeCloud(points, 100 + dt_sec * scan_k));
    } else {
      sensor_msgs::NavSatFix gps;
      gps.header.stamp.fromSec(stamp_sec);
      const auto position = truth(k);
      const auto lat_lon = latLonAt(position.x(), position.y());
      gps.latitude = lat_lon[0];
      gps.longitude = lat_lon[1];
      gps.altitude = height_m + ekf_config.gps_height_m;
      ego = ekf.process(gps, imu, status);
    }
    const Eigen::Vector2d position(ego.pose.pose.position.x, ego.pose.pose.position.y);
    if (k > 0 && !in_tunnel && previous_tunnel)
      recovery_jump_m = (position - previous - tangent * speed_mps * dt_sec).norm();
    if (k >= 25) max_error_m = std::max(max_error_m, (position - truth(k)).norm());
    if (in_tunnel && !previous_tunnel) {
      // First TunnelWall call: the Bank state predicted one IMU step, at speed.
      EXPECT_LT((position - previous - tangent * speed_mps * dt_sec).norm(), 0.01);
      EXPECT_NEAR(ego.twist.twist.linear.x, speed_mps, 0.01);
    }
    previous = position;
    previous_tunnel = in_tunnel;
  }
  EXPECT_LT(max_error_m, 0.05);
  EXPECT_GE(recovery_jump_m, 0);
  EXPECT_LT(recovery_jump_m, 0.05);
}
