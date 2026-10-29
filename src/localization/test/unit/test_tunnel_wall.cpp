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

// Section 15 while driving: FullEkf on GPS up to the portal, the last GPS sample assembled until it
// is stale, TunnelWall on the Bank FullEkf left, and FullEkf again on GPS recovery, without jumps.
TEST(TunnelWall, HandoverFromFullEkfAtPortalAndBackOnGpsRecovery) {
  const auto tunnel_config = localization::loadTunnelWallParameters(repositoryParameters("tunnel_wall"));
  const auto ekf_config = localization::loadFullEkfParameters(repositoryParameters("full_ekf"));
  localization::EkfState shared;
  localization::FullEkf ekf(ekf_config, shared);
  localization::TunnelWall tunnel(tunnel_config, shared);
  const Eigen::Vector2d origin(tunnel_config.origin_x_m, tunnel_config.origin_y_m);
  const Eigen::Vector2d tangent(std::cos(tunnel_config.axis_yaw_rad), std::sin(tunnel_config.axis_yaw_rad));
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  const double speed_mps = 15, lateral_m = 0.3, height_m = 28.4, dt_sec = 0.02;
  const int stale_frames = 5;  // gps stale_after_sec 0.1 at 50 Hz
  const auto station = [&](int k) { return -90 + speed_mps * dt_sec * k; };
  const auto truth = [&](int k) { return Eigen::Vector2d(origin + tangent * station(k) + normal * lateral_m); };

  double max_error_m = 0, max_step_m = 0, recovery_jump_m = -1;
  Eigen::Vector2d previous(0, 0);
  int shadow_k = -1;
  bool previous_tunnel = false;
  for (int k = 0; station(k) < tunnel_config.wall_max_station_m + 5; ++k) {
    const double stamp_sec = 100 + dt_sec * k;
    sensor_msgs::Imu imu;
    imu.header.stamp.fromSec(stamp_sec);
    interfaces::CompetitionVehicleStatus status;
    status.header.stamp.fromSec(stamp_sec);
    status.yaw_rad = tunnel_config.axis_yaw_rad - tunnel_config.yaw_offset_rad;
    status.velocity_x_mps = speed_mps;
    const bool shadow = station(k) >= tunnel_config.wall_min_station_m &&
      station(k) <= tunnel_config.wall_max_station_m + 2;
    if (shadow && shadow_k < 0) shadow_k = k;
    const bool in_tunnel = shadow && k - shadow_k >= stale_frames;
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
      const int fix_k = shadow ? shadow_k - 1 : k;  // no new fix inside the portal
      sensor_msgs::NavSatFix gps;
      gps.header.stamp.fromSec(100 + dt_sec * fix_k);
      const auto position = truth(fix_k);
      const auto lat_lon = latLonAt(position.x(), position.y());
      gps.latitude = lat_lon[0];
      gps.longitude = lat_lon[1];
      gps.altitude = height_m + ekf_config.gps_height_m;
      ego = ekf.process(gps, imu, status);
    }
    const Eigen::Vector2d position(ego.pose.pose.position.x, ego.pose.pose.position.y);
    const double step_m = (position - previous - tangent * speed_mps * dt_sec).norm();
    if (k > 0 && !in_tunnel && previous_tunnel) recovery_jump_m = step_m;
    else if (k > 0 && shadow) max_step_m = std::max(max_step_m, step_m);
    if (k >= 25) max_error_m = std::max(max_error_m, (position - truth(k)).norm());
    if (in_tunnel && !previous_tunnel) EXPECT_NEAR(ego.twist.twist.linear.x, speed_mps, 0.01);
    previous = position;
    previous_tunnel = in_tunnel;
  }
  EXPECT_LT(max_step_m, 0.01);  // stale GPS frames and the first TunnelWall call continue the motion
  EXPECT_LT(max_error_m, 0.05);
  EXPECT_GE(recovery_jump_m, 0);
  EXPECT_LT(recovery_jump_m, 0.05);
}

// Checkpoint return during section 15: stuck inside the tunnel, the vehicle is placed back at the
// section-15 checkpoint outside the tunnel where GPS is valid; FullEkf relocates from the stale Bank.
TEST(TunnelWall, ReturnToSectionStartRelocatesOnGps) {
  const auto tunnel_config = localization::loadTunnelWallParameters(repositoryParameters("tunnel_wall"));
  const auto ekf_config = localization::loadFullEkfParameters(repositoryParameters("full_ekf"));
  localization::EkfState shared;
  localization::FullEkf ekf(ekf_config, shared);
  localization::TunnelWall tunnel(tunnel_config, shared);
  const Eigen::Vector2d origin(tunnel_config.origin_x_m, tunnel_config.origin_y_m);
  const Eigen::Vector2d tangent(std::cos(tunnel_config.axis_yaw_rad), std::sin(tunnel_config.axis_yaw_rad));
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  const Eigen::Vector2d checkpoint(43.761510782991536, -476.41094469698146);
  const double checkpoint_yaw_rad = -137.400207519 * M_PI / 180;
  const double height_m = 28.4, dt_sec = 0.02;
  Eigen::Vector2d truth = origin + tangent * -90;
  double speed_mps = 15, yaw_rad = tunnel_config.axis_yaw_rad;
  int converged_k = -1, return_k = -1;
  for (int k = 0; k < 900; ++k) {
    const double stamp_sec = 100 + dt_sec * k;
    const double station = tangent.dot(truth - origin);
    if (station >= 20 && speed_mps > 0) speed_mps = 0;   // stuck after a collision
    if (k == 600) {                                    // return: placed at rest on the checkpoint
      truth = checkpoint;
      yaw_rad = checkpoint_yaw_rad;
      return_k = k;
    }
    sensor_msgs::Imu imu;
    imu.header.stamp.fromSec(stamp_sec);
    interfaces::CompetitionVehicleStatus status;
    status.header.stamp.fromSec(stamp_sec);
    status.yaw_rad = yaw_rad - tunnel_config.yaw_offset_rad;
    status.velocity_x_mps = speed_mps;
    const bool gps_shadow = return_k < 0 && station >= tunnel_config.wall_min_station_m;
    nav_msgs::Odometry ego;
    if (gps_shadow) {
      std::vector<Point> points;
      for (double x = -15; x <= 15; x += 0.25) {
        const double point_station = station + x;
        if (point_station > tunnel_config.wall_max_station_m) continue;
        for (double z : {0.5, 1.0, 1.5, 2.0}) {
          const double lateral_m = normal.dot(truth - origin), slope_m = tunnel_config.wall_normal_z_ratio * z;
          points.push_back({float(x), float(tunnel_config.left_wall_m - lateral_m - slope_m), float(z), 0});
          points.push_back({float(x), float(tunnel_config.right_wall_m - lateral_m - slope_m), float(z), 0});
        }
      }
      ego = tunnel.process(imu, status, makeCloud(points, stamp_sec));
    } else {
      sensor_msgs::NavSatFix gps;
      const int fix_k = k - k % 2;   // 25 Hz
      gps.header.stamp.fromSec(100 + dt_sec * fix_k);
      const auto lat_lon = latLonAt(truth.x(), truth.y());
      gps.latitude = lat_lon[0];
      gps.longitude = lat_lon[1];
      gps.altitude = height_m + ekf_config.gps_height_m;
      ego = ekf.process(gps, imu, status);
    }
    const Eigen::Vector2d position(ego.pose.pose.position.x, ego.pose.pose.position.y);
    if (return_k >= 0) {
      const double error_m = (position - truth).norm();
      if (error_m < 0.1 && converged_k < 0) converged_k = k;
      if (error_m >= 0.1) converged_k = -1;
    }
    truth += speed_mps * dt_sec * Eigen::Vector2d(std::cos(yaw_rad), std::sin(yaw_rad));
  }
  EXPECT_GE(converged_k, 0);
  EXPECT_LT((converged_k - return_k) * dt_sec, 4.0);
}
