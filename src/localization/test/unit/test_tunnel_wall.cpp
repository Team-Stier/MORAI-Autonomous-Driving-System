#include <localization/cores/tunnel_wall.hpp>
#include <localization/parameters/tunnel_wall.hpp>
#include <gtest/gtest.h>
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
  std::memcpy(cloud.data.data(), points.data(), cloud.row_step);
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
