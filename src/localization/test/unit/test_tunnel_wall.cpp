#include <localization/cores/tunnel_wall.hpp>
#include <localization/cores/full_ekf.hpp>
#include <localization/parameters/tunnel_wall.hpp>
#include <localization/validators/tunnel_wall.hpp>
#include <localization/policies/remove_invalid.hpp>
#include <node_pipeline/detail/message_traits.hpp>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

namespace {
using Vector = Eigen::Vector3d;
using Rotation = Eigen::Quaterniond;
Rotation attitude(double roll, double pitch, double yaw) {
  return Eigen::AngleAxisd(yaw, Vector::UnitZ()) *
         Eigen::AngleAxisd(pitch, Vector::UnitY()) *
         Eigen::AngleAxisd(roll, Vector::UnitX());
}
struct Fixture {
  localization::TunnelWallParameters parameters;
  sensor_msgs::Imu imu;
  interfaces::CompetitionVehicleStatus status;
  std::shared_ptr<localization::StateBank> bank = std::make_shared<localization::StateBank>();
  Fixture() {
    parameters.origin_x_m = -20;
    parameters.origin_y_m = 100;
    parameters.yaw_offset_rad = 0;
    bank->initialized = true;
    bank->state << -13, 100, 0, 0, 0, 0;
    bank->covariance *= 1000;
    bank->previous_imu_stamp = 1000000000;
    status.angular_velocity_x_radps = 0.01;
    status.angular_velocity_y_radps = 0.02;
    status.angular_velocity_z_radps = 0.03;
    stamp(1);
  }
  void stamp(double sec) {
    imu.header.stamp.fromSec(sec);
    status.header.stamp = imu.header.stamp;
    status.header.frame_id = "base_link";
    imu.header.frame_id = "imu_link";
    // IMU attitude disagrees deliberately; vehicle status owns attitude.
    imu.orientation = geometry_msgs::Quaternion{};
    imu.orientation.z = 1;
  }
  sensor_msgs::PointCloud2 walls(double lateral_m, double scan_sec = 1,
                               const Rotation& rotation = Rotation::Identity()) const {
    std::vector<Vector> points;
    const auto basis = attitude(0, 0, parameters.axis_yaw_rad);
    for (double wall : {parameters.left_wall_m, parameters.right_wall_m}) {
      for (int x = -16; x <= 16; ++x) {
        for (int z = 1; z <= 3; ++z)
          points.push_back(rotation.conjugate() *
            (basis * Vector(x, wall - lateral_m - parameters.wall_normal_z_ratio * z, z)));
      }
    }
    // Parallel obstacle faces and the floor must not move the wall estimate.
    for (int i = 0; i < 12; ++i) {
      points.push_back(rotation.conjugate() * (basis * Vector(i, 2, 1)));
      points.push_back(rotation.conjugate() * (basis * Vector(i, -2, 1)));
      points.push_back(rotation.conjugate() * (basis * Vector(i, 7, -0.3)));
    }
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp.fromSec(scan_sec);
    cloud.header.frame_id = "base_link";
    cloud.width = points.size();
    cloud.height = 1;
    cloud.point_step = 16;
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.is_dense = true;
    for (int axis = 0; axis < 4; ++axis) {
      sensor_msgs::PointField field;
      field.name = std::vector<std::string>{"x", "y", "z", "time"}[axis];
      field.offset = axis * 4;
      field.datatype = sensor_msgs::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    cloud.data.resize(cloud.row_step);
    for (std::size_t i = 0; i < points.size(); ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        const float value = points[i][axis];
        std::memcpy(cloud.data.data() + i * cloud.point_step + axis * 4, &value, 4);
      }
    }
    return cloud;
  }
};
}

TEST(TunnelWall, FitsWallsWithObstaclesAndPreservesLongitudinalAnchor) {
  Fixture f;
  localization::TunnelWall core(f.parameters, f.bank);
  const auto message = core.process(f.imu, f.status, f.walls(1.25));
  EXPECT_NEAR(message.pose.pose.position.x, -13, 1e-9);
  EXPECT_NEAR(message.pose.pose.position.y, 101.25, 0.002);
  EXPECT_EQ(message.child_frame_id, "base_link");
  EXPECT_TRUE(message.header.frame_id.empty());
  EXPECT_TRUE(message.header.stamp.isZero());
  EXPECT_DOUBLE_EQ(message.pose.pose.position.z, 0);
  for (double covariance : message.pose.covariance) EXPECT_DOUBLE_EQ(covariance, 0);
  for (double covariance : message.twist.covariance) EXPECT_DOUBLE_EQ(covariance, 0);
}

TEST(TunnelWall, UsesVehicleGtAttitudeAndAngularVelocity) {
  Fixture f;
  f.parameters.axis_yaw_rad = 0.6;
  f.status.roll_rad = 0.05;
  f.status.pitch_rad = -0.04;
  f.status.yaw_rad = f.parameters.axis_yaw_rad;
  const Rotation rotation = attitude(f.status.roll_rad, f.status.pitch_rad, f.status.yaw_rad);
  f.bank->state.head<3>() = Vector(-20, 100, 0) +
    attitude(0, 0, f.parameters.axis_yaw_rad) * Vector(7, 0, 0);
  localization::TunnelWall core(f.parameters, f.bank);
  const auto message = core.process(f.imu, f.status, f.walls(-0.75, 1, rotation));
  const Vector expected = Vector(-20, 100, 0) +
    attitude(0, 0, f.parameters.axis_yaw_rad) * Vector(7, -0.75, 0);
  EXPECT_NEAR(message.pose.pose.position.x, expected.x(), 0.002);
  EXPECT_NEAR(message.pose.pose.position.y, expected.y(), 0.002);
  const auto& q = message.pose.pose.orientation;
  EXPECT_NEAR(q.x, rotation.x(), 1e-9);
  EXPECT_NEAR(q.y, rotation.y(), 1e-9);
  EXPECT_NEAR(q.z, rotation.z(), 1e-9);
  EXPECT_NEAR(q.w, rotation.w(), 1e-9);
  EXPECT_FLOAT_EQ(message.twist.twist.angular.x, f.status.angular_velocity_x_radps);
  EXPECT_FLOAT_EQ(message.twist.twist.angular.y, f.status.angular_velocity_y_radps);
  EXPECT_FLOAT_EQ(message.twist.twist.angular.z, f.status.angular_velocity_z_radps);
}

TEST(TunnelWall, IntegratesGtSpeedWithoutDuplicatingPredictionOrScanUpdates) {
  Fixture f;
  f.status.velocity_x_mps = 4;
  f.bank->state[3] = 4;
  localization::TunnelWall core(f.parameters, f.bank);
  auto cloud = f.walls(0);
  core.process(f.imu, f.status, cloud);
  f.stamp(1.02);
  const auto moved = core.process(f.imu, f.status, cloud);
  const auto duplicate = core.process(f.imu, f.status, cloud);
  EXPECT_NEAR(moved.pose.pose.position.x, -12.92, 1e-6);
  EXPECT_DOUBLE_EQ(moved.pose.pose.position.x, duplicate.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(moved.pose.pose.position.y, duplicate.pose.pose.position.y);
  EXPECT_NEAR(moved.twist.twist.linear.x, 4, 1e-6);
}

TEST(TunnelWall, EstimatesLateralVelocityFromWallMotion) {
  Fixture f;
  // Isolate LiDAR velocity observation with an uncertain sideways rolling model.
  f.parameters.lateral_velocity_noise_mps = 1e6;
  f.status.angular_velocity_x_radps = f.status.angular_velocity_y_radps = 0;
  f.status.angular_velocity_z_radps = 0;
  f.status.velocity_x_mps = 5;
  f.bank->state[3] = 5;
  localization::TunnelWall core(f.parameters, f.bank);
  nav_msgs::Odometry message;
  for (int i = 0; i <= 100; ++i) {
    const double sec = 1 + i * 0.02;
    f.stamp(sec);
    // A new wall scan at 10 Hz, repeated by the 50 Hz pipeline.
    const double scan_sec = 1 + (i / 5) * 0.1;
    message = core.process(f.imu, f.status, f.walls(0.5 * (scan_sec - 1), scan_sec));
  }
  EXPECT_NEAR(message.pose.pose.position.x, -3, 0.002);
  EXPECT_NEAR(message.pose.pose.position.y, 101, 0.025);
  EXPECT_NEAR(message.twist.twist.linear.y, 0.5, 0.025);
}

TEST(TunnelWall, CannotObserveAnIncorrectLongitudinalSeedFromParallelWalls) {
  Fixture f;
  auto second_bank = std::make_shared<localization::StateBank>(*f.bank);
  second_bank->state[0] += 12;
  localization::TunnelWall first(f.parameters, f.bank), other(f.parameters, second_bank);
  const auto a = first.process(f.imu, f.status, f.walls(0.3));
  const auto b = other.process(f.imu, f.status, f.walls(0.3));
  EXPECT_NEAR(b.pose.pose.position.x - a.pose.pose.position.x, 12, 1e-9);
  EXPECT_DOUBLE_EQ(b.pose.pose.position.y, a.pose.pose.position.y);
}

TEST(TunnelWall, SharesPersistentBankStateAndClockAcrossSwitches) {
  Fixture f;
  localization::EkfParameters ekf_parameters;
  ekf_parameters.yaw_offset_rad = 0;
  auto bank = std::make_shared<localization::StateBank>();
  localization::FullEkf full_ekf(ekf_parameters, bank);
  auto shared_bank = bank;
  bank.reset();
  sensor_msgs::NavSatFix gps;
  gps.latitude = 37.0;
  gps.longitude = 127.0;
  gps.altitude = 10;
  gps.header.stamp = f.imu.header.stamp;
  f.status.velocity_x_mps = 3;
  const auto seeded = full_ekf.process(gps, f.imu, f.status);
  ASSERT_TRUE(shared_bank->initialized);
  EXPECT_DOUBLE_EQ(shared_bank->state[0], seeded.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(shared_bank->state[1], seeded.pose.pose.position.y);
  f.parameters.origin_y_m = seeded.pose.pose.position.y;
  localization::TunnelWall centered_wall(f.parameters, shared_bank);
  f.stamp(1.02);
  const auto tunnel = centered_wall.process(f.imu, f.status, f.walls(0, 1.02));
  EXPECT_NEAR(tunnel.pose.pose.position.x - seeded.pose.pose.position.x, 0.06, 1e-6);
  EXPECT_EQ(shared_bank->previous_imu_stamp, f.imu.header.stamp.toNSec());
  const auto recovered = full_ekf.process(gps, f.imu, f.status);
  EXPECT_DOUBLE_EQ(recovered.pose.pose.position.x, tunnel.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(recovered.pose.pose.position.y, tunnel.pose.pose.position.y);
}

TEST(TunnelWall, WallCorrectionDoesNotChangeStationFromCorrelatedGpsCovariance) {
  Fixture f;
  f.bank->covariance(0, 1) = f.bank->covariance(1, 0) = 500;
  localization::TunnelWall core(f.parameters, f.bank);
  const auto message = core.process(f.imu, f.status, f.walls(1.25));
  EXPECT_NEAR(message.pose.pose.position.x, -13, 1e-9);
  EXPECT_NEAR(message.pose.pose.position.y, 101.25, 0.002);
}

TEST(TunnelWall, UsesOneWallConsensusWhenTheOtherWallIsOccluded) {
  Fixture f;
  auto cloud = f.walls(1.25);
  cloud.width = 99; // Retain the left wall; the fixture generated it first.
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  localization::TunnelWall core(f.parameters, f.bank);
  const auto message = core.process(f.imu, f.status, cloud);
  EXPECT_NEAR(message.pose.pose.position.x, -13, 1e-9);
  EXPECT_NEAR(message.pose.pose.position.y, 101.25, 0.002);
}

TEST(TunnelWall, FloorPointsSupplyNoWallInformation) {
  Fixture f;
  auto cloud = f.walls(0);
  for (std::size_t i = 0; i < cloud.width; ++i) {
    const float floor_z = -0.3;
    std::memcpy(cloud.data.data() + i * cloud.point_step + 8, &floor_z, 4);
  }
  localization::TunnelWall core(f.parameters, f.bank);
  const auto message = core.process(f.imu, f.status, cloud);
  EXPECT_DOUBLE_EQ(message.pose.pose.position.x, -13);
  EXPECT_DOUBLE_EQ(message.pose.pose.position.y, 100);
}

TEST(TunnelWall, FitsSurveyedInclinedWallPlanes) {
  Fixture f;
  f.parameters.wall_normal_z_ratio = 0.023916318426;
  localization::TunnelWall core(f.parameters, f.bank);
  const auto message = core.process(f.imu, f.status, f.walls(1.25));
  EXPECT_NEAR(message.pose.pose.position.x, -13, 1e-9);
  EXPECT_NEAR(message.pose.pose.position.y, 101.25, 0.002);
}

TEST(TunnelWall, AlignsDelayedVehicleSpeedToImuPredictionTime) {
  Fixture f;
  f.status.angular_velocity_x_radps = f.status.angular_velocity_y_radps = 0;
  f.status.angular_velocity_z_radps = 0;
  f.bank->state[3] = 5;
  f.status.velocity_x_mps = 5;
  f.imu.linear_acceleration.x = 2;
  f.imu.header.stamp.fromSec(1.1);
  // The selected GT observation predates the current IMU by 100 ms.
  f.status.header.stamp.fromSec(1);
  localization::TunnelWall core(f.parameters, f.bank);
  const auto result = core.process(f.imu, f.status, f.walls(0));
  EXPECT_NEAR(result.twist.twist.linear.x, 5.2, 1e-6);
  EXPECT_NEAR(result.pose.pose.position.x, -12.49, 1e-5);
}

TEST(TunnelWall, DeskewsIndividualPointsDuringRotationAndLateralMotion) {
  Fixture f;
  // The synthetic scan intentionally includes body sideways motion.
  f.parameters.lateral_velocity_noise_mps = 1e6;
  f.status.angular_velocity_x_radps = f.status.angular_velocity_y_radps = 0;
  f.status.angular_velocity_z_radps = 0.3;
  f.status.velocity_x_mps = 5;
  f.bank->state.tail<3>() = Vector(5, 0.6, 0);
  auto cloud = f.walls(1.25);
  for (std::size_t i = 0; i < cloud.width; ++i) {
    float xyz[3];
    std::memcpy(xyz, cloud.data.data() + i * cloud.point_step, sizeof(xyz));
    const float dt = -0.12f + 0.04f * static_cast<float>(i % 5) / 4;
    const Vector point = attitude(0, 0, 0.3 * dt).conjugate() *
      (Vector(xyz[0], xyz[1], xyz[2]) - Vector(5, 0.6, 0) * dt);
    for (int axis = 0; axis < 3; ++axis) {
      const float value = point[axis];
      std::memcpy(cloud.data.data() + i * cloud.point_step + axis * 4, &value, 4);
    }
    std::memcpy(cloud.data.data() + i * cloud.point_step + 12, &dt, 4);
  }
  localization::TunnelWall core(f.parameters, f.bank);
  const auto result = core.process(f.imu, f.status, cloud);
  EXPECT_NEAR(result.pose.pose.position.x, -13, 1e-9);
  EXPECT_NEAR(result.pose.pose.position.y, 101.25, 0.002);
}

TEST(TunnelWall, ExcludesParallelStructureBeyondSurveyedWallEnds) {
  Fixture f;
  // The fixture station is 7 m; all points at local x > 3 lie beyond this wall.
  f.parameters.wall_min_station_m = -20;
  f.parameters.wall_max_station_m = 10;
  auto cloud = f.walls(1.25);
  for (std::size_t i = 0; i < cloud.width; ++i) {
    float x;
    std::memcpy(&x, cloud.data.data() + i * cloud.point_step, 4);
    if (x <= 3) x = 4; // Put every apparent wall beyond the surveyed endpoint.
    std::memcpy(cloud.data.data() + i * cloud.point_step, &x, 4);
  }
  localization::TunnelWall core(f.parameters, f.bank);
  const auto result = core.process(f.imu, f.status, cloud);
  EXPECT_DOUBLE_EQ(result.pose.pose.position.x, -13);
  EXPECT_DOUBLE_EQ(result.pose.pose.position.y, 100);
}

TEST(TunnelWall, ConstrainsRearAxleSidewaysVelocityBeforeWallsAreVisible) {
  Fixture f;
  f.status.yaw_rad = 0.2;
  f.status.velocity_x_mps = 5;
  f.bank->state.tail<3>() = Vector(5, 0.3, 0);
  auto cloud = f.walls(0);
  for (std::size_t i = 0; i < cloud.width; ++i) {
    const float floor_z = -0.3;
    std::memcpy(cloud.data.data() + i * cloud.point_step + 8, &floor_z, 4);
  }
  localization::TunnelWall core(f.parameters, f.bank);
  const auto result = core.process(f.imu, f.status, cloud);
  EXPECT_NEAR(result.twist.twist.linear.y, 0, 0.05);
  EXPECT_DOUBLE_EQ(result.pose.pose.position.x, -13);
  EXPECT_DOUBLE_EQ(result.pose.pose.position.y, 100);
}

namespace {
namespace np = node_pipeline;
namespace runtime = node_pipeline::detail;
struct TunnelFixtureParameters {
  localization::TunnelWallParameters operator()(const np::Parameters&) const {
    localization::TunnelWallParameters parameters;
    parameters.origin_x_m = -20;
    parameters.origin_y_m = 100;
    parameters.yaw_offset_rad = 0;
    return parameters;
  }
};
void registerTunnel(runtime::Engine& engine, const Fixture& fixture) {
  runtime::InputSpec gps{"gps"}, imu{"imu"}, status{"competition_status"}, cloud{"lidar"};
  gps.limits.frame = "gps_link";
  imu.limits.frame = "imu_link";
  status.limits.frame = cloud.limits.frame = "base_link";
  engine.input<sensor_msgs::NavSatFix>(gps);
  engine.input<sensor_msgs::Imu>(imu);
  engine.input<interfaces::CompetitionVehicleStatus>(status);
  engine.input<sensor_msgs::PointCloud2>(cloud);
  runtime::CoreSpec core;
  core.name = "tunnel_wall.hpp";
  core.stage = "stage1";
  core.inputs = {"imu", "competition_status", "lidar"};
  core.outputs = {{"ego", ""}};
  engine.core<localization::TunnelWall, TunnelFixtureParameters,
    localization::validators::TunnelWall, localization::policies::RemoveInvalid,
    localization::StateBank>(core, np::Parameters{}, fixture.bank);
  runtime::StageSpec stage;
  stage.name = "stage1";
  stage.section_core_map[0] = "tunnel_wall.hpp";
  engine.stage(stage);
}
void pushTunnel(runtime::Engine& engine, const Fixture& fixture,
                const sensor_msgs::PointCloud2& cloud, int missing = -1) {
  const np::Clocks clocks{1000000000, 1000000000};
  if (missing != 0)
    engine.push("imu", std::make_shared<const sensor_msgs::Imu>(fixture.imu), clocks, 1000000000);
  if (missing != 1)
    engine.push("competition_status", std::make_shared<const interfaces::CompetitionVehicleStatus>(
      fixture.status), clocks, 1000000000);
  if (missing != 2)
    engine.push("lidar", std::make_shared<const sensor_msgs::PointCloud2>(cloud), clocks, 1000000000);
}
}

TEST(TunnelWall, PipelineRequiresImuStatusAndCloudButNotGps) {
  Fixture f;
  for (const int missing : {0, 1, 2}) {
    runtime::Engine engine;
    registerTunnel(engine, f);
    pushTunnel(engine, f, f.walls(0.25), missing);
    const auto before = f.bank->state;
    EXPECT_EQ(np::ProcessStatus::Rejected, engine.process({1000000000, 1000000000}).status);
    EXPECT_TRUE(before.isApprox(f.bank->state, 0));
  }
  runtime::Engine engine;
  registerTunnel(engine, f);
  pushTunnel(engine, f, f.walls(0.25));
  ASSERT_EQ(np::ProcessStatus::Published, engine.process({1000000000, 1000000000}).status);
  EXPECT_NEAR(engine.result<nav_msgs::Odometry>("ego").pose.pose.position.y, 100.25, 0.002);
}

TEST(TunnelWall, PipelineRejectsInvalidCloudContract) {
  Fixture f;
  const auto cloud = f.walls(0);
  {
    runtime::Engine engine;
    registerTunnel(engine, f);
    pushTunnel(engine, f, cloud);
    EXPECT_EQ(np::ProcessStatus::Published, engine.process({1000000000, 1000000000}).status);
  }
  for (int failure = 0; failure < 4; ++failure) {
    auto invalid = cloud;
    if (failure == 0) invalid.fields.pop_back();
    if (failure == 1) invalid.data.resize(1);
    if (failure == 2) invalid.is_bigendian = true;
    if (failure == 3) {
      const float value = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(invalid.data.data(), &value, sizeof(value));
    }
    runtime::Engine engine;
    registerTunnel(engine, f);
    pushTunnel(engine, f, invalid);
    const auto before = f.bank->state;
    EXPECT_EQ(np::ProcessStatus::Rejected, engine.process({1000000000, 1000000000}).status);
    EXPECT_TRUE(before.isApprox(f.bank->state, 0));
  }
}

TEST(TunnelWall, PipelineRemovesStaleInputs) {
  Fixture fixture;
  runtime::Engine engine;
  registerTunnel(engine, fixture);
  pushTunnel(engine, fixture, fixture.walls(0));
  EXPECT_EQ(np::ProcessStatus::Rejected, engine.process({2000000000, 1000000000}).status);
  const auto& issues = engine.health().issues("imu");
  EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(), np::Kind::Missing));
}
