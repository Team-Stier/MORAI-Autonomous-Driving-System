#include <gtest/gtest.h>
#include <localization/cores/full_ekf.hpp>
#include <localization/cores/tunnel_wall.hpp>
#include <localization/validators/tunnel_inputs.hpp>
#include <limits>

namespace {
struct Fixture {
  sensor_msgs::NavSatFix gps;
  sensor_msgs::Imu imu;
  interfaces::CompetitionVehicleStatus status;
  sensor_msgs::PointCloud2 cloud;
  Fixture() {
    gps.header.frame_id = "gps_link";
    gps.latitude = 37;
    gps.longitude = 127;
    gps.altitude = 30;
    imu.header.frame_id = "imu_link";
    status.header.frame_id = "base_link";
    status.velocity_x_mps = 6;
    cloud.header.frame_id = "base_link_level";
    cloud.height = 1;
    cloud.width = 1;
    cloud.point_step = 16;
    cloud.row_step = 16;
    cloud.data.resize(16, 0);
    for (const auto* name : {"x", "y", "z", "time"}) {
      sensor_msgs::PointField field;
      field.name = name;
      field.offset = cloud.fields.size() * sizeof(float);
      field.datatype = sensor_msgs::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    setTime(0);
    gps.header.stamp = imu.header.stamp;
  }
  void setTime(std::uint32_t nsec) {
    imu.header.stamp = ros::Time(1791638400, nsec);
    status.header.stamp = imu.header.stamp;
    cloud.header.stamp = imu.header.stamp;
  }
};
localization::EkfParameters fullParameters() {
  localization::EkfParameters value;
  value.yaw_offset_rad = 0;
  value.gps_position_noise = 0.01;
  return value;
}
localization::TunnelWallParameters tunnelParameters() {
  localization::TunnelWallParameters value;
  value.yaw_offset_rad = 0;
  return value;
}
}  // namespace

TEST(TunnelWall, LeveledWallPointsDoNotReceiveTheBodyTiltAgain) {
  Fixture fixture;
  fixture.status.roll_rad = 0.4;
  fixture.status.pitch_rad = -0.3;
  fixture.status.velocity_x_mps = 0;
  fixture.cloud.width = 20;
  fixture.cloud.row_step = fixture.cloud.width * fixture.cloud.point_step;
  fixture.cloud.data.resize(fixture.cloud.row_step);
  for (std::size_t i = 0; i < fixture.cloud.width; ++i) {
    const float point[] = {static_cast<float>(i) * 0.1f, 5.0f, 1.0f, 0.0f};
    std::memcpy(fixture.cloud.data.data() + i * fixture.cloud.point_step, point, sizeof(point));
  }
  localization::EkfState shared;
  shared.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  shared.previous_status_stamp = fixture.status.header.stamp.toNSec();
  localization::TunnelWall core(tunnelParameters(), shared);
  const auto result = core.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_NEAR(0, result.pose.pose.position.y, 1e-6);
}

TEST(TunnelWall, UsesImuAngularVelocityForPointDeskew) {
  Fixture fixture;
  fixture.status.yaw_rad = 0.05;
  fixture.status.angular_velocity_z_radps = -0.5;
  fixture.imu.angular_velocity.z = 0.5;
  fixture.cloud.width = 20;
  fixture.cloud.row_step = fixture.cloud.width * fixture.cloud.point_step;
  fixture.cloud.data.resize(fixture.cloud.row_step);
  for (std::size_t i = 0; i < fixture.cloud.width; ++i) {
    // At t=-0.1 s the body yaw was zero; all returns lie on map y=5.
    const float point[] = {static_cast<float>(i) * 0.1f, 5.0f, 1.0f, -0.1f};
    std::memcpy(fixture.cloud.data.data() + i * fixture.cloud.point_step, point, sizeof(point));
  }
  localization::EkfState shared;
  shared.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  shared.previous_status_stamp = fixture.status.header.stamp.toNSec();
  localization::TunnelWall core(tunnelParameters(), shared);
  const auto result = core.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_NEAR(0, result.pose.pose.position.y, 1e-6);
}

TEST(TunnelWall, CalibratesIntegrationTimeFromMeasuredAccelerationAndSpeedChange) {
  Fixture fixture;
  fixture.status.velocity_x_mps = 10;
  localization::EkfState shared;
  shared.state[3] = 10;
  shared.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  shared.previous_status_stamp = fixture.status.header.stamp.toNSec();
  localization::TunnelWall core(tunnelParameters(), shared);
  for (int i = 1; i <= 500; ++i) {
    const auto stamp = ros::Time(1791638400 + i / 50, (i % 50) * 20000000);
    fixture.imu.header.stamp = stamp;
    fixture.status.header.stamp = stamp;
    fixture.cloud.header.stamp = stamp;
    fixture.imu.linear_acceleration.x = 2;
    fixture.status.velocity_x_mps = 10 + i * 0.032;
    core.process(fixture.imu, fixture.status, fixture.cloud);
  }
  // Ten host seconds contain eight simulated seconds: 10*8 + 0.5*2*8^2.
  EXPECT_NEAR(144, shared.state[0], 3);
  EXPECT_NEAR(26, shared.state[3], 0.02);
}

TEST(TunnelWall, SharesStateCovarianceAndSampleStampsAcrossCoreSwitches) {
  Fixture fixture;
  localization::EkfState state;
  localization::FullEkf full(fullParameters(), state);
  localization::TunnelWall tunnel(tunnelParameters(), state);
  const auto first = full.process(fixture.gps, fixture.imu, fixture.status);
  EXPECT_TRUE(state.initialized);
  EXPECT_NEAR(first.pose.pose.position.z, 28.8, 1e-6);
  fixture.setTime(50000000);
  fixture.imu.linear_acceleration.x = 2;
  fixture.status.velocity_x_mps = 6.1;
  const auto entry = tunnel.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_NEAR(entry.pose.pose.position.x - first.pose.pose.position.x, 0.3025, 1e-6);
  EXPECT_NEAR(entry.pose.pose.position.y, first.pose.pose.position.y, 1e-6);
  EXPECT_NEAR(entry.pose.pose.position.z, 28.8, 1e-6);
  fixture.setTime(100000000);
  fixture.status.velocity_x_mps = 6.2;
  const auto last = tunnel.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_NEAR(last.pose.pose.position.x - first.pose.pose.position.x, 0.61, 1e-6);
  const auto covariance = state.covariance;
  const auto returned = full.process(fixture.gps, fixture.imu, fixture.status);
  const auto repeated = tunnel.process(fixture.imu, fixture.status, fixture.cloud);
  for (const auto& result : {returned, repeated}) {
    EXPECT_DOUBLE_EQ(result.pose.pose.position.x, last.pose.pose.position.x);
    EXPECT_DOUBLE_EQ(result.pose.pose.position.y, last.pose.pose.position.y);
    EXPECT_DOUBLE_EQ(result.pose.pose.position.z, last.pose.pose.position.z);
    EXPECT_DOUBLE_EQ(result.twist.twist.linear.x, last.twist.twist.linear.x);
    EXPECT_EQ(result.child_frame_id, "base_link");
    EXPECT_TRUE(result.header.stamp.isZero());
  }
  EXPECT_TRUE(state.covariance.isApprox(covariance, 0));
  EXPECT_EQ(state.previous_imu_stamp, fixture.imu.header.stamp.toNSec());
  EXPECT_EQ(state.previous_status_stamp, fixture.status.header.stamp.toNSec());
  EXPECT_EQ(state.previous_lidar_stamp, fixture.cloud.header.stamp.toNSec());
  EXPECT_EQ(state.previous_gps_stamp, fixture.gps.header.stamp.toNSec());
}

TEST(TunnelWall, ResumesGpsHeightCorrectionInSharedAbsoluteMapState) {
  Fixture fixture;
  localization::EkfState state;
  localization::FullEkf full(fullParameters(), state);
  localization::TunnelWall tunnel(tunnelParameters(), state);
  full.process(fixture.gps, fixture.imu, fixture.status);
  fixture.setTime(50000000);
  tunnel.process(fixture.imu, fixture.status, fixture.cloud);
  fixture.setTime(100000000);
  fixture.gps.header.stamp = fixture.imu.header.stamp;
  fixture.gps.altitude = 31;
  const auto result = full.process(fixture.gps, fixture.imu, fixture.status);
  EXPECT_NEAR(result.pose.pose.position.z, 29.8, 0.02);
  EXPECT_DOUBLE_EQ(result.pose.pose.position.z, state.state[2]);
  EXPECT_EQ(state.previous_gps_stamp, fixture.gps.header.stamp.toNSec());
}

TEST(TunnelWall, FreshGpsReturnPreservesMovingSharedStateAndDoesNotFuseSameStampTwice) {
  Fixture fixture;
  const auto gps_xy = localization::projectGps(fixture.gps.latitude, fixture.gps.longitude);
  localization::EkfState state;
  state.initialized = true;
  state.state << gps_xy[0] - 144, gps_xy[1], 28.8, 10, 0, 0;
  state.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  state.previous_status_stamp = fixture.status.header.stamp.toNSec();
  state.previous_gps_stamp = fixture.gps.header.stamp.toNSec();
  localization::FullEkf full(fullParameters(), state);
  localization::TunnelWall tunnel(tunnelParameters(), state);
  nav_msgs::Odometry last;
  for (int i = 1; i <= 500; ++i) {
    const auto stamp = ros::Time(1791638400 + i / 50, (i % 50) * 20000000);
    fixture.imu.header.stamp = stamp;
    fixture.status.header.stamp = stamp;
    fixture.cloud.header.stamp = stamp;
    fixture.imu.linear_acceleration.x = 2;
    fixture.status.velocity_x_mps = 10 + i * 0.032;
    last = tunnel.process(fixture.imu, fixture.status, fixture.cloud);
  }
  EXPECT_NEAR(gps_xy[0], last.pose.pose.position.x, 3);
  EXPECT_NEAR(26, last.twist.twist.linear.x, 0.02);
  fixture.gps.header.stamp = fixture.imu.header.stamp;
  const auto returned = full.process(fixture.gps, fixture.imu, fixture.status);
  EXPECT_LT(std::abs(returned.pose.pose.position.x - last.pose.pose.position.x), 3);
  EXPECT_NEAR(gps_xy[0], returned.pose.pose.position.x, 0.01);
  EXPECT_NEAR(gps_xy[1], returned.pose.pose.position.y, 1e-6);
  EXPECT_NEAR(28.8, returned.pose.pose.position.z, 1e-6);
  const auto returned_state = state.state;
  const auto returned_covariance = state.covariance;
  full.process(fixture.gps, fixture.imu, fixture.status);
  EXPECT_TRUE(state.state.isApprox(returned_state, 0));
  EXPECT_TRUE(state.covariance.isApprox(returned_covariance, 0));
  EXPECT_EQ(state.previous_gps_stamp, fixture.gps.header.stamp.toNSec());
}

TEST(TunnelWall, ForwardSpeedCorrectionPreservesUnobservedVerticalVelocity) {
  Fixture fixture;
  fixture.status.pitch_rad = 0.1;
  fixture.status.velocity_x_mps = 6;
  localization::EkfState state;
  state.covariance(5, 5) = 100;
  state.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  localization::TunnelWall tunnel(tunnelParameters(), state);
  tunnel.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_DOUBLE_EQ(0, state.state[5]);
  EXPECT_GT(state.state[3], 0);
}

TEST(TunnelInputs, RejectsNonfiniteAngularRatesAndAcceptsFiniteRates) {
  Fixture fixture;
  localization::TunnelInputs validator;
  node_pipeline::HealthLimits limits;
  std::vector<node_pipeline::Kind> issues;
  fixture.imu.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  validator(fixture.imu, limits, issues);
  ASSERT_EQ(1u, issues.size());
  EXPECT_EQ(node_pipeline::Kind::NaN, issues.front());
  issues.clear();
  fixture.imu.angular_velocity.x = 0;
  fixture.imu.angular_velocity.z = std::numeric_limits<double>::infinity();
  validator(fixture.imu, limits, issues);
  ASSERT_EQ(1u, issues.size());
  EXPECT_EQ(node_pipeline::Kind::Inf, issues.front());
  issues.clear();
  fixture.imu.angular_velocity.y = -0.5;
  fixture.imu.angular_velocity.z = 0.5;
  validator(fixture.imu, limits, issues);
  EXPECT_TRUE(issues.empty());
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
