#include <gtest/gtest.h>
#include <localization/cores/full_ekf.hpp>
#include <localization/cores/tunnel_wall.hpp>

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
  shared.initialized = true;
  shared.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  shared.previous_status_stamp = fixture.status.header.stamp.toNSec();
  localization::TunnelWall core(tunnelParameters(), shared);
  const auto result = core.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_NEAR(0, result.pose.pose.position.y, 1e-6);
}

namespace {
// 20 wall points 5 m to the left of the vehicle (tunnelParameters: axis along x, walls at +-5 m from the origin).
void leftWall(Fixture& fixture) {
  fixture.status.velocity_x_mps = 0;
  fixture.cloud.width = 20;
  fixture.cloud.row_step = fixture.cloud.width * fixture.cloud.point_step;
  fixture.cloud.data.resize(fixture.cloud.row_step);
  for (std::size_t i = 0; i < fixture.cloud.width; ++i) {
    const float point[] = {static_cast<float>(i) * 0.1f, 5.0f, 1.0f, 0.0f};
    std::memcpy(fixture.cloud.data.data() + i * fixture.cloud.point_step, point, sizeof(point));
  }
}
}  // namespace

TEST(TunnelWall, RefusesToRunBeforeFullEkfInitialisesTheSharedState) {
  // Started (or restarted) while GPS is missing: zero state and zero stamps would integrate dt = epoch seconds.
  Fixture fixture;
  fixture.imu.linear_acceleration.x = 0.1;
  localization::EkfState shared;
  localization::TunnelWall core(tunnelParameters(), shared);
  EXPECT_THROW(core.process(fixture.imu, fixture.status, fixture.cloud), std::runtime_error);
  EXPECT_TRUE(shared.state.isZero());
  EXPECT_EQ(shared.previous_imu_stamp, 0u);
}

TEST(TunnelWall, NoWallPullOutsideTheTunnelCorridor) {
  // 200 m beside the axis but inside the station window: the walls seen there are not the tunnel's.
  Fixture fixture;
  leftWall(fixture);
  localization::EkfState shared;
  shared.initialized = true;
  shared.state[1] = 200;
  shared.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  shared.previous_status_stamp = fixture.status.header.stamp.toNSec();
  localization::TunnelWall core(tunnelParameters(), shared);
  const auto outside = core.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_DOUBLE_EQ(outside.pose.pose.position.y, 200);
  EXPECT_EQ(shared.previous_lidar_stamp, fixture.cloud.header.stamp.toNSec());   // the cloud is consumed
  // Inside the corridor the same walls still correct the lateral position (1 m off -> pulled back to the axis).
  localization::EkfState inside_state;
  inside_state.initialized = true;
  inside_state.state[1] = 1;
  inside_state.previous_imu_stamp = shared.previous_imu_stamp;
  inside_state.previous_status_stamp = shared.previous_status_stamp;
  localization::TunnelWall inside_core(tunnelParameters(), inside_state);
  const auto inside = inside_core.process(fixture.imu, fixture.status, fixture.cloud);
  EXPECT_LT(std::abs(inside.pose.pose.position.y), 0.1);
}

TEST(TunnelWall, SkipsPredictionAcrossReversedOrLongImuGaps) {
  Fixture fixture;
  fixture.status.velocity_x_mps = 6;
  localization::EkfState shared;
  shared.initialized = true;
  shared.state[3] = 6;
  shared.previous_imu_stamp = fixture.imu.header.stamp.toNSec();
  localization::TunnelWall core(tunnelParameters(), shared);
  fixture.imu.header.stamp = ros::Time(1791638405, 0);                 // 5 s gap: no core ran, or a stall
  EXPECT_DOUBLE_EQ(core.process(fixture.imu, fixture.status, fixture.cloud).pose.pose.position.x, 0);
  fixture.imu.header.stamp = ros::Time(1791638405, 50000000);          // the next normal step moves on
  EXPECT_NEAR(core.process(fixture.imu, fixture.status, fixture.cloud).pose.pose.position.x, 0.3, 1e-6);
  fixture.imu.header.stamp = ros::Time(0, 0);                          // reversed (sim time not started)
  EXPECT_NEAR(core.process(fixture.imu, fixture.status, fixture.cloud).pose.pose.position.x, 0.3, 1e-6);
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

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
