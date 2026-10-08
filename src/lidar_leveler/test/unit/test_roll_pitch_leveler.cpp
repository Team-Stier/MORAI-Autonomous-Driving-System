#include <lidar_leveler/cores/roll_pitch_leveler.hpp>
#include <lidar_leveler/cores/pass_through.hpp>
#include <lidar_leveler/validators/leveling_inputs.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

namespace {
using Point = std::array<double, 3>;
const double kHalfPi = std::acos(-1.0) / 2.0;

sensor_msgs::PointCloud2 makeCloud(const std::vector<Point>& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 7;
  cloud.header.stamp.fromNSec(1000000000);
  cloud.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32,
    "ring", 1, sensor_msgs::PointField::UINT16);
  cloud.height = 1;
  cloud.width = points.size();
  cloud.point_step = 32;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.assign(cloud.row_step, 0xa5);
  cloud.is_dense = true;
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& p : points) {
    *x = p[0]; *y = p[1]; *z = p[2];
    ++x; ++y; ++z;
  }
  return cloud;
}

tf2_msgs::TFMessage makeMount(Point translation = {{0, 0, 0}},
                               Point rpy = {{0, 0, 0}}) {
  tf2_msgs::TFMessage message;
  message.transforms.resize(2);
  message.transforms[0].header.frame_id = "base_link";
  message.transforms[0].child_frame_id = "gps_link";
  auto& mount = message.transforms[1];
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.translation.x = translation[0];
  mount.transform.translation.y = translation[1];
  mount.transform.translation.z = translation[2];
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  mount.transform.rotation.x = q.x();
  mount.transform.rotation.y = q.y();
  mount.transform.rotation.z = q.z();
  mount.transform.rotation.w = q.w();
  return message;
}

interfaces::CompetitionVehicleStatus attitude(double roll, double pitch, double yaw = 0) {
  interfaces::CompetitionVehicleStatus status;
  status.header.frame_id = "base_link";
  status.header.stamp.fromNSec(1000000000);
  status.roll_rad = roll;
  status.pitch_rad = pitch;
  status.yaw_rad = yaw;
  return status;
}

void expectPoints(const sensor_msgs::PointCloud2& cloud, const std::vector<Point>& expected) {
  ASSERT_EQ(expected.size(), cloud.width * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& p : expected) {
    EXPECT_NEAR(p[0], *x, 2e-5);
    EXPECT_NEAR(p[1], *y, 2e-5);
    EXPECT_NEAR(p[2], *z, 2e-5);
    EXPECT_TRUE(std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z));
    ++x; ++y; ++z;
  }
}

// Independent scalar rotations for constructing a world-ground observation.
// These do not call tf2 or the production Core.
Point rotateX(Point p, double angle) {
  return {{p[0], std::cos(angle)*p[1] - std::sin(angle)*p[2],
    std::sin(angle)*p[1] + std::cos(angle)*p[2]}};
}
Point rotateY(Point p, double angle) {
  return {{std::cos(angle)*p[0] + std::sin(angle)*p[2], p[1],
    -std::sin(angle)*p[0] + std::cos(angle)*p[2]}};
}
Point rotateZ(Point p, double angle) {
  return {{std::cos(angle)*p[0] - std::sin(angle)*p[1],
    std::sin(angle)*p[0] + std::cos(angle)*p[1], p[2]}};
}
}  // namespace

TEST(RollPitchLeveler, ZeroAttitudeMatchesPassThroughIncludingArbitraryMount) {
  const auto cloud = makeCloud({{{1, 2, 3}}, {{-1, 0, 0.5}}});
  for (const auto& mount : {makeMount({{1.5, 0, 1.403}}),
                           makeMount({{1.5, 0, 1.403}}, {{0.23, -0.38, 0.51}})}) {
    const auto output = lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(0, 0, 1.2), mount);
    const auto expected = lidar_leveler::PassThrough{}.process(cloud, mount);
    expectPoints(output, {{{*sensor_msgs::PointCloud2ConstIterator<float>(expected, "x"),
      *sensor_msgs::PointCloud2ConstIterator<float>(expected, "y"),
      *sensor_msgs::PointCloud2ConstIterator<float>(expected, "z")}},
      {{*++sensor_msgs::PointCloud2ConstIterator<float>(expected, "x"),
      *++sensor_msgs::PointCloud2ConstIterator<float>(expected, "y"),
      *++sensor_msgs::PointCloud2ConstIterator<float>(expected, "z")}}});
  }
}

TEST(RollPitchLeveler, PositiveAndNegativeRollUseRightHandRotation) {
  const auto cloud = makeCloud({{{0, 1, 0}}, {{0, 0, 1}}});
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(kHalfPi, 0), makeMount()),
    {{{0, 0, 1}}, {{0, -1, 0}}});
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(-kHalfPi, 0), makeMount()),
    {{{0, 0, -1}}, {{0, 1, 0}}});
}

TEST(RollPitchLeveler, PositiveAndNegativePitchUseRightHandRotation) {
  const auto cloud = makeCloud({{{1, 0, 0}}, {{0, 0, 1}}});
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(0, kHalfPi), makeMount()),
    {{{0, 0, -1}}, {{1, 0, 0}}});
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(0, -kHalfPi), makeMount()),
    {{{0, 0, 1}}, {{-1, 0, 0}}});
}

TEST(RollPitchLeveler, SimultaneousRotationAppliesRollBeforePitch) {
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(
    makeCloud({{{1, 2, 3}}}), attitude(kHalfPi, kHalfPi), makeMount()), {{{2, -3, -1}}});
}

TEST(RollPitchLeveler, IgnoresYawAtNonzeroRollAndPitch) {
  const auto cloud = makeCloud({{{1, 2, 3}}, {{-2, 0.5, -1}}});
  const auto mount = makeMount({{1.5, 0, 1.403}});
  const auto expected = ros::serialization::serializeMessage(
    lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(0.3, -0.4, 0), mount));
  for (double yaw : {-2.0, 1.7}) {
    const auto actual = ros::serialization::serializeMessage(
      lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(0.3, -0.4, yaw), mount));
    ASSERT_EQ(expected.num_bytes, actual.num_bytes);
    EXPECT_EQ(0, std::memcmp(expected.buf.get(), actual.buf.get(), expected.num_bytes));
  }
}

TEST(RollPitchLeveler, AppliesActualMountTranslationOnceBeforeLeveling) {
  const auto cloud = makeCloud({{{0, 0, 0}}, {{1, 2, 3}}});
  const auto mount = makeMount({{1.5, 0, 1.403}});
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(0, 0), mount),
    {{{1.5, 0, 1.403}}, {{2.5, 2, 4.403}}});
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(cloud, attitude(0, kHalfPi), mount),
    {{{1.403, 0, -1.5}}, {{4.403, 2, -2.5}}});
}

TEST(RollPitchLeveler, StaticMountRotationPrecedesVehicleRotation) {
  // Mount: Rz(90) Rx(90); level: Ry(90) Rx(90).
  // (1,2,3) -> (3,1,2) -> +(1.5,0,1.403) -> (1,-3.403,-4.5).
  expectPoints(lidar_leveler::RollPitchLeveler{}.process(makeCloud({{{1, 2, 3}}}),
    attitude(kHalfPi, kHalfPi), makeMount({{1.5, 0, 1.403}}, {{kHalfPi, 0, kHalfPi}})),
    {{{1, -3.403, -4.5}}});
}

TEST(RollPitchLeveler, FlatWorldGroundRemainsHorizontalAndYawRelative) {
  const std::vector<Point> world_relative = {{{-4, -2, -2}}, {{5, -2, -2}},
    {{-4, 3, -2}}, {{5, 3, -2}}};
  const Point mount_translation = {{1.5, 0, 1.403}};
  const Point mount_rpy = {{0.23, -0.38, 0.51}};
  for (const Point angles : {Point{{0, 0, 0}}, Point{{0.4, 0, 0.7}},
      Point{{-0.4, 0, -1.1}}, Point{{0, 0.3, 0.7}}, Point{{0, -0.3, -1.1}},
      Point{{0.37, -0.28, 1.2}}, Point{{-0.37, 0.28, -1.2}}}) {
    const auto status = attitude(angles[0], angles[1], angles[2]);
    std::vector<Point> raw, expected;
    for (auto point : world_relative) {
      expected.push_back(rotateZ(point, -status.yaw_rad));
      // world -> body: Rx(-roll) Ry(-pitch) Rz(-yaw).
      auto body = rotateX(rotateY(rotateZ(point, -status.yaw_rad),
        -status.pitch_rad), -status.roll_rad);
      for (std::size_t axis = 0; axis < 3; ++axis) body[axis] -= mount_translation[axis];
      // body -> lidar: inverse of Rz(mz) Ry(my) Rx(mx), after subtracting mount.
      raw.push_back(rotateX(rotateY(rotateZ(body, -mount_rpy[2]),
        -mount_rpy[1]), -mount_rpy[0]));
    }
    const auto output = lidar_leveler::RollPitchLeveler{}.process(
      makeCloud(raw), status, makeMount(mount_translation, mount_rpy));
    expectPoints(output, expected);  // All four output z values must equal -2.
  }
}

TEST(RollPitchLeveler, PreservesFieldsLayoutOtherBytesCountAndCoreHeader) {
  auto cloud = makeCloud({{{1, 2, 3}}, {{-1, 0, 0.5}}, {{2, 1, -2}}, {{0, 0, 0}}});
  cloud.height = 2;
  cloud.width = 2;
  cloud.row_step = cloud.width * cloud.point_step;
  const auto original = cloud;
  const auto output = lidar_leveler::RollPitchLeveler{}.process(
    cloud, attitude(0.3, -0.4), makeMount({{1.5, 0, 1.403}}));
  EXPECT_EQ(original, cloud);
  EXPECT_EQ(cloud.header, output.header);  // Header injection belongs to Pipeline.
  EXPECT_EQ(cloud.fields, output.fields);
  EXPECT_EQ(cloud.width, output.width);
  EXPECT_EQ(cloud.height, output.height);
  EXPECT_EQ(cloud.point_step, output.point_step);
  EXPECT_EQ(cloud.row_step, output.row_step);
  EXPECT_EQ(cloud.is_bigendian, output.is_bigendian);
  EXPECT_EQ(cloud.is_dense, output.is_dense);
  ASSERT_EQ(cloud.data.size(), output.data.size());
  for (std::size_t i = 0; i < output.data.size(); ++i) {
    if (i % cloud.point_step >= 12) {
      EXPECT_EQ(cloud.data[i], output.data[i]);
    }
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(output, "x"), y(output, "y"), z(output, "z");
  for (; x != x.end(); ++x, ++y, ++z)
    EXPECT_TRUE(std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z));
}

TEST(RollPitchLeveler, EmptyCloudPreservesLayoutAndHeader) {
  const auto cloud = makeCloud({});
  EXPECT_EQ(cloud, lidar_leveler::RollPitchLeveler{}.process(
    cloud, attitude(0.3, -0.4), makeMount({{1.5, 0, 1.403}})));
}

TEST(RollPitchLeveler, PreservesPaddingBetweenOrganizedRows) {
  // 36-byte rows exercise padding not divisible by point_step; 64-byte rows
  // reproduce padding mutation without triggering the old unbounded loop.
  for (const std::uint32_t row_step : {36u, 64u}) {
    auto cloud = makeCloud({{{1, 2, 3}}, {{-1, 0, 0.5}}});
    const auto packed = cloud.data;
    cloud.height = 2;
    cloud.width = 1;
    cloud.row_step = row_step;
    cloud.data.assign(cloud.height * cloud.row_step, 0x3e);
    for (std::size_t row = 0; row < cloud.height; ++row)
      std::copy_n(packed.begin() + row * cloud.point_step, cloud.point_step,
        cloud.data.begin() + row * cloud.row_step);
    const auto original = cloud;
    const auto output = lidar_leveler::RollPitchLeveler{}.process(
      cloud, attitude(kHalfPi, kHalfPi), makeMount({{1.5, 0, 1.403}}));
    EXPECT_EQ(original, cloud);
    EXPECT_EQ(cloud.header, output.header);
    EXPECT_EQ(cloud.fields, output.fields);
    EXPECT_EQ(cloud.width, output.width);
    EXPECT_EQ(cloud.height, output.height);
    EXPECT_EQ(cloud.point_step, output.point_step);
    EXPECT_EQ(cloud.row_step, output.row_step);
    ASSERT_EQ(cloud.data.size(), output.data.size());
    const std::array<Point, 2> expected = {{{{2, -4.403, -2.5}}, {{0, -1.903, -0.5}}}};
    for (std::size_t row = 0; row < cloud.height; ++row) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        float value;
        std::memcpy(&value, &output.data[row * output.row_step + axis * sizeof(float)], sizeof(float));
        EXPECT_NEAR(expected[row][axis], value, 2e-5);
        EXPECT_TRUE(std::isfinite(value));
      }
      for (std::size_t byte = 12; byte < cloud.row_step; ++byte) {
        EXPECT_EQ(cloud.data[row * cloud.row_step + byte], output.data[row * output.row_step + byte]);
      }
    }
  }
}

TEST(LevelingInputs, DeclaresMountLayoutAndFiniteAttitude) {
  const lidar_leveler::LevelingInputs validator;
  const node_pipeline::HealthLimits limits;
  std::vector<node_pipeline::Kind> issues;
  validator(makeCloud({{{1, 2, 3}}}), limits, issues);
  validator(makeMount(), limits, issues);
  validator(attitude(0.1, -0.2), limits, issues);
  EXPECT_TRUE(issues.empty());
  auto no_mount = makeMount();
  no_mount.transforms.pop_back();
  validator(no_mount, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}, issues);
  issues.clear();
  auto no_z = makeCloud({{{1, 2, 3}}});
  no_z.fields.erase(no_z.fields.begin() + 2);
  validator(no_z, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}, issues);
  issues.clear();
  validator(attitude(std::nan(""), INFINITY), limits, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::NaN, node_pipeline::Kind::Inf}), issues);
}
