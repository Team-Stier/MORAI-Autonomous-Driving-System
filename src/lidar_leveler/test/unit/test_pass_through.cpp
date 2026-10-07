#include <lidar_leveler/core_dispatch.hpp>
#include <lidar_leveler/components.hpp>
#include <lidar_leveler/header_injector.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cstring>
#include <cmath>

TEST(PassThrough, AppliesMountingTfAndPreservesOtherFields) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 7;
  cloud.header.stamp.fromNSec(1234567890);
  cloud.header.frame_id = "velodyne";
  cloud.height = 1;
  cloud.width = 2;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32,
    "ring", 1, sensor_msgs::PointField::UINT16);
  cloud.point_step = 32;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.assign(cloud.row_step, 0xa5);
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = 1; *y = 2; *z = 3;
  ++x; ++y; ++z;
  *x = -1; *y = 0; *z = 0.5;

  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(2);
  tf_static.transforms[0].header.frame_id = "base_link";
  tf_static.transforms[0].child_frame_id = "gps_link";
  auto& mount = tf_static.transforms[1];
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.translation.x = 10;
  mount.transform.translation.y = 20;
  mount.transform.translation.z = 30;
  mount.transform.rotation.z = std::sqrt(0.5);
  mount.transform.rotation.w = std::sqrt(0.5);

  lidar_leveler::CoreSet cores{lidar_leveler::PassThrough{}};
  lidar_leveler::Data data;
  data.get<0>().sample.message = std::make_shared<const sensor_msgs::PointCloud2>(cloud);
  node_pipeline::StageData stage_data;
  stage_data.update(data);
  node_pipeline::CoreResult result;
  bool called = false;
  const auto before = [&] { called = true; };
  EXPECT_FALSE(lidar_leveler::processCore(cores, lidar_leveler::CoreId::PassThrough, stage_data, result, before));
  EXPECT_FALSE(called);
  data.get<2>().sample.message = std::make_shared<const tf2_msgs::TFMessage>(tf_static);
  stage_data.update(data);
  ASSERT_TRUE(lidar_leveler::processCore(cores, lidar_leveler::CoreId::PassThrough, stage_data, result, before));
  EXPECT_TRUE(called);

  auto expected_cloud = cloud;
  sensor_msgs::PointCloud2Iterator<float> expected_x(expected_cloud, "x"),
    expected_y(expected_cloud, "y"), expected_z(expected_cloud, "z");
  *expected_x = 8; *expected_y = 21; *expected_z = 33;
  ++expected_x; ++expected_y; ++expected_z;
  *expected_x = 10; *expected_y = 19; *expected_z = 30.5;
  const auto expected = ros::serialization::serializeMessage(expected_cloud);
  const auto output = result(stage_data);
  const auto actual = ros::serialization::serializeMessage(output.message<sensor_msgs::PointCloud2>(0));
  ASSERT_EQ(expected.num_bytes, actual.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected.buf.get(), actual.buf.get(), expected.num_bytes));
}


namespace {
sensor_msgs::PointCloud2 registrationCloud() {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 7;
  cloud.header.stamp.fromNSec(1000000000);
  cloud.header.frame_id = "velodyne";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = 1;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = 2;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = 3;
  return cloud;
}
tf2_msgs::TFMessage registrationMount() {
  tf2_msgs::TFMessage message;
  message.transforms.resize(1);
  auto& mount = message.transforms.front();
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.translation.x = 1.5;
  mount.transform.translation.z = 1.403;
  mount.transform.rotation.w = 1;
  return message;
}
interfaces::CompetitionVehicleStatus registrationAttitude() {
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromNSec(1000000000);
  status.header.frame_id = "base_link";
  status.roll_rad = std::acos(-1.0) / 2;
  status.pitch_rad = std::acos(-1.0) / 2;
  status.yaw_rad = 1.2;
  return status;
}
struct TestAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
  static std::int64_t stamp(const tf2_msgs::TFMessage&) { return 0; }
  static std::string frame(const tf2_msgs::TFMessage&) { return {}; }
};
struct TestIO {
  using MessageType = sensor_msgs::PointCloud2;
  std::shared_ptr<std::size_t> publications;
  std::shared_ptr<MessageType> output;
  node_pipeline::Clocks readClocks() const { return {1000000000, 1000000000}; }
  void publish(const MessageType& message) { ++*publications; *output = message; }
  void reportHealth(const node_pipeline::StageHealth&, std::size_t) const {}
  void report(const node_pipeline::ProcessReport&) const noexcept {}
};
}  // namespace

TEST(RollPitchRegistration, RequiresAllThreeInputsAndPreservesCloudTimeMetadata) {
  lidar_leveler::CoreSet cores{};
  const auto cloud = registrationCloud();
  const auto competition = registrationAttitude();
  const auto tf_static = registrationMount();
  for (int missing = -1; missing < 3; ++missing) {
    lidar_leveler::Data typed;
    if (missing != 0) typed.get<0>().sample.message = std::make_shared<const sensor_msgs::PointCloud2>(cloud);
    if (missing != 1) typed.get<1>().sample.message = std::make_shared<const interfaces::CompetitionVehicleStatus>(competition);
    if (missing != 2) typed.get<2>().sample.message = std::make_shared<const tf2_msgs::TFMessage>(tf_static);
    typed.get<0>().sample.stamp_ns = 123;
    typed.get<0>().sample.receipt_steady_ns = 456;
    typed.get<0>().sample.has_stamp = true;
    typed.get<0>().sample.has_receipt = true;
    node_pipeline::StageData data;
    data.update(typed);
    node_pipeline::CoreResult result;
    std::size_t calls = 0;
    const auto processed = lidar_leveler::processCore(cores, lidar_leveler::CoreId::RollPitchLeveler,
      data, result, [&] { ++calls; });
    EXPECT_EQ(missing == -1, processed);
    EXPECT_EQ(missing == -1 ? 1u : 0u, calls);
    if (missing != -1) {
      EXPECT_FALSE(bool(result));
      continue;
    }
    ASSERT_TRUE(processed);
    const auto output = result(data);
    ASSERT_EQ(1u, output.inputs.size());
    const auto& actual = output.message<sensor_msgs::PointCloud2>(0);
    EXPECT_NEAR(2, *sensor_msgs::PointCloud2ConstIterator<float>(actual, "x"), 2e-5);
    EXPECT_NEAR(-4.403, *sensor_msgs::PointCloud2ConstIterator<float>(actual, "y"), 2e-5);
    EXPECT_NEAR(-2.5, *sensor_msgs::PointCloud2ConstIterator<float>(actual, "z"), 2e-5);
    EXPECT_EQ(cloud.header, actual.header);
    EXPECT_EQ(123, output.inputs[0].stamp_ns);
    EXPECT_EQ(456, output.inputs[0].receipt_steady_ns);
    EXPECT_TRUE(output.inputs[0].has_stamp);
    EXPECT_TRUE(output.inputs[0].has_receipt);
  }
}

TEST(LevelerPipeline, PublishesLevelFrameAndPassThroughComparisonFrame) {
  namespace pkg = lidar_leveler;
  namespace np = node_pipeline;
  using Components = pkg::Components<TestAccess>;
  for (const auto core : {pkg::CoreId::RollPitchLeveler, pkg::CoreId::PassThrough}) {
    auto selector = Components::selector();
    selector.section_core_map[0] = core;
    const auto frame = core == pkg::CoreId::RollPitchLeveler ? "base_link_level" : "base_link";
    std::array<np::HealthLimits, 3> limits;
    limits[0].frame = "velodyne";
    limits[1].frame = "base_link";
    limits[2].check_stamp = false;
    limits[2].check_skew = false;
    limits[2].check_receive_timeout = false;
    auto publications = std::make_shared<std::size_t>(0);
    auto output = std::make_shared<sensor_msgs::PointCloud2>();
    Components::Node<TestIO, pkg::HeaderInjector> node(
      {np::ExecutionMode::OnInput, pkg::Input::Cloud, 10}, Components::Buffer(8), {},
      {np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Data>(
        Components::Checker(limits, 1000000000), Components::policy(np::HealthLimits{}),
        selector, pkg::processCore)}, pkg::CoreSet{pkg::PassThrough{}, pkg::RollPitchLeveler{}},
      pkg::HeaderInjector{frame}, TestIO{publications, output});
    const auto cloud = registrationCloud();
    node.onInput(pkg::InputTag<pkg::Input::TfStatic>{}, registrationMount());
    // RollPitchLeveler must not publish until competition arrives.
    if (core == pkg::CoreId::RollPitchLeveler) {
      const auto report = node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, cloud);
      EXPECT_EQ(np::ProcessStatus::NoCore, report.status);
      EXPECT_EQ(0u, *publications);
      node.onInput(pkg::InputTag<pkg::Input::Competition>{}, registrationAttitude());
    }
    const auto report = node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, cloud);
    ASSERT_EQ(np::ProcessStatus::Published, report.status);
    EXPECT_EQ(1u, *publications);
    EXPECT_EQ(frame, output->header.frame_id);
    EXPECT_EQ(cloud.header.stamp, output->header.stamp);
    EXPECT_EQ(cloud.header.seq, output->header.seq);
    EXPECT_NEAR(core == pkg::CoreId::RollPitchLeveler ? 2 : 2.5,
      *sensor_msgs::PointCloud2ConstIterator<float>(*output, "x"), 2e-5);
    EXPECT_NEAR(core == pkg::CoreId::RollPitchLeveler ? -4.403 : 2,
      *sensor_msgs::PointCloud2ConstIterator<float>(*output, "y"), 2e-5);
    EXPECT_NEAR(core == pkg::CoreId::RollPitchLeveler ? -2.5 : 4.403,
      *sensor_msgs::PointCloud2ConstIterator<float>(*output, "z"), 2e-5);
  }
}

TEST(LevelerHeader, DefaultProductionFrameUsesLevelCoordinatesAndOriginalStamp) {
  auto cloud = registrationCloud();
  node_pipeline::Context<3> context;
  context.stamp_ns = 123456789;
  lidar_leveler::HeaderInjector{}.injectHeader(cloud, context);
  EXPECT_EQ("base_link_level", cloud.header.frame_id);
  EXPECT_EQ(123456789u, cloud.header.stamp.toNSec());
  EXPECT_EQ(7u, cloud.header.seq);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
