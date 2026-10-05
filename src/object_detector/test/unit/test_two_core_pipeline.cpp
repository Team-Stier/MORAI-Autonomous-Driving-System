#include <object_detector/components.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>

namespace pkg = object_detector;
namespace np = node_pipeline;

struct Access {
  static std::int64_t stamp(const sensor_msgs::PointCloud2& message) { return message.header.stamp.toNSec(); }
  static const std::string& frame(const sensor_msgs::PointCloud2& message) { return message.header.frame_id; }
};
using Components = pkg::Components<Access>;
struct IO {
  using MessageType = interfaces::Objects;
  std::shared_ptr<std::size_t> publications;
  std::shared_ptr<interfaces::Objects> output;
  np::Clocks readClocks() const { return {1000000000, 1000000000}; }
  void publish(const interfaces::Objects& objects) {
    ++*publications;
    if (output) *output = objects;
  }
  void reportHealth(const np::StageHealth&, std::size_t) const {}
  void report(const np::ProcessReport&) const noexcept {}
};
struct HeaderInjector {
  void injectHeader(interfaces::Objects& message, const np::Context<1>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "base_link";
  }
};

TEST(ObjectDetectorTwoCore, RequiresBothStageSelectionsBeforePublication) {
  // 어느 단계든 미선택이면 최종 발행하지 않는다.
  for (int selected_stage = 0; selected_stage < 3; ++selected_stage) {
    auto selector = Components::selector();
    if (selected_stage == 1) selector.stage1.section_core_map[0] = pkg::FirstCoreId{};
    if (selected_stage == 2) selector.stage2.section_core_map[0] = pkg::SecondCoreId{};
    auto publications = std::make_shared<std::size_t>(0);
    np::HealthLimits limits;
    limits.frame = "base_link";
    Components::Node<IO, HeaderInjector> node(
      {np::ExecutionMode::OnInput, pkg::Input::Cloud, 10}, Components::Buffer(8), {},
      {np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Data>(
       Components::Checker(limits, 1000000000), Components::policy(limits), selector.stage1, pkg::processStage1),
     np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Stage2Data>(
       Components::Stage2Checker(limits, 1000000000), Components::Stage2Policy{{limits.upper}, {limits.lower}},
       selector.stage2, pkg::processStage2)}, {}, {}, IO{publications});
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp.fromNSec(1000000000);
    cloud.header.frame_id = "base_link";
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(0);
    const auto report = node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, cloud);
    EXPECT_EQ(np::ProcessStatus::NoCore, report.status);
    EXPECT_EQ(1u, report.health_checks);
    EXPECT_EQ(0u, *publications);
  }
}

TEST(ObjectDetectorTwoCore, PublishesDbscanObjectsThroughSecondCore) {
  auto selector = Components::selector();
  selector.stage1.section_core_map[0] = pkg::FirstCoreId::Dbscan;
  selector.stage2.section_core_map[0] = pkg::SecondCoreId::ObjectsPassThrough;
  auto publications = std::make_shared<std::size_t>(0);
  auto output = std::make_shared<interfaces::Objects>();
  auto parameters = pkg::DbscanParameters{};
  parameters.dbscan_min_points = 1;
  parameters.min_cluster_size = 1;
  np::HealthLimits limits;
  limits.frame = "base_link";
  Components::Node<IO, HeaderInjector> node(
    {np::ExecutionMode::OnInput, pkg::Input::Cloud, 10}, Components::Buffer(8), {},
    {np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Data>(
       Components::Checker(limits, 1000000000), Components::policy(limits), selector.stage1, pkg::processStage1),
     np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Stage2Data>(
       Components::Stage2Checker(limits, 1000000000), Components::Stage2Policy{{limits.upper}, {limits.lower}},
       selector.stage2, pkg::processStage2)},
    pkg::CoreSet{pkg::Dbscan(parameters), pkg::ObjectsPassThrough{}}, {}, IO{publications, output});
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp.fromNSec(1000000000);
  cloud.header.frame_id = "base_link";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = 4.0f;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = 0.0f;
  *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = 0.6f;
  const auto report = node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, cloud);
  EXPECT_EQ(np::ProcessStatus::Published, report.status);
  EXPECT_EQ(1u, *publications);
  ASSERT_EQ(1u, output->objects.size());
  EXPECT_NEAR(4.0, output->objects.front().position.x, 1e-6);
  EXPECT_EQ("unknown", output->objects.front().label);
  EXPECT_FALSE(output->objects.front().velocity_available);
  EXPECT_FALSE(output->objects.front().acceleration_available);
  EXPECT_EQ(cloud.header.stamp, output->header.stamp);
  EXPECT_EQ("base_link", output->header.frame_id);
  EXPECT_EQ(output->objects.front().position, output->objects.front().bbox_pose.position);
  EXPECT_DOUBLE_EQ(1.0, output->objects.front().bbox_pose.orientation.w);
}

TEST(ObjectDetectorTwoCore, MissingInputsDoNotCallEitherCore) {
  pkg::CoreSet cores{};
  np::StageData data;
  np::CoreResult result;
  std::size_t calls = 0;
  const np::BeforeCore before = [&] { ++calls; };
  EXPECT_FALSE(pkg::processStage1(cores, pkg::FirstCoreId::Dbscan, data, result, before));
  EXPECT_FALSE(pkg::processStage2(cores, pkg::SecondCoreId::ObjectsPassThrough, data, result, before));
  EXPECT_EQ(0u, calls);
  EXPECT_FALSE(bool(result));
}

TEST(ObjectDetectorTwoCore, PassThroughPreservesPayloadAndInputTimeMetadata) {
  interfaces::Objects objects;
  objects.header.stamp.fromNSec(123);
  objects.header.frame_id = "base_link";
  interfaces::Object object;
  object.id = 42;
  object.label = "unknown";
  object.position.x = 4.0;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1.0;
  object.bbox_size.x = 0.5;
  object.velocity_available = false;
  object.acceleration_available = false;
  objects.objects.push_back(object);
  pkg::Stage2Data typed;
  typed.get<0>().sample.message = std::make_shared<interfaces::Objects>(objects);
  typed.get<0>().sample.stamp_ns = 123;
  typed.get<0>().sample.receipt_steady_ns = 456;
  typed.get<0>().sample.has_stamp = true;
  typed.get<0>().sample.has_receipt = true;
  np::StageData data;
  data.update(typed);
  pkg::CoreSet cores{};
  np::CoreResult result;
  std::size_t calls = 0;
  ASSERT_TRUE(pkg::processStage2(cores, pkg::SecondCoreId::ObjectsPassThrough,
                               data, result, [&] { ++calls; }));
  const auto output = result(data);
  EXPECT_EQ(1u, calls);
  ASSERT_EQ(1u, output.inputs.size());
  EXPECT_EQ(objects, output.message<interfaces::Objects>(0));
  EXPECT_EQ(123, output.inputs[0].stamp_ns);
  EXPECT_EQ(456, output.inputs[0].receipt_steady_ns);
  EXPECT_EQ(data.inputs[0].has_stamp, output.inputs[0].has_stamp);
  EXPECT_EQ(data.inputs[0].has_receipt, output.inputs[0].has_receipt);
}
