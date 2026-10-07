#include <object_detector/cores/cluster_match.hpp>
#include <object_detector/cores/tracker.hpp>
#include <object_detector/parameters/cluster_match.hpp>
#include <object_detector/parameters/tracker.hpp>
#include <object_detector/validators/cluster_match.hpp>
#include <node_pipeline/detail/message_traits.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>

namespace pkg = object_detector;
namespace np = node_pipeline;
namespace runtime = node_pipeline::detail;

namespace {
sensor_msgs::PointCloud2 emptyCloud() {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp.fromNSec(1000000000);
  cloud.header.frame_id = "base_link";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  return cloud;
}
interfaces::CompetitionVehicleStatus vehicleStatus() {
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromNSec(1000000000);
  status.header.frame_id = "base_link";
  return status;
}
runtime::CoreSpec clusterSpec() {
  runtime::CoreSpec spec;
  spec.name = "cluster_match.hpp";
  spec.stage = "stage1";
  spec.inputs = {"cloud", "competition"};
  spec.outputs = {{"objects", ""}, {"competition", "competition"}};
  return spec;
}
runtime::CoreSpec trackerSpec() {
  runtime::CoreSpec spec;
  spec.name = "tracker.hpp";
  spec.stage = "stage2";
  spec.inputs = {"objects", "competition"};
  spec.outputs = {{"objects", ""}};
  return spec;
}
void clusterStage(runtime::Engine& engine, bool selected = true) {
  runtime::InputSpec cloud{"cloud"}, status{"competition"};
  cloud.limits.frame = status.limits.frame = "base_link";
  engine.input<sensor_msgs::PointCloud2>(cloud);
  engine.input<interfaces::CompetitionVehicleStatus>(status);
  engine.core<pkg::ClusterMatch, pkg::parameters::ClusterMatch, pkg::validators::ClusterMatch>(
    clusterSpec(), np::Parameters{});
  runtime::StageSpec stage;
  stage.name = "stage1";
  if (selected) stage.section_core_map[0] = "cluster_match.hpp";
  engine.stage(stage);
}
void trackerStage(runtime::Engine& engine) {
  engine.core<pkg::Tracker, pkg::parameters::Tracker>(trackerSpec(), np::Parameters{});
  runtime::StageSpec stage;
  stage.name = "stage2";
  stage.section_core_map[0] = "tracker.hpp";
  engine.stage(stage);
}
void pushCloud(runtime::Engine& engine) {
  engine.push("cloud", std::make_shared<const sensor_msgs::PointCloud2>(emptyCloud()),
    {1000000000, 1000000000}, 1000000000);
}
void pushStatus(runtime::Engine& engine) {
  engine.push("competition", std::make_shared<const interfaces::CompetitionVehicleStatus>(vehicleStatus()),
    {1000000000, 1000000000}, 1000000000);
}
}

TEST(ObjectDetectorStages, UnregisteredFirstStageStopsWithoutAResult) {
  runtime::Engine engine;
  clusterStage(engine, false);
  pushCloud(engine);
  const auto report = engine.process({1000000000, 1000000000});
  EXPECT_EQ(np::ProcessStatus::NoCore, report.status);
  EXPECT_TRUE(engine.output().empty());
}

TEST(ObjectDetectorStages, NativeTupleRetainsStatusTimeAndOnlyNewDetections) {
  runtime::Engine engine;
  clusterStage(engine);
  pushCloud(engine);
  pushStatus(engine);
  ASSERT_EQ(np::ProcessStatus::Published, engine.process({1000000000, 1000000000}).status);
  const auto& middle = engine.output();
  ASSERT_EQ(2u, middle.size());
  EXPECT_EQ(1000000000, middle.at("competition").stamp_ns);
  EXPECT_EQ(1000000000, middle.at("competition").receipt_ns);
  EXPECT_TRUE(middle.at("competition").has_stamp);
  EXPECT_TRUE(middle.at("competition").has_receipt);
  EXPECT_NE(engine.assembled().at("competition").value, middle.at("competition").value);
  EXPECT_FALSE(middle.at("objects").has_stamp);
  EXPECT_FALSE(middle.at("objects").has_receipt);
  EXPECT_EQ(0u, middle.count("cloud"));
  EXPECT_TRUE(engine.result<interfaces::Objects>("objects").objects.empty());

  runtime::CoreModel<pkg::Tracker, pkg::parameters::Tracker> tracker(trackerSpec(), np::Parameters{});
  auto input = middle;
  tracker.contract(input);
  const auto output = tracker.compute(input);
  ASSERT_EQ(1u, output.size());
  EXPECT_EQ(std::type_index(typeid(interfaces::Objects)), output.at("objects").type);
  EXPECT_FALSE(output.at("objects").has_stamp);
  EXPECT_FALSE(output.at("objects").has_receipt);
}

TEST(ObjectDetectorStages, MissingCloudOrStatusBlocksTheFirstCoreAndTheChain) {
  for (int missing = 0; missing < 2; ++missing) {
    runtime::Engine engine;
    clusterStage(engine);
    trackerStage(engine);
    if (missing != 0) pushCloud(engine);
    if (missing != 1) pushStatus(engine);
    EXPECT_EQ(np::ProcessStatus::Rejected, engine.process({1000000000, 1000000000}).status);
    EXPECT_TRUE(engine.output().empty());
  }
  runtime::Engine engine;
  clusterStage(engine);
  trackerStage(engine);
  pushCloud(engine);
  pushStatus(engine);
  EXPECT_EQ(np::ProcessStatus::Published, engine.process({1000000000, 1000000000}).status);
  EXPECT_TRUE(engine.result<interfaces::Objects>("objects").objects.empty());
}

TEST(ObjectDetectorStages, MissingIntermediateObjectsOrStatusBlocksTracker) {
  for (int missing = 0; missing < 2; ++missing) {
    runtime::Engine engine;
    runtime::InputSpec objects{"objects"}, status{"competition"};
    objects.limits.frame.clear();
    status.limits.frame = "base_link";
    engine.input<interfaces::Objects>(objects);
    engine.input<interfaces::CompetitionVehicleStatus>(status);
    trackerStage(engine);
    if (missing != 0)
      engine.push("objects", std::make_shared<const interfaces::Objects>(),
        {1000000000, 1000000000}, 0, false);
    if (missing != 1) pushStatus(engine);
    EXPECT_EQ(np::ProcessStatus::Rejected, engine.process({1000000000, 1000000000}).status);
    EXPECT_TRUE(engine.output().empty());
  }
}
