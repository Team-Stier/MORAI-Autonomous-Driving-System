#include <object_detector/components.hpp>
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
  np::Clocks readClocks() const { return {1000000000, 1000000000}; }
  void publish(const interfaces::Objects&) { ++*publications; }
  void reportHealth(const np::StageHealth&, std::size_t) const {}
  void report(const np::ProcessReport&) const noexcept {}
};
struct HeaderInjector {
  void injectHeader(interfaces::Objects&, const np::Context<1>&) const {}
};

TEST(ObjectDetectorStages, UnregisteredFirstStageStopsWithoutPublication) {
  auto selector = Components::selector();
  auto publications = std::make_shared<std::size_t>(0);
  np::HealthLimits limits; limits.frame = "base_link";
  using Context = Components::Buffer::ContextType;
  Components::Node<IO, HeaderInjector> node(
    {np::ExecutionMode::OnInput, pkg::Input::Cloud, 10}, Components::Buffer(8), {},
    {np::makeStage<pkg::CoreSet, Context, pkg::Data>(Components::Checker(limits, 1000000000),
       Components::policy(limits), selector.stage1, pkg::processStage1),
     np::makeStage<pkg::CoreSet, Context, pkg::Stage2Data>(Components::Stage2Checker(limits, 1000000000),
       Components::Stage2Policy{{limits.upper},{limits.lower}}, selector.stage2, pkg::processStage2)}, {}, {}, IO{publications});
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp.fromNSec(1000000000); cloud.header.frame_id = "base_link";
  const auto report = node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, cloud);
  EXPECT_EQ(np::ProcessStatus::NoCore, report.status); EXPECT_EQ(1u, report.health_checks);
  EXPECT_EQ(0u, *publications);
}
