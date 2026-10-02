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
  std::shared_ptr<std::size_t> publications;
  np::Clocks readClocks() const { return {1000000000, 1000000000}; }
  void publish(const interfaces::Objects&) { ++*publications; }
  void reportHealth(const np::Health<pkg::Input, 1>&, std::size_t) const {}
  void report(const np::ProcessReport&) const noexcept {}
};
struct HeaderInjector {
  void injectHeader(interfaces::Objects&, const np::Context<1>&) const {}
};

TEST(ObjectDetectorTwoCore, RequiresBothStageSelectionsBeforeCallingCores) {
  // 선택표 결합만 검사한다. Core 구현·등록은 추가하지 않는다.
  for (int selected_stage = 0; selected_stage < 3; ++selected_stage) {
    auto selector = Components::selector();
    if (selected_stage == 1) selector.core1.section_core_map[0] = pkg::FirstCoreId{};
    if (selected_stage == 2) selector.core2.section_core_map[0] = pkg::SecondCoreId{};
    auto publications = std::make_shared<std::size_t>(0);
    np::HealthLimits limits;
    limits.frame = "base_link";
    Components::Node<IO, HeaderInjector> node(
      {np::ExecutionMode::OnInput, pkg::Input::Cloud, 10}, Components::Buffer(8), {},
      Components::Checker(limits, 1000000000), Components::policy(limits), selector, {}, {}, IO{publications});
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp.fromNSec(1000000000);
    cloud.header.frame_id = "base_link";
    const auto report = node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, cloud);
    EXPECT_EQ(np::ProcessStatus::NoCore, report.status);
    EXPECT_EQ(1u, report.health_checks);
    EXPECT_EQ(0u, *publications);
  }
}
