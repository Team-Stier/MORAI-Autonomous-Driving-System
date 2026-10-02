#include <object_detector/components.hpp>
#include <node_pipeline/ros.hpp>
namespace pkg = object_detector;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
using IO = np::RosIO<Message>;
struct RosHeaderInjector {
  void injectHeader(interfaces::Objects& message, const np::Context<1>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "base_link";
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, np::retainMessage(msg)); }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& msg) {
    node.onSection(np::Section{true, msg->state, static_cast<std::int64_t>(msg->header.stamp.toNSec()), msg->header.frame_id});
  }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detector_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const std::map<std::string, pkg::Input> topics{{nh.resolveName("/velodyne_points/leveled"), pkg::Input::Cloud}};
  const std::map<std::string, pkg::Input> inputs{{"cloud", pkg::Input::Cloud}};
  const auto settings = np::loadRosConfig(nh, params, topics, inputs,
    std::map<std::string, pkg::FirstCoreId>{}, selector.core1, policy);
  ros::NodeHandle core1_params(params, "core1"), core2_params(params, "core2");
  auto mapping_policy = policy;
  // TODO: 각 단계의 Core 파일 이름→ID를 등록한다.
  np::loadRosConfig(nh, core1_params, topics, inputs,
    std::map<std::string, pkg::FirstCoreId>{}, selector.core1, mapping_policy);
  np::loadRosConfig(nh, core2_params, topics, inputs,
    std::map<std::string, pkg::SecondCoreId>{}, selector.core2, mapping_policy);
  std::array<np::HealthLimits, 1> limits;
  limits.fill(settings.health);
  limits[0].frame = "base_link";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/cloud/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/cloud/receive_timeout_sec", 0.3);
  IO io{nh.advertise<interfaces::Objects>("/objects", 1)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    Components::Checker(limits, io.readClocks().steady_ns), policy, selector, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_cloud = nh.subscribe("/velodyne_points/leveled", 1, &RosCallbacks::onCloud, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
