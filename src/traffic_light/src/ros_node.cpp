#include <traffic_light/template_components.hpp>
#include <node_pipeline/ros.hpp>
namespace pkg = traffic_light;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
using IO = np::RosIO<Message>;
struct RosHeaderInjector {
  void injectHeader(interfaces::TrafficLight& message, const np::Context<1>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "camera_7_optical_frame";
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onImage(const sensor_msgs::CompressedImage::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Image>{}, np::retainMessage(msg)); }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& msg) {
    node.onSection(np::Section{true, msg->state, static_cast<std::int64_t>(msg->header.stamp.toNSec()), msg->header.frame_id});
  }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "traffic_light_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/image/camera_7/compressed"), pkg::Input::Image}},
    std::map<std::string, pkg::Input>{{"image", pkg::Input::Image}},
    std::map<std::string, pkg::CoreId>{}, selector, policy);
  std::array<np::HealthLimits, 1> limits;
  limits.fill(settings.health);
  limits[0].frame = "camera_7_optical_frame";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/image/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/image/receive_timeout_sec", 0.3);
  IO io{nh.advertise<interfaces::TrafficLight>("/traffic_light", 1)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    Components::Checker(limits, io.readClocks().steady_ns), policy, selector, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_image = nh.subscribe("/image/camera_7/compressed", 1, &RosCallbacks::onImage, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
