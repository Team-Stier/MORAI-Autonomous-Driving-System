#include <section_annotator/template_components.hpp>
#include <node_pipeline/ros.hpp>
#include <fstream>
#include <iterator>
namespace pkg = section_annotator;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
using IO = np::RosIO<Message>;
struct RosHeaderInjector {
  void injectHeader(interfaces::SectionAnnotation& message, const np::Context<1>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "map";
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onEgo(const geometry_msgs::PoseStamped::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Ego>{}, np::retainMessage(msg)); }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "section_annotator_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/localization/ego"), pkg::Input::Ego}},
    std::map<std::string, pkg::Input>{{"ego", pkg::Input::Ego}},
    std::map<std::string, pkg::CoreId>{}, selector, policy);
  std::array<np::HealthLimits, 1> limits;
  limits.fill(settings.health);
  limits[0].frame = "map";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/ego/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/ego/receive_timeout_sec", 0.3);
  IO io{nh.advertise<interfaces::SectionAnnotation>("/section_annotation", 1)};
  std::string map_file_path;
  if (!params.getParam("map_file", map_file_path)) throw std::invalid_argument("missing map_file");
  std::ifstream map_file_stream(map_file_path);
  if (!map_file_stream) throw std::runtime_error("cannot open " + map_file_path);
  const std::string map_file_data{std::istreambuf_iterator<char>(map_file_stream), std::istreambuf_iterator<char>()};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    Components::Checker(limits, io.readClocks().steady_ns), policy, selector, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_ego = nh.subscribe("/localization/ego", 1, &RosCallbacks::onEgo, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
