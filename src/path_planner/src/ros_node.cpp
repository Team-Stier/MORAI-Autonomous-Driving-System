#include <path_planner/components.hpp>
#include <node_pipeline/ros.hpp>
#include <fstream>
#include <iterator>
namespace pkg = path_planner;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
  static std::int64_t stamp(const tf2_msgs::TFMessage& m) { return m.transforms.empty() ? 0 : m.transforms.front().header.stamp.toNSec(); }
  static std::string frame(const tf2_msgs::TFMessage& m) { return m.transforms.empty() ? "" : m.transforms.front().header.frame_id; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
using IO = np::RosIO<Message>;
struct RosHeaderInjector {
  void injectHeader(interfaces::Path& message, const np::Context<4>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "map";
    for (auto& point : message.points) point.pose.header = message.header;
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onEgo(const geometry_msgs::PoseStamped::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Ego>{}, np::retainMessage(msg)); }
  void onCompetition(const interfaces::CompetitionVehicleStatus::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Competition>{}, np::retainMessage(msg)); }
  void onObjects(const interfaces::Objects::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Objects>{}, np::retainMessage(msg)); }
  void onTf(const tf2_msgs::TFMessage::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Tf>{}, np::retainMessage(msg)); }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& msg) {
    node.onSection(np::Section{true, msg->state, static_cast<std::int64_t>(msg->header.stamp.toNSec()), msg->header.frame_id});
  }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/localization/ego"), pkg::Input::Ego}, {nh.resolveName("/vehicle/competition_status"), pkg::Input::Competition}, {nh.resolveName("/objects"), pkg::Input::Objects}, {nh.resolveName("/tf"), pkg::Input::Tf}},
    std::map<std::string, pkg::Input>{{"ego", pkg::Input::Ego}, {"competition", pkg::Input::Competition}, {"objects", pkg::Input::Objects}, {"tf", pkg::Input::Tf}},
    std::map<std::string, pkg::CoreId>{}, selector, policy);
  std::array<np::HealthLimits, 4> limits;
  limits.fill(settings.health);
  limits[0].frame = "map";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/ego/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/ego/receive_timeout_sec", 0.3);
  limits[1].frame = "base_link";
  limits[1].stale_ns = np::readDurationNs(params, "inputs/competition/stale_after_sec", 0.3);
  limits[1].receive_timeout_ns = np::readDurationNs(params, "inputs/competition/receive_timeout_sec", 0.3);
  limits[2].frame = "base_link";
  limits[2].stale_ns = np::readDurationNs(params, "inputs/objects/stale_after_sec", 0.3);
  limits[2].receive_timeout_ns = np::readDurationNs(params, "inputs/objects/receive_timeout_sec", 0.3);
  limits[3].frame = "map";
  limits[3].stale_ns = np::readDurationNs(params, "inputs/tf/stale_after_sec", 0.3);
  limits[3].receive_timeout_ns = np::readDurationNs(params, "inputs/tf/receive_timeout_sec", 0.3);
  IO io{nh.advertise<interfaces::Path>("/local_path", 1)};
  std::string map_file_path;
  if (!params.getParam("map_file", map_file_path)) throw std::invalid_argument("missing map_file");
  std::ifstream map_file_stream(map_file_path);
  if (!map_file_stream) throw std::runtime_error("cannot open " + map_file_path);
  const std::string map_file_data{std::istreambuf_iterator<char>(map_file_stream), std::istreambuf_iterator<char>()};
  std::string checkpoints_file_path;
  if (!params.getParam("checkpoints_file", checkpoints_file_path)) throw std::invalid_argument("missing checkpoints_file");
  std::ifstream checkpoints_file_stream(checkpoints_file_path);
  if (!checkpoints_file_stream) throw std::runtime_error("cannot open " + checkpoints_file_path);
  const std::string checkpoints_file_data{std::istreambuf_iterator<char>(checkpoints_file_stream), std::istreambuf_iterator<char>()};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    Components::Checker(limits, io.readClocks().steady_ns), policy, selector, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_ego = nh.subscribe("/localization/ego", 1, &RosCallbacks::onEgo, &callbacks);
  const auto sub_competition = nh.subscribe("/vehicle/competition_status", 1, &RosCallbacks::onCompetition, &callbacks);
  const auto sub_objects = nh.subscribe("/objects", 1, &RosCallbacks::onObjects, &callbacks);
  const auto sub_tf = nh.subscribe("/tf", 1, &RosCallbacks::onTf, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
