#include <control/template_components.hpp>
#include <node_pipeline/ros.hpp>
namespace pkg = control;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
using IO = np::RosIO<Message>;
struct RosHeaderInjector {
  void injectHeader(morai_msgs::CtrlCmd&, const np::Context<3>&) const {}
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onPath(const interfaces::Path::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Path>{}, np::retainMessage(msg)); }
  void onEgo(const geometry_msgs::PoseStamped::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Ego>{}, np::retainMessage(msg)); }
  void onCompetition(const interfaces::CompetitionVehicleStatus::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Competition>{}, np::retainMessage(msg)); }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/local_path/velocity_injected"), pkg::Input::Path}, {nh.resolveName("/localization/ego"), pkg::Input::Ego}, {nh.resolveName("/vehicle/competition_status"), pkg::Input::Competition}},
    std::map<std::string, pkg::Input>{{"path", pkg::Input::Path}, {"ego", pkg::Input::Ego}, {"competition", pkg::Input::Competition}},
    std::map<std::string, pkg::CoreId>{}, selector, policy);
  std::array<np::HealthLimits, 3> limits;
  limits.fill(settings.health);
  limits[0].frame = "base_link";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/path/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/path/receive_timeout_sec", 0.3);
  limits[1].frame = "map";
  limits[1].stale_ns = np::readDurationNs(params, "inputs/ego/stale_after_sec", 0.3);
  limits[1].receive_timeout_ns = np::readDurationNs(params, "inputs/ego/receive_timeout_sec", 0.3);
  limits[2].frame = "base_link";
  limits[2].stale_ns = np::readDurationNs(params, "inputs/competition/stale_after_sec", 0.3);
  limits[2].receive_timeout_ns = np::readDurationNs(params, "inputs/competition/receive_timeout_sec", 0.3);
  IO io{nh.advertise<morai_msgs::CtrlCmd>("/control_cmd", 1)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    Components::Checker(limits, io.readClocks().steady_ns), policy, selector, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_path = nh.subscribe("/local_path/velocity_injected", 1, &RosCallbacks::onPath, &callbacks);
  const auto sub_ego = nh.subscribe("/localization/ego", 1, &RosCallbacks::onEgo, &callbacks);
  const auto sub_competition = nh.subscribe("/vehicle/competition_status", 1, &RosCallbacks::onCompetition, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
