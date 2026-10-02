#include <control/components.hpp>
#include <control/tracking/ros_parameters.hpp>
#include <node_pipeline/ros.hpp>
#include <memory>
namespace pkg = control;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
struct Activity { std::int64_t last_healthy_output_ns = 0; };
struct IO : np::RosIO<Message> {
  std::shared_ptr<Activity> activity;
  double safe_brake_command;
  IO(ros::Publisher publisher, std::shared_ptr<Activity> state, double brake)
    : np::RosIO<Message>{publisher}, activity(std::move(state)), safe_brake_command(brake) {}
  void publish(const Message& message) const {
    np::RosIO<Message>::publish(message);
    activity->last_healthy_output_ns = readClocks().steady_ns;
  }
  void publishSafe() const {
    Message command;
    command.longlCmdType = 1;
    command.brake = safe_brake_command;
    np::RosIO<Message>::publish(command);
  }
};
struct RosHeaderInjector {
  void injectHeader(morai_msgs::CtrlCmd&, const np::Context<3>&) const {}
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  IO io;
  std::int64_t watchdog_timeout_ns;
  bool periodic;
  std::uint64_t last_path_stamp = 0;
  // 계산 오류는 안전 명령 후 노드를 종료한다. 같은 계산을 재시도하지 않는다.
  void failed(const char* reason) {
    io.publishSafe();
    ROS_FATAL("control calculation failed: %s", reason);
    ros::shutdown();
  }
  void onPath(const interfaces::Path::ConstPtr& msg) {
    if (!periodic && last_path_stamp && msg->header.stamp.toNSec() <= last_path_stamp) {
      if (msg->header.stamp.toNSec() < last_path_stamp) io.publishSafe();
      return;
    }
    try {
      const auto report = node.onInput(pkg::InputTag<pkg::Input::Path>{}, np::retainMessage(msg));
      if (report.status == np::ProcessStatus::Published) last_path_stamp = msg->header.stamp.toNSec();
      else if (report.status == np::ProcessStatus::NoCore || report.status == np::ProcessStatus::Rejected) io.publishSafe();
    } catch (const std::exception& error) { failed(error.what()); }
      catch (...) { failed("unknown exception"); }
  }
  void onEgo(const nav_msgs::Odometry::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Ego>{}, np::retainMessage(msg)); }
  void onCompetition(const interfaces::CompetitionVehicleStatus::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Competition>{}, np::retainMessage(msg)); }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& msg) {
    node.onSection(np::Section{true, msg->state, static_cast<std::int64_t>(msg->header.stamp.toNSec()), msg->header.frame_id});
  }
  void onPeriodic(const ros::SteadyTimerEvent&) {
    try {
      const auto report = node.onPeriodic();
      if (report.status == np::ProcessStatus::NoCore || report.status == np::ProcessStatus::Rejected) io.publishSafe();
    } catch (const std::exception& error) { failed(error.what()); }
      catch (...) { failed("unknown exception"); }
  }
  void onWatchdog(const ros::SteadyTimerEvent&) {
    if (io.readClocks().steady_ns - io.activity->last_healthy_output_ns > watchdog_timeout_ns) io.publishSafe();
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh, params("~"), tracking_params(params, "tracking");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/local_path/velocity_injected"), pkg::Input::Path}, {nh.resolveName("/localization/ego"), pkg::Input::Ego}, {nh.resolveName("/vehicle/competition_status"), pkg::Input::Competition}},
    std::map<std::string, pkg::Input>{{"path", pkg::Input::Path}, {"ego", pkg::Input::Ego}, {"competition", pkg::Input::Competition}},
    std::map<std::string, pkg::CoreId>{{"path_tracking.hpp", pkg::CoreId::PathTracking}}, selector, policy);
  policy.priority.fill(100);
  const auto parameters = pkg::tracking::loadParameters(tracking_params);
  double safe_brake;
  params.param("safe_brake_command", safe_brake, 0.5);
  if (!std::isfinite(safe_brake) || safe_brake <= 0 || safe_brake > 1)
    throw std::invalid_argument("safe_brake_command must be in (0, 1]");
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
  IO io(nh.advertise<Message>("/control_cmd", 1), std::make_shared<Activity>(), safe_brake);
  io.activity->last_healthy_output_ns = io.readClocks().steady_ns;
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    Components::Checker(limits, io.readClocks().steady_ns, {parameters.longitudinal_mpc.maximum_speed_mps}),
    policy, selector, Components::CoreSet(parameters), {}, io);
  const auto timeout = std::min({limits[0].receive_timeout_ns, limits[1].receive_timeout_ns, limits[2].receive_timeout_ns});
  RosCallbacks callbacks{node, io, timeout, settings.execution.mode == np::ExecutionMode::Periodic};
  const auto sub_path = nh.subscribe("/local_path/velocity_injected", 1, &RosCallbacks::onPath, &callbacks);
  const auto sub_ego = nh.subscribe("/localization/ego", 1, &RosCallbacks::onEgo, &callbacks);
  const auto sub_competition = nh.subscribe("/vehicle/competition_status", 1, &RosCallbacks::onCompetition, &callbacks);
  const auto sub_section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  const auto watchdog = nh.createSteadyTimer(ros::WallDuration(np::readDurationNs(params, "watchdog_period_sec", 0.02) * 1e-9),
                                            &RosCallbacks::onWatchdog, &callbacks);
  ros::spin();
}
