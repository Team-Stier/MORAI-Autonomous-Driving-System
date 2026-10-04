#include <speed_filter/components.hpp>
#include <speed_filter/config.hpp>
#include <node_pipeline/ros.hpp>
#include <fstream>
#include <iterator>
namespace pkg = speed_filter;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
  // Header 없는 명령은 원래 생성 시각을 만들지 않는다.
  static std::int64_t stamp(const morai_msgs::CtrlCmd&) { return 0; }
  static std::string frame(const morai_msgs::CtrlCmd&) { return ""; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
using IO = np::RosIO<Message>;
struct RosHeaderInjector {
  void injectHeader(morai_msgs::CtrlCmd&, const np::Context<2>&) const {}
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onCommand(const morai_msgs::CtrlCmd::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Command>{}, np::retainMessage(msg)); }
  void onCompetition(const interfaces::CompetitionVehicleStatus::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Competition>{}, np::retainMessage(msg)); }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& msg) {
    node.onSection(np::Section{true, msg->state, static_cast<std::int64_t>(msg->header.stamp.toNSec()), msg->header.frame_id});
  }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "speed_filter_node");
  ros::NodeHandle nh, params("~"), stage_params(params, "stage1");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const std::map<std::string, pkg::Input> inputs{
    {"command", pkg::Input::Command}, {"competition", pkg::Input::Competition}};
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/control_cmd"), pkg::Input::Command}, {nh.resolveName("/vehicle/competition_status"), pkg::Input::Competition}},
    inputs, std::map<std::string, pkg::CoreId>{}, selector, policy);
  np::loadStageConfig(stage_params, inputs,
    std::map<std::string, pkg::CoreId>{{"speed_limit60.hpp", pkg::CoreId::SpeedLimit60},
                                     {"no_speed_limit.hpp", pkg::CoreId::NoSpeedLimit}}, selector, policy);
  const auto speed_limit60 = pkg::loadSpeedFilterParameters(params, "speed_limit60", 60.0);
  const auto no_speed_limit = pkg::loadSpeedFilterParameters(params, "no_speed_limit", 0.0);
  std::array<np::HealthLimits, 2> limits;
  limits.fill(settings.health);
  limits[0].frame = "map";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/command/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/command/receive_timeout_sec", 0.3);
  limits[1].frame = "base_link";
  limits[1].stale_ns = np::readDurationNs(params, "inputs/competition/stale_after_sec", 0.3);
  limits[1].receive_timeout_ns = np::readDurationNs(params, "inputs/competition/receive_timeout_sec", 0.3);
  limits[0].check_stamp = false;
  limits[0].check_skew = false;
  IO io{nh.advertise<morai_msgs::CtrlCmd>("/morai_udp_bridge/control_cmd", 1)};
  std::string map_file_path;
  if (!params.getParam("map_file", map_file_path)) throw std::invalid_argument("missing map_file");
  std::ifstream map_file_stream(map_file_path);
  if (!map_file_stream) throw std::runtime_error("cannot open " + map_file_path);
  const std::string map_file_data{std::istreambuf_iterator<char>(map_file_stream), std::istreambuf_iterator<char>()};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    std::vector<Components::Stage>{np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Data>(
      Components::Checker(limits, io.readClocks().steady_ns), policy, selector, pkg::processCore)},
    pkg::CoreSet{pkg::SpeedLimit60{speed_limit60}, pkg::NoSpeedLimit{no_speed_limit}}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_command = nh.subscribe("/control_cmd", 1, &RosCallbacks::onCommand, &callbacks);
  const auto sub_competition = nh.subscribe("/vehicle/competition_status", 1, &RosCallbacks::onCompetition, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
