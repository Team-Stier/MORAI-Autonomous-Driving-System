#include <lidar_leveler/components.hpp>
#include <lidar_leveler/header_injector.hpp>
#include <node_pipeline/ros.hpp>
namespace pkg = lidar_leveler;
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
using RosHeaderInjector = pkg::HeaderInjector;
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Cloud>{}, np::retainMessage(msg)); }
  void onCompetition(const interfaces::CompetitionVehicleStatus::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Competition>{}, np::retainMessage(msg)); }
  void onTfStatic(const tf2_msgs::TFMessage::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::TfStatic>{}, np::retainMessage(msg)); }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_leveler_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/velodyne_points"), pkg::Input::Cloud}, {nh.resolveName("/vehicle/competition_status"), pkg::Input::Competition}, {nh.resolveName("/tf_static"), pkg::Input::TfStatic}},
    std::map<std::string, pkg::Input>{{"cloud", pkg::Input::Cloud}, {"competition", pkg::Input::Competition}, {"tf_static", pkg::Input::TfStatic}},
    std::map<std::string, pkg::CoreId>{}, selector, policy);
  ros::NodeHandle stage1_params(params, "stage1");
  np::loadStageConfig(stage1_params,
    std::map<std::string, pkg::Input>{{"cloud", pkg::Input::Cloud}, {"competition", pkg::Input::Competition}, {"tf_static", pkg::Input::TfStatic}},
    std::map<std::string, pkg::CoreId>{{"pass_through.hpp", pkg::CoreId::PassThrough},
      {"roll_pitch_leveler.hpp", pkg::CoreId::RollPitchLeveler}}, selector, policy);
  std::array<np::HealthLimits, 3> limits;
  limits.fill(settings.health);
  limits[0].frame = "velodyne";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/cloud/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/cloud/receive_timeout_sec", 0.3);
  limits[1].frame = "base_link";
  limits[1].stale_ns = np::readDurationNs(params, "inputs/competition/stale_after_sec", 0.3);
  limits[1].receive_timeout_ns = np::readDurationNs(params, "inputs/competition/receive_timeout_sec", 0.3);
  limits[2].frame = "map";
  limits[2].stale_ns = np::readDurationNs(params, "inputs/tf_static/stale_after_sec", 0.3);
  limits[2].receive_timeout_ns = np::readDurationNs(params, "inputs/tf_static/receive_timeout_sec", 0.3);
  limits[2].check_stamp = false;
  limits[2].check_skew = false;
  limits[2].check_receive_timeout = false;
  IO io{nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/leveled", 1)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    {np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Data>(
      Components::Checker(limits, io.readClocks().steady_ns), policy, selector, pkg::processCore)},
    pkg::CoreSet{pkg::PassThrough{}, pkg::RollPitchLeveler{}}, RosHeaderInjector{settings.health.frame}, io);
  RosCallbacks callbacks{node};
  const auto sub_cloud = nh.subscribe("/velodyne_points", 1, &RosCallbacks::onCloud, &callbacks);
  const auto sub_competition = nh.subscribe("/vehicle/competition_status", 1, &RosCallbacks::onCompetition, &callbacks);
  const auto sub_tf_static = nh.subscribe("/tf_static", 1, &RosCallbacks::onTfStatic, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
