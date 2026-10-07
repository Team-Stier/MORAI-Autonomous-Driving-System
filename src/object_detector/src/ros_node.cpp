#include <object_detector/components.hpp>
#include <object_detector/config.hpp>
#include <object_detector/frame_contract.hpp>
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
using RosHeaderInjector = pkg::HeaderInjector;
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
    std::map<std::string, pkg::FirstCoreId>{}, selector.stage1, policy);
  ros::NodeHandle stage1_params(params, "stage1"), stage2_params(params, "stage2");
  auto stage2_policy = Components::Stage2Policy{{settings.health.upper}, {settings.health.lower}};
  np::loadStageConfig(stage1_params, inputs,
    std::map<std::string, pkg::FirstCoreId>{{"dbscan.hpp", pkg::FirstCoreId::Dbscan}}, selector.stage1, policy);
  np::loadStageConfig(stage2_params, std::map<std::string, pkg::Stage2Input>{{"objects", pkg::Stage2Input::Objects}},
    std::map<std::string, pkg::SecondCoreId>{{"objects_pass_through.hpp", pkg::SecondCoreId::ObjectsPassThrough}}, selector.stage2, stage2_policy);
  const auto dbscan_parameters = pkg::loadDbscanParameters(params);
  std::array<np::HealthLimits, 1> limits;
  limits.fill(settings.health);
  // #BASE_LINK_LEVEL
  // /velodyne_points/leveled는 lidar_leveler가 장착 TF와 roll/pitch 보정을 한 번
  // 적용한 base_link_level 점군이다. 원점과 yaw는 차체 base_link와 같지만
  // roll/pitch가 제거되므로 두 frame의 좌표값을 서로 같은 것으로 볼 수 없다.
  // ROI x/y/z 범위, VoxelGrid, DBSCAN 거리와 OBB 중심·방향은 이 수평 frame에서
  // 계산한다. 여기서 mounting offset이나 차량 자세를 다시 적용하면 이중 보정이 된다.
  // 잘못 base_link로 표시된 cloud는 FrameMismatch로 거부한다. 헤더만 수정하여
  // 통과시키지 말고 실제 입력을 계약에 맞게 제공해야 한다. 출력도 같은 frame이며
  // 다른 frame이 필요한 소비 모듈이 자신의 입력 경계에서 timestamp에 맞게 변환한다.
  limits[0].frame = pkg::kPerceptionFrame;
  limits[0].stale_ns = np::readDurationNs(params, "inputs/cloud/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/cloud/receive_timeout_sec", 0.3);
  IO io{nh.advertise<interfaces::Objects>("/objects", 1)};
  const auto stage2_limits = limits;
  using Context = Components::Buffer::ContextType;
  const auto started = io.readClocks().steady_ns;
  std::vector<np::Stage<pkg::CoreSet, Context>> stages{
    np::makeStage<pkg::CoreSet, Context, pkg::Data>(Components::Checker(limits, started), policy,
      selector.stage1, pkg::processStage1),
    np::makeStage<pkg::CoreSet, Context, pkg::Stage2Data>(Components::Stage2Checker(stage2_limits, started), stage2_policy,
      selector.stage2, pkg::processStage2)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    std::move(stages), pkg::CoreSet{pkg::Dbscan(dbscan_parameters), pkg::ObjectsPassThrough{}}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_cloud = nh.subscribe("/velodyne_points/leveled", 1, &RosCallbacks::onCloud, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
