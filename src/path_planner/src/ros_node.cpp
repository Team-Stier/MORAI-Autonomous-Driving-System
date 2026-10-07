#include <path_planner/components.hpp>
#include <path_planner/config.hpp>
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
  void injectHeader(interfaces::Path& message, const Components::Buffer::ContextType& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "map";
    for (auto& point : message.points) point.pose.header = message.header;
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onEgo(const nav_msgs::Odometry::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Ego>{}, np::retainMessage(msg)); }
  void onCompetition(const interfaces::CompetitionVehicleStatus::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Competition>{}, np::retainMessage(msg)); }
  void onObjects(const interfaces::Objects::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Objects>{}, np::retainMessage(msg)); }
  void onTf(const tf2_msgs::TFMessage::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Tf>{}, np::retainMessage(msg)); }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& msg) {
    node.onSection(np::Section{true, msg->state, static_cast<std::int64_t>(msg->header.stamp.toNSec()), msg->header.frame_id});
    node.onInput(pkg::InputTag<pkg::Input::Section>{}, np::retainMessage(msg));
  }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const std::map<std::string, pkg::Input> topics{{nh.resolveName("/localization/ego"), pkg::Input::Ego}, {nh.resolveName("/vehicle/competition_status"), pkg::Input::Competition}, {nh.resolveName("/objects"), pkg::Input::Objects}, {nh.resolveName("/tf"), pkg::Input::Tf}};
  const std::map<std::string, pkg::Input> inputs{{"ego", pkg::Input::Ego}, {"competition", pkg::Input::Competition}, {"objects", pkg::Input::Objects}, {"tf", pkg::Input::Tf}, {"section", pkg::Input::Section}};
  const auto settings = np::loadRosConfig(nh, params,
    topics, inputs, std::map<std::string, pkg::FirstCoreId>{}, selector.stage1, policy);
  ros::NodeHandle stage1_params(params, "stage1"), stage2_params(params, "stage2"), stage3_params(params, "stage3");
  auto stage2_policy = Components::Stage2Policy{{settings.health.upper}, {settings.health.lower}};
  auto stage3_policy = Components::Stage3Policy{{settings.health.upper}, {settings.health.lower}};
  np::loadStageConfig(stage1_params, inputs,
    std::map<std::string, pkg::FirstCoreId>{{"global_path.hpp", pkg::FirstCoreId::GlobalPath}}, selector.stage1, policy);
  np::loadStageConfig(stage2_params,
    std::map<std::string, pkg::Stage2Input>{{"ego", pkg::Stage2Input::Ego}, {"competition", pkg::Stage2Input::Competition},
      {"objects", pkg::Stage2Input::Objects}, {"tf", pkg::Stage2Input::Tf},
      {"section", pkg::Stage2Input::Section}, {"route", pkg::Stage2Input::Route}},
    std::map<std::string, pkg::SecondCoreId>{{"prediction_pass_through.hpp", pkg::SecondCoreId::PredictionPassThrough}}, selector.stage2, stage2_policy);
  np::loadStageConfig(stage3_params,
    std::map<std::string, pkg::Stage3Input>{{"ego", pkg::Stage3Input::Ego}, {"competition", pkg::Stage3Input::Competition},
      {"objects", pkg::Stage3Input::Objects}, {"tf", pkg::Stage3Input::Tf}, {"section", pkg::Stage3Input::Section},
      {"route", pkg::Stage3Input::Route}, {"predictions", pkg::Stage3Input::Predictions}},
    std::map<std::string, pkg::ThirdCoreId>{{"local_path.hpp", pkg::ThirdCoreId::LocalPath}}, selector.stage3, stage3_policy);
  std::array<np::HealthLimits, Components::Buffer::count> limits;
  limits.fill(settings.health);
  limits[0].frame = "map";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/ego/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/ego/receive_timeout_sec", 0.3);
  limits[1].frame = "base_link";
  limits[1].stale_ns = np::readDurationNs(params, "inputs/competition/stale_after_sec", 0.3);
  limits[1].receive_timeout_ns = np::readDurationNs(params, "inputs/competition/receive_timeout_sec", 0.3);
  // #DOWNSTREAM_CONTRACT
  // perception의 /objects는 base_link_level이다. 같은 뒷차축 원점과 yaw를
  // 공유하더라도 차량 roll/pitch가 제거되어 차체 고정 base_link와 좌표축이 다르다.
  // 현재 PredictionPassThrough는 좌표와 header를 그대로 복사하므로 예측 배열도
  // base_link_level을 유지한다. 현재 LocalPath는 객체 위치를 실제 장애물 회피
  // 계산에 사용하지 않고 map 중심선 경로만 계산하므로 변환을 추가하지 않는다.
  // 향후 tracking·prediction·회피·충돌 검사는 객체를 base_link라고 가정하면 안 된다.
  // Objects.header.frame_id를 확인하고 다른 frame이 필요하면 해당 알고리즘 입력
  // 경계에서 메시지 timestamp에 맞는 명시적인 TF 변환을 수행해야 한다.
  // 헤더 이름만 바꾸거나 최신 차량 자세로 역회전하면 frame 또는 시간 불일치가 생긴다.
  // competition은 실제 차체 상태이므로 위 슬롯 1의 base_link 계약은 그대로 둔다.
  limits[2].frame = pkg::kObjectsFrame;
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
  const auto global_path = pkg::readGlobalPathParameters(map_file_data, checkpoints_file_data);
  const auto local_path = pkg::loadLocalPathParameters(stage3_params, global_path.map);
  std::array<np::HealthLimits, 6> stage2_limits{{limits[0], limits[1], limits[2], limits[3], limits[4], settings.health}};
  std::array<np::HealthLimits, 7> stage3_limits{{limits[0], limits[1], limits[2], limits[3], limits[4], settings.health, settings.health}};
  // #DOWNSTREAM_CONTRACT
  // 예측 원소는 원본 Objects.header와 좌표를 유지하므로 원본과 같은 frame을 검사한다.
  // 여기서 base_link를 기대하면 정상적인 수평 perception 결과가 제거되어 후속
  // LocalPath 호출이 막힌다. 검사 이름을 맞추는 것이며 객체를 회전하는 단계가 아니다.
  stage3_limits[6].frame = pkg::kObjectsFrame;
  using Context = Components::Buffer::ContextType;
  const auto started = io.readClocks().steady_ns;
  std::vector<np::Stage<pkg::CoreSet, Context>> stages{
    np::makeStage<pkg::CoreSet, Context, pkg::Data>(Components::Checker(limits, started), policy,
      selector.stage1, pkg::retainInputs(pkg::processStage1)),
    np::makeStage<pkg::CoreSet, Context, pkg::Stage2Data>(Components::Stage2Checker(stage2_limits, started), stage2_policy,
      selector.stage2, pkg::retainInputs(pkg::processStage2)),
    np::makeStage<pkg::CoreSet, Context, pkg::Stage3Data>(Components::Stage3Checker(stage3_limits, started), stage3_policy,
      selector.stage3, pkg::processStage3)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    std::move(stages),
    pkg::CoreSet{pkg::GlobalPath(global_path,
        nh.advertise<nav_msgs::Path>("/visualizer/global_path", 1, true)), pkg::PredictionPassThrough{}, pkg::LocalPath(local_path)}, {}, io);
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
