// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
#include "template_components.hpp"
#include <node_pipeline/ros.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <interfaces/SectionAnnotation.h>
namespace np = node_pipeline;
namespace ex = pipeline_two_core_template;
// TODO: 실제 구독·중간·발행 메시지 타입과 필드 접근을 지정한다.
using Message = geometry_msgs::Vector3Stamped;
struct RosAccess {
  static std::int64_t stamp(const Message& message) { return message.header.stamp.toNSec(); }
  static const std::string& frame(const Message& message) { return message.header.frame_id; }
  static double& value(Message& message) { return message.vector.x; }
  static double value(const Message& message) { return message.vector.x; }
};
using Components = ex::Components<Message, RosAccess>;
using IO = np::RosIO<Message>;
// TODO: 최종 메시지의 발행 헤더 기준을 지정한다.
struct RosHeaderInjector {
  void injectHeader(Message& result, const np::Context<2>& context) const {
    result.header.stamp.fromNSec(context.stamp_ns);
    result.header.frame_id = context.frame;
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onPrimary(const Message::ConstPtr& message) { node.onInput(ex::Primary{}, np::retainMessage(message)); }
  void onAuxiliary(const Message::ConstPtr& message) { node.onInput(ex::Auxiliary{}, np::retainMessage(message)); }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& message) {
    node.onSection(np::Section{true, message->state, static_cast<std::int64_t>(message->header.stamp.toNSec()), message->header.frame_id});
  }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "node_pipeline_two_core_template");
  ros::NodeHandle nh, params("~");
  // TODO: 실제 토픽을 지정하고 config/template.yaml을 private namespace에 로드한다.
  const auto primary_topic = nh.resolveName("/node_pipeline_two_core_template/primary");
  const auto auxiliary_topic = nh.resolveName("/node_pipeline_two_core_template/auxiliary");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const std::map<std::string, ex::Input> topics{{primary_topic, ex::Input::Primary}, {auxiliary_topic, ex::Input::Auxiliary}};
  const std::map<std::string, ex::Input> inputs{{"primary", ex::Input::Primary}, {"auxiliary", ex::Input::Auxiliary}};
  // 공통 실행·health·policy 설정은 private namespace에서 읽는다.
  const auto settings = np::loadRosConfig(nh, params, topics, inputs,
    std::map<std::string, ex::FirstCoreId>{}, selector.core1, policy);
  // 단계별 namespace에서는 Core 매핑을 읽는다. 공통 policy는 복사본으로 보존한다.
  ros::NodeHandle core1_params(params, "core1"), core2_params(params, "core2");
  auto mapping_policy = policy;
  np::loadRosConfig(nh, core1_params, topics, inputs,
    std::map<std::string, ex::FirstCoreId>{{"first_core.hpp", ex::FirstCoreId::First}}, selector.core1, mapping_policy);
  np::loadRosConfig(nh, core2_params, topics, inputs,
    std::map<std::string, ex::SecondCoreId>{{"second_core.hpp", ex::SecondCoreId::Second}}, selector.core2, mapping_policy);
  IO io{nh.advertise<Message>("/node_pipeline_two_core_template/output", 1)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    Components::Checker(settings.health, io.readClocks().steady_ns), policy, selector, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto primary = nh.subscribe(primary_topic, 1, &RosCallbacks::onPrimary, &callbacks);
  const auto auxiliary = nh.subscribe(auxiliary_topic, 1, &RosCallbacks::onAuxiliary, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  // 단일 callback queue에서 두 Core와 발행을 순차 실행한다.
  ros::spin();
}
