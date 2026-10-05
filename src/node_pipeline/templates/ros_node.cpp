// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
// 원래 ROS 메시지 타입으로 공개 라이브러리를 조합하는 실행 패키지 템플릿다.
#include "template_components.hpp"
#include <node_pipeline/ros.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <interfaces/SectionAnnotation.h>
namespace np = node_pipeline;
namespace ex = pipeline_template;
// TODO: 구독·발행할 실제 ROS 타입, 토픽 이름, 출력 필드와 header 기준을 지정한다.
using Message = geometry_msgs::Vector3Stamped;
// 메시지 변환은 하지 않는다. 원래 ROS 타입의 필드만 지정한다.
struct RosAccess {
  static std::int64_t stamp(const Message& message) { return message.header.stamp.toNSec(); }
  static const std::string& frame(const Message& message) { return message.header.frame_id; }
  static double& value(Message& message) { return message.vector.x; }
  static double value(const Message& message) { return message.vector.x; }
};
using Components = ex::Components<Message, RosAccess>;
using IO = np::RosIO<Message>;
// TODO: 목표 발행 메시지의 헤더 기준을 지정한다. payload는 Core가 완성한다.
struct RosHeaderInjector {
  void injectHeader(Message& message, const np::Context<2>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = context.frame;
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
// ROS 연결 adapter에는 구독 메시지의 변환이나 계산 구현을 넣지 않는다.
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
  ros::init(argc, argv, "node_pipeline_template");
  ros::NodeHandle nh, params("~");
  const auto primary_topic = nh.resolveName("/node_pipeline_template/primary");
  const auto auxiliary_topic = nh.resolveName("/node_pipeline_template/auxiliary");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  // 설정 로딩과 검증은 라이브러리에서 수행한다. 템플릿는 실제 이름과 ID만 연결한다.
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, ex::Input>{{primary_topic, ex::Input::Primary}, {auxiliary_topic, ex::Input::Auxiliary}},
    std::map<std::string, ex::Input>{{"primary", ex::Input::Primary}, {"auxiliary", ex::Input::Auxiliary}},
    std::map<std::string, ex::CoreId>{}, selector, policy);
  ros::NodeHandle stage1_params(params, "stage1");
  np::loadStageConfig(stage1_params,
    std::map<std::string, ex::Input>{{"primary", ex::Input::Primary}, {"auxiliary", ex::Input::Auxiliary}},
    std::map<std::string, ex::CoreId>{{"scalar_core.hpp", ex::CoreId::Primary}, {"auxiliary_core.hpp", ex::CoreId::Auxiliary}}, selector, policy);
  IO io{nh.advertise<Message>("/node_pipeline_template/output", 1)};
  // 실행 노드는 helper 객체를 구성한다. 큐·조립·검사·라우팅을 다시 구현하지 않는다.
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    {Components::stage(Components::Checker(settings.health, io.readClocks().steady_ns), policy, selector)}, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto primary = nh.subscribe(primary_topic, 1, &RosCallbacks::onPrimary, &callbacks);
  const auto auxiliary = nh.subscribe(auxiliary_topic, 1, &RosCallbacks::onAuxiliary, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &RosCallbacks::onSection, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  // 단일 callback queue로 실행하여 helper와 core 상태를 직렬로 사용한다.
  ros::spin();
}
