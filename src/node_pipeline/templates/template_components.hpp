// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
#pragma once
// 템플릿는 공통 구현을 재작성하지 않고 공개 라이브러리를 조합한다.
#include <node_pipeline/pipeline.hpp>
#include "core_dispatch.hpp"
namespace pipeline_template {
namespace np = node_pipeline;
// enum 순서는 입력 tuple의 순서와 같다.
// TODO: 입력 ID, 메시지 타입 pack, 필드 Access, Validator와 policy·selector를 지정한다.
enum class Input { Primary, Auxiliary };
using Kind = np::Kind;
template<class Enum> constexpr std::size_t index(Enum value) { return static_cast<std::size_t>(value); }
using Primary = std::integral_constant<Input, Input::Primary>;
using Auxiliary = std::integral_constant<Input, Input::Auxiliary>;
// 노드별 메시지 필드 접근만 남긴다.
struct DemoAccess {
  static std::int64_t stamp(const DemoMessage& message) { return message.stamp_ns; }
  static const std::string& frame(const DemoMessage& message) { return message.frame; }
  static double& value(DemoMessage& message) { return message.value; }
  static double value(const DemoMessage& message) { return message.value; }
};
// 값의 의미와 필드 접근은 노드에 전문화한다.
template<class Access> struct ScalarValidator {
  template<class Message> void operator()(const Message& message, const np::HealthLimits& limits, std::vector<Kind>& issues) const {
    if (Access::frame(message) != limits.frame) issues.push_back(Kind::FrameMismatch);
    np::checkNumber(Access::value(message), limits, issues);
  }
};
// 실제 메시지 타입들을 <T...>로 지정한다. 템플릿는 두 토픽의 타입이 같다.
template<class Message, class Access, class OutputMessage = Message> struct Components {
  using Buffer = np::InputBuffer<Input, Access, Message, Message>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 2, ScalarValidator<Access>>;
  using Policy = np::InputPolicy<Input, np::ClampUpper<Access>, np::ClampLower<Access>>;
  using Selector = np::CoreSelector<Input, CoreId>;
  using CoreSet = pipeline_template::CoreSet<Message, Access, OutputMessage>;
  using Data = np::Data<Message, Message>;
  using Stage = np::Stage<CoreSet, typename Buffer::ContextType>;
  static Stage stage(Checker checker, Policy policy, Selector selector) {
    return np::makeStage<CoreSet, typename Buffer::ContextType, Data>(std::move(checker), std::move(policy),
      std::move(selector), processCore<Message, Access, OutputMessage>);
  }
  // strategy 객체와 한도만 연결한다.
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  // 템플릿 기본 선택 표다. ROS config는 이 표를 덮어쓴다.
  static Selector selector() {
    Selector result;
    result.section_core_map = {{0, CoreId::Primary}};
    result.health_core_map = {{{{Input::Primary, Kind::Missing}}, 100, CoreId::Auxiliary},
                             {{{Input::Auxiliary, Kind::Missing}}, 90, CoreId::Primary}};
    return result;
  }
  template<class IO, class Builder> using Node = np::PipelineNode<Buffer, Assembler, CoreSet, Builder, IO>;
};
// ROS 없는 가이드에서도 payload를 바꾸지 않고 메타데이터만 주입한다.
struct DemoHeaderInjector {
  void injectHeader(DemoMessage& message, const np::Context<2>& context) const {
    message.stamp_ns = context.stamp_ns;
    message.frame = context.frame;
  }
};
}
