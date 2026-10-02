// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
#pragma once
#include <node_pipeline/components.hpp>
#include "types.hpp"
#include "core_set.hpp"
namespace pipeline_two_core_template {
namespace np = node_pipeline;
// TODO: 입력 ID와 메시지 타입 pack을 같은 순서로 지정한다.
enum class Input { Primary, Auxiliary };
using Primary = std::integral_constant<Input, Input::Primary>;
using Auxiliary = std::integral_constant<Input, Input::Auxiliary>;
// 각 단계에 공통 Selector를 사용하고 두 단계 모두 선택된 경우에만 호출한다.
struct Selector {
  np::CoreSelector<Input, FirstCoreId> core1;
  np::CoreSelector<Input, SecondCoreId> core2;
  template<class Health> np::Selection<CoreIds> select(std::uint8_t section, const Health& health) const {
    const auto first = core1.select(section, health);
    const auto second = core2.select(section, health);
    return {first.selected && second.selected, {first.id, second.id}};
  }
};
struct DemoAccess {
  static std::int64_t stamp(const DemoMessage& message) { return message.stamp_ns; }
  static const std::string& frame(const DemoMessage& message) { return message.frame; }
  static double& value(DemoMessage& message) { return message.value; }
  static double value(const DemoMessage& message) { return message.value; }
};
// TODO: 구독 메시지의 필드 계약을 검사한다.
template<class Access> struct ScalarValidator {
  template<class Message> void operator()(const Message& message, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const {
    if (Access::frame(message) != limits.frame) issues.push_back(np::Kind::FrameMismatch);
    np::checkNumber(Access::value(message), limits, issues);
  }
};
// 예제는 입력·중간 메시지·최종 메시지에 같은 실제 메시지 타입을 사용한다.
template<class Message, class Access> struct Components {
  using Buffer = np::InputBuffer<Input, Access, Message, Message>;
  using Assembler = np::SnapshotAssembler<Buffer, Access>;
  using Checker = np::HealthChecker<Input, 2, ScalarValidator<Access>>;
  using Policy = np::InputPolicy<Input, np::ClampUpper<Access>, np::ClampLower<Access>>;
  using Selector = pipeline_two_core_template::Selector;
  using CoreSet = pipeline_two_core_template::CoreSet<Message, Access>;
  static Policy policy(const np::HealthLimits& limits) { return {{limits.upper}, {limits.lower}}; }
  static Selector selector() {
    Selector result;
    result.core1.section_core_map = {{0, FirstCoreId::First}};
    result.core2.section_core_map = {{0, SecondCoreId::Second}};
    return result;
  }
  template<class IO, class HeaderInjector> using Node = np::PipelineNode<Buffer, Assembler, Checker, Policy, Selector, CoreSet, HeaderInjector, IO>;
};
struct DemoHeaderInjector {
  void injectHeader(DemoMessage& result, const np::Context<2>& context) const {
    result.stamp_ns = context.stamp_ns;
    result.frame = context.frame;
  }
};
}
