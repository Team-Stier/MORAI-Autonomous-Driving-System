// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
// ROS 없이 공개 라이브러리를 조합하는 템플릿다.
#include "template_components.hpp"
#include <cassert>
#include <iostream>
namespace np = node_pipeline;
namespace ex = pipeline_template;
// 노드가 구독하는 두 upstream의 실제 타입을 지정한다.
using Components = ex::Components<ex::DemoMessage, ex::DemoAccess>;
// 발행 기록과 시계는 IO 복사본 사이에 공유한다.
struct State { np::Clocks clocks{1000000000, 1000000000}; std::vector<ex::DemoMessage> published; };
struct IO {
  std::shared_ptr<State> state;
  np::Clocks readClocks() const { return state->clocks; }
  void publish(const ex::DemoMessage& message) { state->published.push_back(message); }
  void reportHealth(const np::Health<ex::Input, 2>&, std::size_t) const {}
  void report(const np::ProcessReport&) const noexcept {}
};
int main() {
  // 공개 API로 helper와 설정을 조합한다.
  auto state = std::make_shared<State>();
  IO io{state};
  np::HealthLimits limits;
  Components::Node<IO, ex::DemoHeaderInjector> node(
    {np::ExecutionMode::OnInput, ex::Input::Primary, 10}, Components::Buffer(8), {},
    Components::Checker(limits, state->clocks.steady_ns), Components::policy(limits),
    Components::selector(), {}, {}, io);
  // 정상 보조 입력을 준비하고 기준 입력의 범위 초과를 보정한다.
  ex::DemoMessage auxiliary{0.5, state->clocks.ros_ns, "map"};
  node.onInput(ex::Auxiliary{}, auxiliary);
  ex::DemoMessage primary{2, state->clocks.ros_ns, "map"};
  const auto report = node.onInput(ex::Primary{}, primary);
  // 기존 자체 점검을 새 공개 API에 맞춘다. 원본 메시지는 바뀌지 않는다.
  assert(report.status == np::ProcessStatus::Published && report.health_checks == 2);
  assert(primary.value == 2 && state->published.back().value == 1);
  // 빈 큐에서도 스냅샷을 반환하고 Missing을 표현한다.
  Components::Buffer empty(8);
  Components::Assembler assembler;
  assert(!assembler.assemble(empty, {}).data.get<0>().present());
  std::cout << "node_pipeline template guide\n";
}
