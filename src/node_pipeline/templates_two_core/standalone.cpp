// 가이드 전용: catkin 실행·빌드 대상이 아니다. ROS 없는 공개 API와 자체 점검 예제다.
// 자체 점검: c++ -std=c++14 -I src/node_pipeline/include src/node_pipeline/templates_two_core/standalone.cpp -o /tmp/node_pipeline_two_core_check && /tmp/node_pipeline_two_core_check
#include "template_components.hpp"
#include <cassert>
#include <iostream>
namespace np = node_pipeline;
namespace ex = pipeline_two_core_template;
using Components = ex::Components<ex::DemoMessage, ex::DemoAccess>;
struct State { np::Clocks clocks{1000000000, 1000000000}; std::vector<ex::DemoMessage> published; };
struct IO {
  std::shared_ptr<State> state;
  np::Clocks readClocks() const { return state->clocks; }
  void publish(const ex::DemoMessage& result) { state->published.push_back(result); }
  void reportHealth(const np::Health<ex::Input, 2>&, std::size_t) const {}
  void report(const np::ProcessReport&) const noexcept {}
};
int main() {
  // 두 단계 모두 선택되어야 실행할 수 있다. health 매핑은 단계별로 덮어쓴다.
  auto selector = Components::selector();
  np::Health<ex::Input, 2> health;
  const auto selected = selector.select(0, health);
  assert(selected.selected && selected.id.first == ex::FirstCoreId::First && selected.id.second == ex::SecondCoreId::Second);
  selector.core2.section_core_map.clear();
  assert(!selector.select(0, health).selected);
  selector.core2.health_core_map = {{{{ex::Input::Primary, np::Kind::Missing}}, 100, ex::SecondCoreId::Second}};
  health.inputs[0].issues = {np::Kind::Missing};
  assert(selector.select(0, health).selected);
  selector.core1.section_core_map.clear();
  assert(!selector.select(0, health).selected);
  selector.core1.health_core_map = {{{{ex::Input::Primary, np::Kind::Missing}}, 100, ex::FirstCoreId::First}};
  assert(selector.select(0, health).selected);
  auto state = std::make_shared<State>();
  IO io{state};
  np::HealthLimits limits;
  Components::Node<IO, ex::DemoHeaderInjector> node(
    {np::ExecutionMode::OnInput, ex::Input::Primary, 10}, Components::Buffer(8), {},
    Components::Checker(limits, state->clocks.steady_ns), Components::policy(limits),
    Components::selector(), {}, {}, io);
  ex::DemoMessage primary{2, state->clocks.ros_ns, "map"};
  // 필수 보조 입력이 없으면 두 Core와 발행을 건너뛴다.
  assert(node.onInput(ex::Primary{}, primary).status == np::ProcessStatus::NoCore);
  assert(state->published.empty());
  ex::DemoMessage auxiliary{0.5, state->clocks.ros_ns, "map"};
  node.onInput(ex::Auxiliary{}, auxiliary);
  const auto report = node.onInput(ex::Primary{}, primary);
  // policy가 primary를 1로 보정한다. Core1: 두 메시지 {1, 0.5}, Core2: (1 + 0.5) * 0.5.
  assert(report.status == np::ProcessStatus::Published && report.health_checks == 2);
  assert(state->published.size() == 1 && state->published.back().value == 0.75);
  assert(primary.value == 2 && auxiliary.value == 0.5);
  assert(state->published.back().stamp_ns == state->clocks.ros_ns);
  assert(state->published.back().frame == "map");
  // Core1은 실제 메시지 두 개를 반환하고 발행 헤더는 주입하지 않는다.
  ex::FirstCore<ex::DemoMessage, ex::DemoAccess> first;
  const auto data = first.process(primary, auxiliary);
  assert(data.first.value == 2 && data.second.value == 0.5);
  assert(data.first.stamp_ns == 0 && data.second.stamp_ns == 0);
  // 둘 중 한 단계라도 현재 section에 매핑이 없으면 최종 결과를 발행하지 않는다.
  node.onSection(np::Section{true, 1, state->clocks.ros_ns, "map"});
  assert(node.onInput(ex::Primary{}, primary).status == np::ProcessStatus::NoCore);
  assert(state->published.size() == 1);
  std::cout << "two-core template check passed\n";
}
