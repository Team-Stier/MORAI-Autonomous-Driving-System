#pragma once
#include <traffic_light/types.hpp>
namespace traffic_light {
// TODO: 담당 Core 파일을 include하고 ID·필수 입력·process 인자를 등록한다.
enum class CoreId {};
struct CoreSet {
  std::array<std::size_t, 0> requiredInputs(CoreId) const { return {}; }
  interfaces::TrafficLight process(CoreId, const Data&) {
    throw std::logic_error("no registered core");
  }
};
}  // namespace traffic_light
