#pragma once
#include <interfaces/TrafficLight.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace traffic_light {
// 모델 클래스가 뜻하는 점등 등이다.
enum class Lamp : std::uint8_t { Red, Yellow, Green, Left };

inline unsigned lampBit(Lamp lamp) { return 1U << static_cast<unsigned>(lamp); }

// 동시에 검출된 점등 등의 집합을 TrafficLight 조합 enum으로 바꾼다. 빈 집합은 UNKNOWN이다.
// 현재 클래스 체계는 소등을 검출하지 않으므로 OFF를 만들지 않는다.
inline std::uint8_t combinationState(unsigned lamps) {
  using Light = interfaces::TrafficLight;
  const unsigned red = lampBit(Lamp::Red), yellow = lampBit(Lamp::Yellow);
  const unsigned green = lampBit(Lamp::Green), left = lampBit(Lamp::Left);
  const std::array<std::uint8_t, 16> states{{
      Light::UNKNOWN,           // -
      Light::RED,               // R
      Light::YELLOW,            // Y
      Light::RED_YELLOW,        // R Y
      Light::GREEN,             // G
      Light::RED_GREEN,         // R G
      Light::YELLOW_GREEN,      // Y G
      Light::RED_YELLOW_GREEN,  // R Y G
      Light::LEFT,              // L
      Light::RED_LEFT,          // R L
      Light::YELLOW_LEFT,       // Y L
      Light::RED_YELLOW_LEFT,   // R Y L
      Light::GREEN_LEFT,        // G L
      Light::RED_GREEN_LEFT,    // R G L
      Light::YELLOW_GREEN_LEFT, // Y G L
      Light::RED_YELLOW_GREEN_LEFT}};
  static_assert(Light::RED_YELLOW_GREEN_LEFT == 16, "TrafficLight combination enum changed");
  return states[lamps & (red | yellow | green | left)];
}

// 최근 history_size 프레임의 상태(UNKNOWN 포함)에서 최다 득표 상태를 고른다.
// 최다 득표가 minimum_votes 미만이면 UNKNOWN, 동률이면 가장 최근에 나타난 상태다.
// 1 <= minimum_votes <= history_size를 전제한다.
class StateVoter {
 public:
  StateVoter(std::size_t history_size, std::size_t minimum_votes)
      : history_size_(history_size), minimum_votes_(minimum_votes) {}

  std::uint8_t push(std::uint8_t state) {
    history_.push_back(state);
    while (history_.size() > history_size_) history_.pop_front();
    std::array<std::size_t, kStates> counts{};
    for (const auto item : history_) ++counts[item % kStates];
    const std::size_t maximum = *std::max_element(counts.begin(), counts.end());
    if (maximum < minimum_votes_) return interfaces::TrafficLight::UNKNOWN;
    for (auto item = history_.rbegin(); item != history_.rend(); ++item)
      if (counts[*item % kStates] == maximum) return *item;
    return interfaces::TrafficLight::UNKNOWN;
  }

 private:
  static constexpr std::size_t kStates = interfaces::TrafficLight::RED_YELLOW_GREEN_LEFT + 1;
  std::size_t history_size_;
  std::size_t minimum_votes_;
  std::deque<std::uint8_t> history_;
};
}  // namespace traffic_light
