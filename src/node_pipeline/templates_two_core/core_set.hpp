// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 Core와 메시지 인자를 연결한다.
#pragma once
#include <array>
#include <node_pipeline/components.hpp>
#include <stdexcept>
#include "first_core.hpp"
#include "second_core.hpp"
namespace pipeline_two_core_template {
enum class FirstCoreId { First };
enum class SecondCoreId { Second };
struct CoreIds { FirstCoreId first{}; SecondCoreId second{}; };
template<class Message, class Access> struct CoreSet {
  // Core 객체는 호출 사이에도 유지한다. 계산은 각 Core 파일에 둔다.
  FirstCore<Message, Access> first;
  SecondCore<Message, Access> second;
  // 첫 단계가 사용하는 구독 입력을 모두 선언한다. 검사는 Pipeline이 수행한다.
  std::array<std::size_t, 2> requiredInputs(CoreIds ids) const {
    switch (ids.first) {
      case FirstCoreId::First: return {{0, 1}};
    }
    throw std::invalid_argument("unknown first core");
  }
  Message process(CoreIds ids, const node_pipeline::Data<Message, Message>& snapshot) {
    switch (ids.first) {
      case FirstCoreId::First: {
        const auto data = first.process(
          snapshot.template get<0>().message(), snapshot.template get<1>().message());
        switch (ids.second) {
          case SecondCoreId::Second: return second.process(data.first, data.second);
        }
        throw std::invalid_argument("unknown second core");
      }
    }
    throw std::invalid_argument("unknown first core");
  }
};
}
