// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
#pragma once
#include "types.hpp"
namespace pipeline_template {
// 이 파일에는 계산 core 클래스 하나만 둔다. Message는 구독한 실제 메시지 타입이다.
template<class Message, class Access, class OutputMessage = Message> class AuxiliaryCore {
 public:
  // 큐 래퍼가 아닌 실제 메시지를 개별 인자로 받는다.
  // TODO: 실제 알고리즘과 입력별 process 인자, 계산 상태로 교체한다.
  OutputMessage process(const Message& message) {
    // TODO: 목표 발행 메시지의 payload를 계산한다. 헤더는 Core 밖에서 주입한다.
    OutputMessage output;
    Access::value(output) = Access::value(message);
    ++updates_;
    return output;
  }
 private:
  // 계산 호출 사이에도 유지되는 내부 상태다.
  std::uint64_t updates_ = 0;
};
}
