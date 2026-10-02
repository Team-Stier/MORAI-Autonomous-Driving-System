// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 Core 계산으로 구체화한다.
#pragma once
#include <utility>
namespace pipeline_two_core_template {
template<class Message, class Access> class FirstCore {
 public:
  // TODO: 실제 구독 메시지를 개별 인자로 받아 두 중간 메시지의 payload를 계산한다.
  std::pair<Message, Message> process(const Message& msg1, const Message& msg2) {
    Message data1, data2;
    Access::value(data1) = Access::value(msg1);
    Access::value(data2) = Access::value(msg2);
    return {std::move(data1), std::move(data2)};
  }
};
}
