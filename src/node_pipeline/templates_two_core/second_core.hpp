// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 Core 계산으로 구체화한다.
#pragma once
namespace pipeline_two_core_template {
template<class Message, class Access> class SecondCore {
 public:
  // TODO: 두 중간 메시지를 개별 인자로 받아 최종 발행 메시지의 payload를 계산한다.
  Message process(const Message& data1, const Message& data2) {
    Message result;
    Access::value(result) = (Access::value(data1) + Access::value(data2)) * 0.5;
    return result;
  }
};
}
