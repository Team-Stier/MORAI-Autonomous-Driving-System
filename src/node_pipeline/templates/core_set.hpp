// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
#pragma once
#include <node_pipeline/components.hpp>
#include "scalar_core.hpp"
namespace pipeline_template {
// core ID는 노드별 계산 구현과 대응한다.
enum class CoreId { Primary, Auxiliary };
// 알고리즘 구현은 각 core 파일에 두고 여기서는 소유권과 호출만 연결한다.
template<class Message, class Access, class OutputMessage = Message> struct CoreSet {
  // TODO: 실제 Core 파일을 include하고 Core별 필수 입력과 인자를 연결한다.
  ScalarCore<Message, Access, OutputMessage> primary;
  ScalarCore<Message, Access, OutputMessage> auxiliary;
  // 입력 tuple 인덱스로 Core별 필수 인자를 선언한다. 검사는 라이브러리가 수행한다.
  std::array<std::size_t, 1> requiredInputs(CoreId id) const {
    return {{id == CoreId::Primary ? 0u : 1u}};
  }
  OutputMessage process(CoreId id, const node_pipeline::Data<Message, Message>& data) {
    // 원래 구독 메시지 타입을 각각의 인자로 전달한다.
    return id == CoreId::Primary ? primary.process(data.template get<0>().message())
                                : auxiliary.process(data.template get<1>().message());
  }
};
}
