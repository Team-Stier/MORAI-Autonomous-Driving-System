// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
#pragma once
#include <cstdint>
#include <string>
namespace pipeline_template {
// standalone upstream의 실제 메시지 타입이다. ROS 템플릿에서는 ROS 메시지를 직접 사용한다.
struct DemoMessage { double value = 0; std::int64_t stamp_ns = 0; std::string frame = "map"; };
// TODO: 구독·발행할 실제 메시지 타입을 사용한다. 별도 계산 결과 래퍼를 만들지 않는다.
}
