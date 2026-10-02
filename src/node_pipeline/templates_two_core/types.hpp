// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 메시지 타입으로 구체화한다.
#pragma once
#include <cstdint>
#include <string>
namespace pipeline_two_core_template {
// standalone 입력·중간 결과·최종 결과에 사용하는 메시지다.
struct DemoMessage { double value = 0; std::int64_t stamp_ns = 0; std::string frame = "map"; };
}
