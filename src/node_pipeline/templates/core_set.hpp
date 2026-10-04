// 가이드 전용: 실행·빌드 대상이 아니다. TODO: 실제 노드 계약으로 구체화한다.
#pragma once
#include <node_pipeline/components.hpp>
#include "scalar_core.hpp"
#include "auxiliary_core.hpp"
namespace pipeline_template {
enum class CoreId { Primary, Auxiliary };
// CoreSet은 지속 Core 객체만 소유한다.
template<class Message, class Access, class OutputMessage = Message> struct CoreSet {
  ScalarCore<Message, Access, OutputMessage> primary;
  AuxiliaryCore<Message, Access, OutputMessage> auxiliary;
};
}
