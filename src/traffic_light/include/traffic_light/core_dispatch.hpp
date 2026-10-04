#pragma once
#include <traffic_light/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace traffic_light {
// TODO: 등록 Core의 필수 입력 인덱스와 process 인자를 case에 연결한다.
inline bool processCore(CoreSet&, CoreId, const node_pipeline::StageData&,
                        node_pipeline::CoreResult&, const node_pipeline::BeforeCore&) {
  throw std::logic_error("no registered core");
}
}  // namespace traffic_light
