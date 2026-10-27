#pragma once
#include <node_pipeline/extension.hpp>

namespace localization {
// 모든 issue 입력을 제거한다. 선언 입력이 하나라도 없으면 Core를 호출하지 않는다.
struct RemoveAll {
  int priority(node_pipeline::Kind) const { return 100; }
  void correct(node_pipeline::EditableInput& input, node_pipeline::Kind) const { input.remove(); }
};
}  // namespace localization
