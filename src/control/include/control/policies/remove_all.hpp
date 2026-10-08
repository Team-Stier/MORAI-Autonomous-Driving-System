#pragma once
#include <node_pipeline/extension.hpp>

namespace control {
// 모든 issue 입력을 제거한다. 제거된 입력의 Missing은 safe_stop.hpp 선택 규칙에 사용한다.
struct RemoveAll {
  int priority(node_pipeline::Kind) const { return 100; }
  void correct(node_pipeline::EditableInput& input, node_pipeline::Kind) const { input.remove(); }
};
}  // namespace control
