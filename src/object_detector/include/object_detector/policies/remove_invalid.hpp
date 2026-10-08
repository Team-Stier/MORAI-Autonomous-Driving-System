#pragma once
#include <node_pipeline/extension.hpp>

namespace object_detector {
// 값·stamp·frame 오류 입력을 제거한다. Stale·ReceiveTimeout·Skew는 보정하지 않는다.
struct RemoveInvalid {
  int priority(node_pipeline::Kind kind) const {
    switch (kind) {
      case node_pipeline::Kind::InvalidStamp:
      case node_pipeline::Kind::FutureStamp:
      case node_pipeline::Kind::FrameMismatch:
      case node_pipeline::Kind::NaN:
      case node_pipeline::Kind::Inf:
        return 200;
      case node_pipeline::Kind::OutRangePositive:
      case node_pipeline::Kind::OutRangeNegative:
        return 100;
      default:
        return -1;
    }
  }
  void correct(node_pipeline::EditableInput& input, node_pipeline::Kind) const { input.remove(); }
};
}  // namespace object_detector
