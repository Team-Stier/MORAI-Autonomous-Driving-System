#pragma once
#include <path_planner/types.hpp>

namespace path_planner {
class PredictionPassThrough {
 public:
  using Result = PredictedObjects;

  // #DOWNSTREAM_CONTRACT
  // 원본 Objects의 모든 공간 필드는 header.frame_id 기준이다. production 입력은
  // base_link_level이며 이 Core는 좌표 변환이나 외삽 없이 객체와 header를 그대로
  // 10개 원소에 복사한다. 따라서 각 예측 원소도 같은 frame과 관측 stamp를 유지한다.
  // 같은 원점·yaw를 공유한다고 차체 base_link로 이름만 바꾸면 안 된다. LocalPath는
  // 현재 객체를 회피 계산에 사용하지 않으며, 향후 다른 frame을 사용하는 알고리즘이
  // 필요하면 자신의 입력 경계에서 관측 시각에 맞는 명시적인 변환을 수행해야 한다.
  Result process(const interfaces::Objects& objects,
                 const Route&) const {
    Result result;
    result.fill(objects);
    return result;
  }
};
}  // namespace path_planner
