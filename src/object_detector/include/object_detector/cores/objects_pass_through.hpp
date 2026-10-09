#pragma once
#include <interfaces/Objects.h>

namespace object_detector {
// #FRAME_CONTRACT
// detect의 DBSCAN+OBB Objects payload를 track Stage의 출력 타입 그대로 넘긴다.
// 실제 Tracking은 미구현이므로 ID 연속성·운동 추정·미래 위치를 만들지 않는다.
// 객체 중심·bbox 방향·크기·availability와 header를 복사하고 추가 ROI도 적용하지 않는다.
// 최종 stamp·base_link_level header는 공통 Pipeline이 원본 cloud context로 주입한다.
// tracking 개발 시 같은 입력 frame과 메시지 시각을 유지하고 frame 이름만 바꾸지 않는다.
class ObjectsPassThrough {
 public:
  interfaces::Objects process(const interfaces::Objects& objects) {
    return objects;
  }
};
}  // namespace object_detector
