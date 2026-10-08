#pragma once
#include <interfaces/Objects.h>

namespace object_detector {
// 검출 객체를 추적하지 않고 그대로 넘긴다. id는 0, 속도·가속도는 미추정이다.
class ObjectsPassThrough {
 public:
  interfaces::Objects process(const interfaces::Objects& objects) {
    return objects;
  }
};
}  // namespace object_detector
