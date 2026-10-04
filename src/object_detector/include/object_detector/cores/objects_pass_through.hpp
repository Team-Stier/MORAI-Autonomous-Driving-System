#pragma once
#include <interfaces/Objects.h>

namespace object_detector {
class ObjectsPassThrough {
 public:
  interfaces::Objects process(const interfaces::Objects& objects) {
    return objects;
  }
};
}  // namespace object_detector
