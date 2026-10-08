#pragma once
#include <interfaces/Objects.h>

namespace object_detector {
struct ObjectsPassThroughParameters {};
class ObjectsPassThrough {
 public:
  explicit ObjectsPassThrough(ObjectsPassThroughParameters = {}) {}
  interfaces::Objects process(const interfaces::Objects& objects) {
    return objects;
  }
};
}  // namespace object_detector
