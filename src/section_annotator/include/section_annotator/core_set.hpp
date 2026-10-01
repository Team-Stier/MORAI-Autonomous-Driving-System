#pragma once
#include <section_annotator/types.hpp>
namespace section_annotator {
// TODO: 담당 Core 파일을 include하고 ID·필수 입력·process 인자를 등록한다.
enum class CoreId {};
struct CoreSet {

  std::array<std::size_t, 0> requiredInputs(CoreId) const { return {}; }
  interfaces::SectionAnnotation process(CoreId, const Data&) {
    throw std::logic_error("no registered core");
  }
};
}  // namespace section_annotator
