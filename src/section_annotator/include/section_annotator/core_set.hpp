#pragma once
#include <section_annotator/cores/checkpoint_sections.hpp>
namespace section_annotator {
enum class CoreId { CheckpointSections };
struct CoreSet {
  CheckpointSections checkpoint_sections;
  std::array<std::size_t, 1> requiredInputs(CoreId) const { return {{0}}; }
  Message process(CoreId, const Data& data) {
    return checkpoint_sections.process(data.get<0>().message());
  }
};
}  // namespace section_annotator
