#pragma once
#include <section_annotator/cores/checkpoint_sections.hpp>
namespace section_annotator {
enum class CoreId { CheckpointSections };
struct CoreSet {
  CheckpointSections checkpoint_sections;
};
}  // namespace section_annotator
