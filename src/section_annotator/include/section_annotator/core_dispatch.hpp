#pragma once
#include <section_annotator/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace section_annotator {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::CheckpointSections:
      if (!data.hasInputs({0})) return false;
      before();
      result = node_pipeline::wrapResult(cores.checkpoint_sections.process(data.message<nav_msgs::Odometry>(0)));
      return true;
  }
  throw std::invalid_argument("unknown core");
}
}  // namespace section_annotator
