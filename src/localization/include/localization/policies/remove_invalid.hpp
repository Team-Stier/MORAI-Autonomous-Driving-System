#pragma once
#include <node_pipeline/sdk.hpp>

namespace localization { namespace policies {
struct RemoveInvalid {
  bool apply(node_pipeline::InputView& input, const node_pipeline::ProfileHealth& health) const {
    node_pipeline::Issue<std::string, node_pipeline::Kind> selected;
    const auto priority = [&input](const std::string& name, node_pipeline::Kind) {
      return input.present(name) ? 0 : -1;
    };
    if (!node_pipeline::highestPriorityIssue(health, priority, selected)) return false;
    input.remove(selected.input);
    return true;
  }
};
}}  // namespace localization::policies
