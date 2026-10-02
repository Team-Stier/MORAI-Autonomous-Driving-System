#pragma once
#include <control/types.hpp>
#include <control/cores/path_tracking.hpp>
namespace control {
enum class CoreId { PathTracking };
struct CoreSet {
  PathTracking path_tracking;
  explicit CoreSet(tracking::Parameters parameters = {}) : path_tracking(std::move(parameters)) {}
  std::array<std::size_t, 3> requiredInputs(CoreId) const { return {{0, 1, 2}}; }
  morai_msgs::CtrlCmd process(CoreId, const Data& data) {
    return path_tracking.process(data.get<0>().message(), data.get<1>().message(), data.get<2>().message());
  }
};
}
