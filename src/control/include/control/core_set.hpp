#pragma once
#include <control/types.hpp>
#include <control/cores/path_tracking.hpp>
namespace control {
enum class CoreId { PathTracking };
struct CoreSet {
  PathTracking path_tracking;
  explicit CoreSet(tracking::Parameters parameters = {}) : path_tracking(std::move(parameters)) {}
};
}
