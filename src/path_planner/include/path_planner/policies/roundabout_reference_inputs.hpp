#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>

namespace path_planner {
// Correct only stationary GT speed noise. Genuine reverse motion and other
// invalid inputs retain the standard removal policy; the source is unchanged.
struct RoundaboutReferenceInputs {
  int priority(node_pipeline::Kind kind) const {
    switch (kind) {
      case node_pipeline::Kind::InvalidStamp:
      case node_pipeline::Kind::FutureStamp:
      case node_pipeline::Kind::FrameMismatch:
      case node_pipeline::Kind::NaN:
      case node_pipeline::Kind::Inf: return 200;
      case node_pipeline::Kind::OutRangePositive:
      case node_pipeline::Kind::OutRangeNegative: return 100;
      default: return -1;
    }
  }
  void correct(node_pipeline::EditableInput& input, node_pipeline::Kind kind) const {
    bool corrected = false;
    if (kind == node_pipeline::Kind::OutRangeNegative) {
      try {
        input.modify<interfaces::CompetitionVehicleStatus>([&](interfaces::CompetitionVehicleStatus& status) {
          if (std::isfinite(status.velocity_x_mps) &&
              status.velocity_x_mps >= -0.03 && status.velocity_x_mps < 0.0) {
            status.velocity_x_mps = 0.0;
            corrected = true;
          }
        });
      } catch (const std::invalid_argument&) {
        // This issue can also originate from object dimensions of another type.
      }
    }
    if (!corrected) input.remove();
  }
};
}  // namespace path_planner
