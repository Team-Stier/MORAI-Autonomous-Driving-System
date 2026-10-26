#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <vector>

namespace speed_filter {
struct SpeedLimit59Inputs {
  void operator()(const interfaces::CompetitionVehicleStatus& status, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (std::isnan(status.signed_velocity_mps)) issues.push_back(node_pipeline::Kind::NaN);
    else if (std::isinf(status.signed_velocity_mps)) issues.push_back(node_pipeline::Kind::Inf);
  }
};
}  // namespace speed_filter
