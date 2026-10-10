#pragma once
#include <path_planner/libraries/types.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <vector>

namespace path_planner {
struct CorePathInputs {
  void operator()(const interfaces::Objects& objects, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& object : objects.objects) {
      if (!object.velocity_available) {
        issues.push_back(node_pipeline::Kind::Missing);
        continue;
      }
      for (double value : {object.velocity.x, object.velocity.y, object.velocity.z}) {
        if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
        else if (!std::isfinite(value)) issues.push_back(node_pipeline::Kind::Inf);
      }
    }
  }
  void operator()(const PredictedObjects& predictions, const node_pipeline::HealthLimits& limits,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& sample : predictions) (*this)(sample, limits, issues);
  }
  void operator()(const Route& route, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (!route) issues.push_back(node_pipeline::Kind::Missing);
  }
};
}  // namespace path_planner
