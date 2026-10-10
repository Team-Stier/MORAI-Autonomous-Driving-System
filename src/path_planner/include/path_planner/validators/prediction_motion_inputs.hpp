#pragma once
#include <interfaces/Objects.h>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <vector>

namespace path_planner {
struct PredictionMotionInputs {
  void operator()(const interfaces::Objects& objects, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const auto& object : objects.objects) {
      if (!object.velocity_available || !object.acceleration_available)
        issues.push_back(node_pipeline::Kind::Missing);
      if (object.velocity_available) checkVector(object.velocity, issues);
      if (object.acceleration_available) checkVector(object.acceleration, issues);
    }
  }
 private:
  static void checkVector(const geometry_msgs::Vector3& vector, std::vector<node_pipeline::Kind>& issues) {
    for (double value : {vector.x, vector.y, vector.z}) {
      if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
      else if (!std::isfinite(value)) issues.push_back(node_pipeline::Kind::Inf);
    }
  }
};
}  // namespace path_planner
