#pragma once
#include <node_pipeline/sdk.hpp>
#include <tf_publisher/cores/static_tf.hpp>
#include <cmath>
#include <numeric>

namespace tf_publisher { namespace parameters {
struct StaticTf {
  StaticTfParameters operator()(const node_pipeline::Parameters& parameters) const {
    StaticTfParameters value;
    for (const auto& child : parameters.require<std::vector<std::string>>("children")) {
      const auto mount = parameters.child(child);
      const auto translation = mount.require<std::vector<double>>("translation");
      const auto rotation = mount.require<std::vector<double>>("rotation");
      if (child.empty() || child == "base_link" || translation.size() != 3 || rotation.size() != 4)
        throw std::invalid_argument("invalid static transform entry");
      if (std::abs(std::inner_product(rotation.begin(), rotation.end(), rotation.begin(), 0.0) - 1) > 1e-6)
        throw std::invalid_argument("invalid static transform quaternion");
      value.transforms.push_back({child, {{translation[0], translation[1], translation[2]}},
        {{rotation[0], rotation[1], rotation[2], rotation[3]}}});
    }
    return value;
  }
};
}}  // namespace tf_publisher::parameters
