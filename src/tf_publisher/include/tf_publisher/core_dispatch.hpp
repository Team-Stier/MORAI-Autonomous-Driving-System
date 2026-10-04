#pragma once
#include <tf_publisher/core_set.hpp>
#include <node_pipeline/stages.hpp>
#include <stdexcept>
namespace tf_publisher {
inline bool processCore(CoreSet& cores, CoreId id, const node_pipeline::StageData& data,
                        node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::EgoTf:
      if (!data.hasInputs({0})) return false;
      before();
      result = node_pipeline::wrapResult(cores.ego_tf.process(data.message<nav_msgs::Odometry>(0)));
      return true;
  }
  throw std::invalid_argument("unknown core");
}
}  // namespace tf_publisher
