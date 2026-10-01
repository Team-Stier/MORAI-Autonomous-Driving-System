#pragma once
#include <tf_publisher/types.hpp>
#include <tf_publisher/cores/ego_tf.hpp>
namespace tf_publisher {
enum class CoreId { EgoTf };
struct CoreSet {
  EgoTf ego_tf;
  std::array<std::size_t, 1> requiredInputs(CoreId) const { return {{0}}; }
  tf2_msgs::TFMessage process(CoreId, const Data& data) {
    return ego_tf.process(data.get<0>().message());
  }
};
}  // namespace tf_publisher
