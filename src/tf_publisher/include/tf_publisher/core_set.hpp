#pragma once
#include <tf_publisher/types.hpp>
#include <tf_publisher/cores/ego_tf.hpp>
namespace tf_publisher {
enum class CoreId { EgoTf };
struct CoreSet {
  EgoTf ego_tf;
};
}  // namespace tf_publisher
