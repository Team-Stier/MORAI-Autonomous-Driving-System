#pragma once
#include <node_pipeline/types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <string>

namespace lidar_leveler {
struct HeaderInjector {
  std::string frame = "base_link_level";
  void injectHeader(sensor_msgs::PointCloud2& message, const node_pipeline::Context<3>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = frame;
  }
};
}  // namespace lidar_leveler
