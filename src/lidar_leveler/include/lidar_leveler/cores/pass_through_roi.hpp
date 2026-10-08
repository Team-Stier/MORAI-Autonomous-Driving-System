#pragma once
#include <lidar_leveler/libraries/cloud_transform.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

namespace lidar_leveler {
struct PassThroughRoiParameters {
  double z_min_m = -0.5;
  double z_max_m = 4.5;
};

// 장착 변환 뒤 base_link z가 [z_min_m, z_max_m]인 점만 1행 점군으로 남긴다.
class PassThroughRoi {
 public:
  explicit PassThroughRoi(PassThroughRoiParameters parameters) : parameters_(parameters) {}
  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud, const tf2_msgs::TFMessage& tf_static) {
    return transformCloud(cloud, mountingTransform(tf_static, cloud.header.frame_id), true,
                          parameters_.z_min_m, parameters_.z_max_m);
  }

 private:
  PassThroughRoiParameters parameters_;
};
}  // namespace lidar_leveler
