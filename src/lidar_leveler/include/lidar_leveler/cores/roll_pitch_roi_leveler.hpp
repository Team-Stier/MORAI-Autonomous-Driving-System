#pragma once
#include <lidar_leveler/libraries/cloud_transform.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>

namespace lidar_leveler {
struct RollPitchRoiLevelerParameters {
  double z_min_m = -0.5;
  double z_max_m = 4.5;
};

// roll_pitch_leveler.hpp와 같은 수평화 뒤, 수평 좌표 z가 [z_min_m, z_max_m]인 점만 1행 점군으로 남긴다.
class RollPitchRoiLeveler {
 public:
  explicit RollPitchRoiLeveler(RollPitchRoiLevelerParameters parameters) : parameters_(parameters) {}
  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud,
                                  const interfaces::CompetitionVehicleStatus& competition,
                                  const tf2_msgs::TFMessage& tf_static) {
    tf2::Quaternion leveling;
    leveling.setRPY(competition.roll_rad, competition.pitch_rad, 0.0);
    const tf2::Transform transform =
      tf2::Transform(leveling) * mountingTransform(tf_static, cloud.header.frame_id);
    return transformCloud(cloud, transform, true, parameters_.z_min_m, parameters_.z_max_m);
  }

 private:
  RollPitchRoiLevelerParameters parameters_;
};
}  // namespace lidar_leveler
