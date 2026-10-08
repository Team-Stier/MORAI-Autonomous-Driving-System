#pragma once
#include <lidar_leveler/libraries/cloud_transform.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>

namespace lidar_leveler {
// 장착 변환 뒤 차량 roll·pitch를 제거한다. p_level = Ry(pitch)·Rx(roll)·(R_mount·p_raw + t_mount).
// 차량 자세 R_world_body = Rz(yaw)·Ry(pitch)·Rx(roll)에서 yaw만 남긴 수평 좌표로 옮기는 회전이다.
// 출력은 base_link와 원점·yaw를 공유하고 roll·pitch가 0인 좌표이며, 점군 배치와 다른 필드는 보존한다.
class RollPitchLeveler {
 public:
  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud,
                                  const interfaces::CompetitionVehicleStatus& competition,
                                  const tf2_msgs::TFMessage& tf_static) {
    tf2::Quaternion leveling;
    leveling.setRPY(competition.roll_rad, competition.pitch_rad, 0.0);
    leveling.normalize();
    const tf2::Transform transform =
      tf2::Transform(leveling) * mountingTransform(tf_static, cloud.header.frame_id);
    return transformCloud(cloud, transform);
  }
};
}  // namespace lidar_leveler
