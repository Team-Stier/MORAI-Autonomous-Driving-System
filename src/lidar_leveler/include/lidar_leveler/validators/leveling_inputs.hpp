#pragma once
#include <node_pipeline/extension.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace lidar_leveler {
// 점군 수평화 Core가 전제하는 입력 내용을 검사한다.
struct LevelingInputs {
  // little-endian, 행·데이터 경계, 단일 FLOAT32 x·y·z 필드다.
  void operator()(const sensor_msgs::PointCloud2& cloud, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    bool valid = !cloud.is_bigendian && cloud.point_step > 0 &&
      static_cast<std::uint64_t>(cloud.width) * cloud.point_step <= cloud.row_step &&
      static_cast<std::uint64_t>(cloud.height) * cloud.row_step <= cloud.data.size();
    for (const char* name : {"x", "y", "z"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [name](const sensor_msgs::PointField& value) { return value.name == name; });
      valid = valid && field != cloud.fields.end() && field->datatype == sensor_msgs::PointField::FLOAT32 &&
        field->count == 1 && static_cast<std::uint64_t>(field->offset) + sizeof(float) <= cloud.point_step;
    }
    if (!valid) issues.push_back(node_pipeline::Kind::Missing);
  }
  // base_link → velodyne 장착 변환이 있어야 한다. 점군 frame은 node.yaml에서 velodyne으로 검사한다.
  void operator()(const tf2_msgs::TFMessage& tf_static, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (std::none_of(tf_static.transforms.begin(), tf_static.transforms.end(),
          [](const geometry_msgs::TransformStamped& transform) {
            return transform.header.frame_id == "base_link" && transform.child_frame_id == "velodyne";
          }))
      issues.push_back(node_pipeline::Kind::Missing);
  }
  // 수평화에 쓰는 roll·pitch가 유한해야 한다.
  void operator()(const interfaces::CompetitionVehicleStatus& status, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (const double value : {static_cast<double>(status.roll_rad), static_cast<double>(status.pitch_rad)}) {
      if (std::isnan(value)) issues.push_back(node_pipeline::Kind::NaN);
      else if (std::isinf(value)) issues.push_back(node_pipeline::Kind::Inf);
    }
  }
};
}  // namespace lidar_leveler
