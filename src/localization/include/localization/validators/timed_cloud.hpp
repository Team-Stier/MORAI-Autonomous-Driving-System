#pragma once
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <cstdint>
#include <vector>

namespace localization {
// 점군 Core가 전제하는 배치를 검사한다. little-endian, 행·데이터 경계, 단일 FLOAT32 x·y·z·time 필드다.
struct TimedCloud {
  void operator()(const sensor_msgs::PointCloud2& cloud, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    bool valid = !cloud.is_bigendian && cloud.point_step > 0 &&
      static_cast<std::uint64_t>(cloud.width) * cloud.point_step <= cloud.row_step &&
      static_cast<std::uint64_t>(cloud.height) * cloud.row_step <= cloud.data.size();
    for (const char* name : {"x", "y", "z", "time"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [name](const sensor_msgs::PointField& value) { return value.name == name; });
      valid = valid && field != cloud.fields.end() && field->datatype == sensor_msgs::PointField::FLOAT32 &&
        field->count == 1 && static_cast<std::uint64_t>(field->offset) + sizeof(float) <= cloud.point_step;
    }
    if (!valid) issues.push_back(node_pipeline::Kind::Missing);
  }
};
}  // namespace localization
