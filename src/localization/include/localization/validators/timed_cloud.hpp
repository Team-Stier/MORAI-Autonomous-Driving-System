#pragma once
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <cstdint>
#include <vector>

namespace localization {
struct TimedCloud {
  void operator()(const sensor_msgs::PointCloud2& cloud, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (cloud.is_bigendian || cloud.point_step == 0 ||
        static_cast<std::uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
        static_cast<std::uint64_t>(cloud.height) * cloud.row_step > cloud.data.size()) {
      issues.push_back(node_pipeline::Kind::Missing);
      return;
    }
    for (const char* name : {"x", "y", "z", "time"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [name](const sensor_msgs::PointField& value) { return value.name == name; });
      if (field == cloud.fields.end() || field->datatype != sensor_msgs::PointField::FLOAT32 ||
          field->count != 1 || static_cast<std::uint64_t>(field->offset) + sizeof(float) > cloud.point_step) {
        issues.push_back(node_pipeline::Kind::Missing);
        return;
      }
    }
  }
};
}  // namespace localization
