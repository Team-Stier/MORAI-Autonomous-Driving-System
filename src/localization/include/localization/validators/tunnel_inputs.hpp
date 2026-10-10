#pragma once
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace localization {
struct TunnelInputs {
  void operator()(const sensor_msgs::PointCloud2& cloud, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (cloud.is_bigendian || cloud.point_step == 0 ||
        static_cast<std::uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
        static_cast<std::uint64_t>(cloud.height) * cloud.row_step > cloud.data.size()) {
      issues.push_back(node_pipeline::Kind::Missing);
      return;
    }
    std::array<std::uint32_t, 4> offsets{};
    const std::array<const char*, 4> names{{"x", "y", "z", "time"}};
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const sensor_msgs::PointField& value) { return value.name == names[i]; });
      if (field == cloud.fields.end() || field->datatype != sensor_msgs::PointField::FLOAT32 ||
          field->count != 1 || static_cast<std::uint64_t>(field->offset) + sizeof(float) > cloud.point_step) {
        issues.push_back(node_pipeline::Kind::Missing);
        return;
      }
      offsets[i] = field->offset;
    }
    for (std::uint32_t row = 0; row < cloud.height; ++row)
      for (std::uint32_t col = 0; col < cloud.width; ++col)
        for (const auto offset : offsets) {
          float value;
          std::memcpy(&value, cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
                                  static_cast<std::size_t>(col) * cloud.point_step + offset, sizeof(value));
          if (std::isnan(value)) { issues.push_back(node_pipeline::Kind::NaN); return; }
          if (std::isinf(value)) { issues.push_back(node_pipeline::Kind::Inf); return; }
        }
  }
};
}  // namespace localization
