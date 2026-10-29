#pragma once
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <initializer_list>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace lidar_leveler {
struct MotionInputs {
  void operator()(const sensor_msgs::PointCloud2& cloud, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (cloud.is_bigendian || cloud.point_step == 0 ||
        static_cast<std::uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
        static_cast<std::uint64_t>(cloud.height) * cloud.row_step > cloud.data.size()) {
      issues.push_back(node_pipeline::Kind::Missing);
      return;
    }
    std::array<std::uint32_t, 3> offsets{};
    const std::array<const char*, 3> names{{"x", "y", "z"}};
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
    const auto field=std::find_if(cloud.fields.begin(),cloud.fields.end(),
        [](const sensor_msgs::PointField& f){return f.name=="time";});
    if(field==cloud.fields.end() || field->datatype!=sensor_msgs::PointField::FLOAT32 || field->count!=1 ||
        static_cast<std::uint64_t>(field->offset)+sizeof(float)>cloud.point_step) {
      issues.push_back(node_pipeline::Kind::Missing);return;
    }
    for(std::size_t row=0;row<cloud.height;++row) for(std::size_t col=0;col<cloud.width;++col) {
      float time;
      std::memcpy(&time,cloud.data.data()+row*cloud.row_step+col*cloud.point_step+field->offset,sizeof(float));
      if(std::isnan(time)) {issues.push_back(node_pipeline::Kind::NaN);return;}
      if(!std::isfinite(time)) {issues.push_back(node_pipeline::Kind::Inf);return;}
      if(time< -0.3 || time>0.02) {issues.push_back(node_pipeline::Kind::OutRangePositive);return;}
    }
  }
  void operator()(const tf2_msgs::TFMessage& transforms, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    const auto mount = std::find_if(transforms.transforms.begin(), transforms.transforms.end(),
      [](const geometry_msgs::TransformStamped& value) {
        return value.header.frame_id == "base_link" && value.child_frame_id == "velodyne";
      });
    if (mount == transforms.transforms.end()) { issues.push_back(node_pipeline::Kind::Missing); return; }
    const auto& t = mount->transform.translation;
    const auto& q = mount->transform.rotation;
    for (double value : {t.x, t.y, t.z, q.x, q.y, q.z, q.w}) {
      if (std::isnan(value)) { issues.push_back(node_pipeline::Kind::NaN); return; }
      if (std::isinf(value)) { issues.push_back(node_pipeline::Kind::Inf); return; }
    }
    if (std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 1e-6)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
  void operator()(const interfaces::CompetitionVehicleStatus& status, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    for (double value : {status.roll_rad, status.pitch_rad, status.yaw_rad, status.velocity_x_mps}) {
      if (std::isnan(value)) { issues.push_back(node_pipeline::Kind::NaN); return; }
      if (std::isinf(value)) { issues.push_back(node_pipeline::Kind::Inf); return; }
    }
  }
};
}  // namespace lidar_leveler
