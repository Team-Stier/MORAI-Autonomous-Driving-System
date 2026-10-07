#pragma once
#include <lidar_leveler/types.hpp>
#include <node_pipeline/sdk.hpp>
namespace lidar_leveler { namespace validators {
struct PassThrough {
  void operator()(const node_pipeline::InputView& inputs, node_pipeline::ProfileHealth& health) const {
    if (!inputs.present("cloud")) return;
    const auto& cloud = inputs.get<sensor_msgs::PointCloud2>("cloud");
    bool layout = !cloud.is_bigendian && cloud.point_step > 0 &&
      static_cast<std::uint64_t>(cloud.width)*cloud.point_step <= cloud.row_step &&
      static_cast<std::uint64_t>(cloud.height)*cloud.row_step <= cloud.data.size();
    for (const char* name : {"x", "y", "z"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const sensor_msgs::PointField& field) { return field.name == name; });
      layout = layout && field != cloud.fields.end() && field->datatype == sensor_msgs::PointField::FLOAT32 &&
        field->count == 1 && static_cast<std::uint64_t>(field->offset)+sizeof(float) <= cloud.point_step;
    }
    if (!layout) health.issues("cloud").push_back(node_pipeline::Kind::Missing);
    if (!inputs.present("tf_static")) return;
    const auto& transforms = inputs.get<tf2_msgs::TFMessage>("tf_static").transforms;
    if (std::none_of(transforms.begin(), transforms.end(), [&](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "base_link" && transform.child_frame_id == cloud.header.frame_id;
      })) health.issues("tf_static").push_back(node_pipeline::Kind::Missing);
  }
};
}}
