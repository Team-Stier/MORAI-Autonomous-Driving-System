#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <tuple>
#include <utility>

namespace object_detector {
struct ZRoiParameters {
  double z_min_m = 0.2;
  double z_max_m = 2.0;
};

class ZRoi {
 public:
  explicit ZRoi(ZRoiParameters parameters) : parameters_(parameters) {}

  std::tuple<sensor_msgs::PointCloud2, interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const sensor_msgs::PointCloud2& cloud, const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& tf) {
    const auto field_offset = [&](const char* name) {
      return std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const sensor_msgs::PointField& field) { return field.name == name; })->offset;
    };
    const std::uint32_t offsets[] = {field_offset("x"), field_offset("y"), field_offset("z")};
    auto output = cloud;
    std::uint32_t retained = 0;
    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        const auto* record = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
                                               static_cast<std::size_t>(col) * cloud.point_step;
        float xyz[3];
        for (std::size_t axis = 0; axis < 3; ++axis)
          std::memcpy(&xyz[axis], record + offsets[axis], sizeof(float));
        // Ioniq5 base_link collider bounds from spec/ioniq5_bbox.meta.json.
        const bool inside_vehicle =
          xyz[0] >= -0.7899996042251587 && xyz[0] <= 3.845008373260498 &&
          xyz[1] >= -1.0804176330566406 && xyz[1] <= 1.080417275428772 &&
          xyz[2] >= -0.3684999644756317 && xyz[2] <= 1.2755463123321533;
        if (!inside_vehicle && xyz[2] >= parameters_.z_min_m && xyz[2] <= parameters_.z_max_m) {
          std::memcpy(output.data.data() + static_cast<std::size_t>(retained) * cloud.point_step,
                      record, cloud.point_step);
          ++retained;
        }
      }
    }
    output.height = 1;
    output.width = retained;
    output.row_step = retained * output.point_step;
    output.data.resize(output.row_step);
    return std::make_tuple(std::move(output), competition, tf);
  }

 private:
  ZRoiParameters parameters_;
};
}  // namespace object_detector
