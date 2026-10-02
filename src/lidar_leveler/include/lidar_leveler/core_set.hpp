#pragma once
#include <lidar_leveler/types.hpp>
#include <lidar_leveler/cores/pass_through.hpp>
#include <array>
#include <stdexcept>
namespace lidar_leveler {
enum class CoreId { PassThrough };
struct CoreSet {
  PassThrough pass_through;
  std::array<std::size_t, 2> requiredInputs(CoreId id) const {
    switch (id) {
      case CoreId::PassThrough: return {{0, 2}};
    }
    throw std::invalid_argument("unknown core");
  }
  sensor_msgs::PointCloud2 process(CoreId id, const Data& data) {
    switch (id) {
      case CoreId::PassThrough:
        return pass_through.process(data.get<0>().message(), data.get<2>().message());
    }
    throw std::invalid_argument("unknown core");
  }
};
}  // namespace lidar_leveler
