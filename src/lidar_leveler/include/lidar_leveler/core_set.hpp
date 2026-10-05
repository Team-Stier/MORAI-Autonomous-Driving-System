#pragma once
#include <lidar_leveler/types.hpp>
#include <lidar_leveler/cores/pass_through.hpp>
namespace lidar_leveler {
enum class CoreId { PassThrough };
struct CoreSet {
  PassThrough pass_through;
};
}  // namespace lidar_leveler
