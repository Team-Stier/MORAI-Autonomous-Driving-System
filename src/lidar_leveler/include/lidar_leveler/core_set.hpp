#pragma once
#include <lidar_leveler/types.hpp>
#include <lidar_leveler/cores/pass_through.hpp>
#include <lidar_leveler/cores/roll_pitch_leveler.hpp>
namespace lidar_leveler {
enum class CoreId { PassThrough, RollPitchLeveler };
struct CoreSet {
  PassThrough pass_through;
  RollPitchLeveler roll_pitch_leveler;
};
}  // namespace lidar_leveler
