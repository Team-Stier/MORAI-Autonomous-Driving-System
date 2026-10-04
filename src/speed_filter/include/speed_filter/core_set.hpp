#pragma once
#include <speed_filter/types.hpp>
#include <speed_filter/cores/speed_limit60.hpp>
#include <speed_filter/cores/no_speed_limit.hpp>
namespace speed_filter {
enum class CoreId { SpeedLimit60, NoSpeedLimit };
struct CoreSet {
  SpeedLimit60 speed_limit60;
  NoSpeedLimit no_speed_limit;
};
}  // namespace speed_filter
