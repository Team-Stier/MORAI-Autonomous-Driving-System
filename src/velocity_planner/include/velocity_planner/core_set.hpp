#pragma once
#include <velocity_planner/types.hpp>
#include <velocity_planner/cores/plain_velocity60.hpp>
#include <velocity_planner/cores/plain_velocity100.hpp>
namespace velocity_planner {
enum class CoreId { PlainVelocity60, PlainVelocity100 };
struct CoreSet {
  PlainVelocity60 plain_velocity60;
  PlainVelocity100 plain_velocity100;
};
}  // namespace velocity_planner
