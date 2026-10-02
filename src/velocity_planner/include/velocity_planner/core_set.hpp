#pragma once
#include <velocity_planner/types.hpp>
#include <velocity_planner/cores/plain_velocity60.hpp>
#include <velocity_planner/cores/plain_velocity100.hpp>
#include <array>
#include <stdexcept>
namespace velocity_planner {
enum class CoreId { PlainVelocity60, PlainVelocity100 };
struct CoreSet {
  PlainVelocity60 plain_velocity60;
  PlainVelocity100 plain_velocity100;
  std::array<std::size_t, 2> requiredInputs(CoreId id) const {
    switch (id) {
      case CoreId::PlainVelocity60:
      case CoreId::PlainVelocity100: return {{0, 4}};
    }
    throw std::invalid_argument("unknown core");
  }
  interfaces::Path process(CoreId id, const Data& data) {
    switch (id) {
      case CoreId::PlainVelocity60:
        return plain_velocity60.process(data.get<0>().message(), data.get<4>().message());
      case CoreId::PlainVelocity100:
        return plain_velocity100.process(data.get<0>().message(), data.get<4>().message());
    }
    throw std::invalid_argument("unknown core");
  }
};
}  // namespace velocity_planner
